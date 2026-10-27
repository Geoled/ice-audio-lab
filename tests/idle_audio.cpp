// tests/idle_audio.cpp — verify the synth keeps producing sound at zero
// user throttle (idle governor + intake-noise layer). Runs the real synth
// against the engine and measures output RMS in steady idle.
#include "../core/piston_engine.h"
#include "../core/config.h"
#include "../audio/synth.h"
#include <cstdio>
#include <cmath>
#include <vector>

using namespace ice;

int main(int argc, char **argv) {
    std::string path = argc > 1 ? argv[1] : "assets/engines/baseline_i4.json";
    EngineConfig cfg;
    std::string err;
    if (!loadEngineJson(path, cfg, err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }

    PistonEngine eng(cfg);
    eng.setIgnition(true); eng.setStarter(true);
    eng.setThrottle(0.0);            // user gas released
    eng.setIdleControl(true);

    Synth::Params sp;
    sp.sample_rate = 48000;
    sp.jitter = cfg.audio.jitter;
    sp.air_noise = cfg.audio.air_noise;
    sp.hf_gain = cfg.audio.hf_gain;
    sp.volume = 1.0;
    Synth synth;
    synth.initialize(sp, 2);
    synth.setSampleRate(48000);
    synth.setSimRate(18000);
    synth.setEngineContext(0, 0, (int)cfg.cylinders.size());

    const double fSim = 18000, dt = 1.0 / fSim;
    const int stepsPerSec = (int)fSim;
    double taps[2] = {0, 0};

    // 12 s: crank 5 s, then idle with governor, starter off at 8 s
    for (int sec = 0; sec < 12; ++sec) {
        if (sec >= 8) eng.setStarter(false);
        for (int i = 0; i < stepsPerSec; ++i) {
            eng.step(dt);
            eng.computeAudioTap(taps, 2);
            synth.writeTaps(taps, 2, dt);
            synth.setEngineContext(eng.telemetry().rpm, eng.inductionNoise(),
                                   (int)cfg.cylinders.size());
        }
        printf("t=%2ds  rpm=%7.1f  idleThr=%.3f  MAP=%.1f kPa\n",
               sec, eng.rpm(), eng.telemetry().idleThrottle, eng.telemetry().map_kpa);
    }

    // measure audio RMS over the last 2 s (steady idle, user throttle 0)
    const int blockFrames = 480;
    std::vector<float> block(blockFrames * 2);
    double sumSq = 0;
    int nSamples = 0;
    double peak = 0;
    // feed sim + drain audio concurrently for 2 s worth of audio
    const int audioSteps = (int)(2.0 * 48000 / blockFrames);
    for (int b = 0; b < audioSteps; ++b) {
        // produce enough sim steps for this audio block
        const int need = (int)(blockFrames * (18000.0 / 48000.0));
        for (int i = 0; i < need; ++i) {
            eng.step(dt);
            eng.computeAudioTap(taps, 2);
            synth.writeTaps(taps, 2, dt);
            synth.setEngineContext(eng.telemetry().rpm, eng.inductionNoise(),
                                   (int)cfg.cylinders.size());
        }
        synth.renderBlock(block.data(), blockFrames);
        for (int i = 0; i < blockFrames; ++i) {
            const double l = block[i * 2 + 0];
            sumSq += l * l;
            peak = std::fmax(peak, std::fabs(l));
            ++nSamples;
        }
    }
    const double rms = std::sqrt(sumSq / std::max(nSamples, 1));
    printf("\nidle audio: RMS=%.4f  peak=%.4f  over %d samples\n", rms, peak, nSamples);

    // audible threshold: RMS > 0.005 (-46 dBFS) counts as clearly audible
    const bool ok = rms > 0.005 && eng.rpm() > 400;
    printf(ok ? "IDLE AUDIO PASS (engine idles and is audible)\n"
              : "IDLE AUDIO FAIL (silent at zero throttle)\n");
    return ok ? 0 : 2;
}
