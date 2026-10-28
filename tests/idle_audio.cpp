// tests/idle_audio.cpp — verify the synth keeps producing sound at zero
// user throttle (idle governor + intake-noise layer) AND that releasing the
// gas from WOT never produces a silence gap: RMS is measured over sliding
// windows through the decel and idle phases.
#include "../core/piston_engine.h"
#include "../core/config.h"
#include "../audio/synth.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>

using namespace ice;

int main(int argc, char **argv) {
    std::string path = argc > 1 ? argv[1] : "assets/engines/baseline_i4.json";
    EngineConfig cfg;
    std::string err;
    if (!loadEngineJson(path, cfg, err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }

    PistonEngine eng(cfg);
    eng.setIgnition(true); eng.setStarter(true);
    eng.setThrottle(0.0);
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
    const int blockFrames = 480;
    std::vector<float> block(blockFrames * 2);

    // helper: run sim+audio for N seconds at a given throttle, collect
    // per-100ms window RMS
    auto runPhase = [&](double seconds, double throttle, const char *label) {
        eng.setThrottle(throttle);
        const int blocks = (int)(seconds * 48000 / blockFrames);
        std::vector<double> rmsWindows;
        for (int b = 0; b < blocks; ++b) {
            const int need = (int)(blockFrames * (fSim / 48000.0));
            for (int i = 0; i < need; ++i) {
                eng.step(dt);
                double taps[2];
                eng.computeAudioTap(taps, 2);
                synth.writeTaps(taps, 2, dt);
                synth.setEngineContext(eng.telemetry().rpm, eng.inductionNoise(),
                                       (int)cfg.cylinders.size());
            }
            synth.renderBlock(block.data(), blockFrames);
            double sumSq = 0;
            for (int i = 0; i < blockFrames; ++i) sumSq += (double)block[i * 2] * block[i * 2];
            rmsWindows.push_back(std::sqrt(sumSq / blockFrames));
        }
        double mn = 1e30, mx = 0;
        for (double v : rmsWindows) { mn = std::fmin(mn, v); mx = std::fmax(mx, v); }
        printf("%-22s  rpm=%7.1f  rms(min=%.4f avg-of-last=%.4f max=%.4f)\n",
               label, eng.rpm(), mn,
               rmsWindows.empty() ? 0.0 : rmsWindows[rmsWindows.size() - 1], mx);
        return mn;
    };

    // start: crank + settle to idle
    runPhase(6.0, 0.0, "crank+idle");
    // stress: repeated WOT -> release cycles like a user blipping the gas
    double worstGap = 1.0;
    for (int cycle = 0; cycle < 4; ++cycle) {
        char label[64];
        snprintf(label, sizeof(label), "WOT#%d", cycle + 1);
        runPhase(3.0, 1.0, label);
        snprintf(label, sizeof(label), "release#%d", cycle + 1);
        double mn = runPhase(6.0, 0.0, label);
        worstGap = std::fmin(worstGap, mn);
        printf("        after cycle %d: rpm=%.0f idleThr=%.3f\n",
               cycle + 1, eng.rpm(), eng.telemetry().idleThrottle);
    }
    const double gapMin = worstGap;
    // steady idle again
    const double idleMin = runPhase(4.0, 0.0, "steady idle");

    const bool ok = gapMin > 0.003 && idleMin > 0.005 && eng.rpm() > 400;
    printf("\nrelease-phase min RMS=%.4f, idle min RMS=%.4f, rpm=%.0f\n",
           gapMin, idleMin, eng.rpm());
    printf(ok ? "IDLE AUDIO PASS (no silence gap on throttle release)\n"
              : "IDLE AUDIO FAIL (silence gap on throttle release)\n");
    return ok ? 0 : 2;
}
