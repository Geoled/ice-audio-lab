// tests/headless.cpp — engine smoke test without UI/audio device
#include "../core/piston_engine.h"
#include "../core/config.h"
#include <cstdio>
#include <cmath>

using namespace ice;

int main(int argc, char **argv) {
    std::string path = argc > 1 ? argv[1] : "assets/engines/baseline_i4.json";
    EngineConfig cfg;
    std::string err;
    if (!loadEngineJson(path, cfg, err)) {
        fprintf(stderr, "config error: %s\n", err.c_str());
        return 1;
    }

    PistonEngine eng(cfg);
    eng.setIgnition(true);
    eng.setStarter(true);

    const double fSim = 18000;
    const double dt = 1.0 / fSim;
    const int stepsPerSec = (int)fSim;

    double taps[2] = {0, 0};
    double maxTap = 0, sumTap = 0;
    int tapCount = 0;

    // 40 seconds: 6s starter-only, then throttle, starter off at 15s
    for (int sec = 0; sec < 40; ++sec) {
        eng.setThrottle(sec >= 6 ? 1.0 : 0.0);
        if (sec >= 15) eng.setStarter(false);

        for (int i = 0; i < stepsPerSec; ++i) {
            eng.step(dt);
            eng.computeAudioTap(taps, 2);
            if (sec >= 3) {
                const double m = std::fabs(taps[1]);
                if (m > maxTap) maxTap = m;
                sumTap += m;
                ++tapCount;
            }
        }
        const auto &c0 = eng.cylinders()[0];
        printf("t=%2ds  rpm=%7.1f  torque_avg=%7.2f Nm  map=%6.1f kPa  tailTap=%9.4f  P0=%8.1fPa lit0=%d fuel0=%.5f\n",
               sec, eng.rpm(), eng.telemetry().torque_avg_nm,
               eng.telemetry().map_kpa, taps[1],
               c0.gas.pressure(), (int)c0.lit, c0.gas.x_fuel);
        fflush(stdout);
    }

    printf("\npeak |tail| = %.4f, mean |tail| = %.6f over %d samples\n",
           maxTap, tapCount ? sumTap / tapCount : 0.0, tapCount);

    const bool ok = eng.rpm() > 500 && maxTap > 1e-6;
    printf(ok ? "SMOKE TEST PASS\n" : "SMOKE TEST FAIL (engine dead or silent)\n");
    return ok ? 0 : 2;
}
