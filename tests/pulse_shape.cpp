// tests/pulse_shape.cpp — dump the collector/tail taps over 1 second at
// steady WOT to inspect pulse shape, spacing and DC offset
#include "../core/piston_engine.h"
#include "../core/config.h"
#include <cstdio>
#include <cstdlib>

using namespace ice;

int main(int argc, char **argv) {
    std::string path = argc > 1 ? argv[1] : "assets/engines/baseline_i4.json";
    const double thr = argc > 2 ? atof(argv[2]) : 1.0;
    EngineConfig cfg;
    std::string err;
    if (!loadEngineJson(path, cfg, err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }

    PistonEngine eng(cfg);
    eng.setIgnition(true); eng.setStarter(true); eng.setThrottle(thr);
    const double fSim = 18000, dt = 1.0 / fSim;

    for (int i = 0; i < 10 * (int)fSim; ++i) {
        if (i > 6 * (int)fSim) eng.setStarter(false);
        eng.step(dt);
    }
    printf("# steady rpm=%.0f thr=%.2f\n# t_ms  collector  tail  P0\n", eng.rpm(), thr);

    double mnC = 1e30, mxC = -1e30, sumC = 0; int nC = 0;
    double mnT = 1e30, mxT = -1e30, sumT = 0; int nT = 0;
    for (int i = 0; i < (int)(0.5 * fSim); ++i) {
        eng.step(dt);
        const double c = eng.exhaust().collectorSignal();
        const double t = eng.exhaust().tailSignal();
        mnC = std::fmin(mnC, c); mxC = std::fmax(mxC, c); sumC += c; ++nC;
        mnT = std::fmin(mnT, t); mxT = std::fmax(mxT, t); sumT += t; ++nT;
        if (i % 3 == 0) {
            printf("%.4f  %9.1f  %9.1f  %9.1f\n",
                   1000.0 * i / fSim, c, t, eng.cylinders()[0].gas.pressure());
        }
    }
    printf("# collector: min=%.1f max=%.1f mean=%.1f\n", mnC, mxC, sumC / nC);
    printf("# tail:      min=%.1f max=%.1f mean=%.1f\n", mnT, mxT, sumT / nT);
    return 0;
}
