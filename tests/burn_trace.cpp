// tests/burn_trace.cpp — per-cycle energy audit at low rpm
#include "../core/piston_engine.h"
#include "../core/config.h"
#include <cstdio>
#include <cmath>

using namespace ice;

int main(int argc, char **argv) {
    std::string path = argc > 1 ? argv[1] : "assets/engines/baseline_i4.json";
    EngineConfig cfg; std::string err;
    if (!loadEngineJson(path, cfg, err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }

    PistonEngine eng(cfg);
    eng.setIgnition(true); eng.setStarter(true);
    eng.setThrottle(1.0); eng.setIdleControl(true);
    const double fSim = 18000, dt = 1.0 / fSim;

    for (int i = 0; i < 8 * (int)fSim; ++i) {
        if (i > 6 * (int)fSim) eng.setStarter(false);
        eng.step(dt);
    }
    printf("state: rpm=%.0f MAP=%.1f\n", eng.rpm(), eng.telemetry().map_kpa);

    // watch cylinder 1 across its compression TDC (theta crosses 2pi)
    int cycle = 0;
    for (int i = 0; i < (int)fSim; ++i) {
        const double thetaBefore = eng.crankAngle();
        const auto &c0 = eng.cylinders()[0];
        const double xfuelBefore = c0.gas.x_fuel;
        const double pBefore = c0.gas.pressure();
        const int sparksBefore = eng.sparkCounts()[0];
        eng.step(dt);
        if (thetaBefore < 2.0 * M_PI && eng.crankAngle() >= 2.0 * M_PI) {
            ++cycle;
            printf("TDC#%d: P(start of step)=%.1f bar  x_fuel=%.4f  lit-after=%d  "
                   "spark-delta=%d\n",
                   cycle, pBefore / 1e5, xfuelBefore, (int)c0.lit,
                   eng.sparkCounts()[0] - sparksBefore);
        }
    }
    return 0;
}
