// tests/firststep.cpp — trace the very first engine steps to find the lockup
#include "../core/piston_engine.h"
#include "../core/config.h"
#include <cstdio>

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
    eng.setThrottle(1.0);

    const double fSim = 18000;
    const double dt = 1.0 / fSim;

    for (int i = 0; i < 12; ++i) {
        const auto &c0 = eng.cylinders()[0];
        printf("pre  step=%d  theta=%7.4f  rpm=%8.2f  P0=%10.1f  V0=%9.7f  E0=%9.4f n0=%9.6f intLift=%.2f exhLift=%.2f\n",
               i, eng.crankAngle(), eng.rpm(), c0.gas.pressure(), c0.gas.V,
               c0.gas.E, c0.gas.n, c0.intake_lift, c0.exhaust_lift);
        eng.step(dt);
    }
    const auto &c0 = eng.cylinders()[0];
    printf("post theta=%7.4f  rpm=%8.2f  P0=%10.1f  V0=%9.7f  E0=%9.4f\n",
           eng.crankAngle(), eng.rpm(), c0.gas.pressure(), c0.gas.V, c0.gas.E);

    // fine trace: every 250 steps for the first revolutions
    for (int i = 0; i < 12000; ++i) {
        eng.step(dt);
        if (i % 250 == 0) {
            const auto &c0 = eng.cylinders()[0];
            const auto &c1 = eng.cylinders()[1];
            printf("i=%6d  theta=%9.5f  rpm=%9.2f  sparks(c1..c4)=%d/%d/%d/%d  P0=%9.1f P1=%9.1f  lit=%d/%d  fuel=%.5f/%.5f\n",
                   i, eng.crankAngle(), eng.rpm(),
                   eng.sparkCounts()[0], eng.sparkCounts()[1], eng.sparkCounts()[2], eng.sparkCounts()[3],
                   c0.gas.pressure(), c1.gas.pressure(),
                   (int)c0.lit, (int)c1.lit, c0.gas.x_fuel, c1.gas.x_fuel);
        }
    }
    return 0;
}
