// tests/release_trace.cpp — fine trace of the throttle-release phase to find
// why the engine sags/stalls in the GUI (idle governor state, fuel, spark).
#include "../core/piston_engine.h"
#include "../core/config.h"
#include <cstdio>
#include <cmath>

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

    const double fSim = 18000, dt = 1.0 / fSim;
    const int stepsPerSec = (int)fSim;

    // crank 5 s
    for (int s = 0; s < 5; ++s)
        for (int i = 0; i < stepsPerSec; ++i) eng.step(dt);
    eng.setStarter(false);
    // idle settle 5 s
    for (int s = 0; s < 5; ++s)
        for (int i = 0; i < stepsPerSec; ++i) eng.step(dt);
    printf("idle: rpm=%.0f idleThr=%.3f MAP=%.1f fuel=%.4f\n",
           eng.rpm(), eng.telemetry().idleThrottle, eng.telemetry().map_kpa,
           eng.cylinders()[0].gas.x_fuel);

    // WOT 3 s (hold W)
    eng.setThrottle(1.0);
    for (int s = 0; s < 3; ++s)
        for (int i = 0; i < stepsPerSec; ++i) eng.step(dt);
    printf("WOT end: rpm=%.0f idleThr=%.3f\n", eng.rpm(), eng.telemetry().idleThrottle);

    // release: 0.25-s trace for 12 s
    eng.setThrottle(0.0);
    for (int q = 0; q < 48; ++q) {
        for (int i = 0; i < stepsPerSec / 4; ++i) eng.step(dt);
        const auto &c0 = eng.cylinders()[0];
        printf("t=%5.2fs  rpm=%7.1f  idleThr=%.3f  MAP=%6.1f  fuel=%6.4f  sparks=%d/%d/%d/%d  lit=%d  P0=%7.1f\n",
               q * 0.25, eng.rpm(), eng.telemetry().idleThrottle,
               eng.telemetry().map_kpa, c0.gas.x_fuel,
               eng.sparkCounts()[0], eng.sparkCounts()[1], eng.sparkCounts()[2], eng.sparkCounts()[3],
               (int)c0.lit, c0.gas.pressure() / 1e5);
    }
    return 0;
}
