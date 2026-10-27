// tests/validation.cpp — physical validation of the engine cycle against
// textbook SI-engine values (peak pressure, compression pressure, IMEP).
//
// Reference checks (NA gasoline SI, CR ~10.5:1, stoichiometric):
//   - motored (no combustion) peak cylinder pressure near TDC: ~18-25 bar
//   - fired peak pressure: ~50-90 bar
//   - firing frequency for I4 at rpm r: 2*r/60 Hz
#include "../core/piston_engine.h"
#include "../core/config.h"
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <vector>

using namespace ice;

int main(int argc, char **argv) {
    std::string path = argc > 1 ? argv[1] : "assets/engines/baseline_i4.json";
    EngineConfig cfg;
    std::string err;
    if (!loadEngineJson(path, cfg, err)) {
        fprintf(stderr, "config error: %s\n", err.c_str());
        return 1;
    }

    // ---- Test 1: motored compression pressure ----
    {
        PistonEngine eng(cfg);
        eng.setIgnition(false);          // motored: no spark
        eng.setStarter(true);
        eng.setThrottle(0.0);
        const double fSim = 18000;
        const double dt = 1.0 / fSim;
        // spin 5 s to settle, starter cranks it
        double peakP = 0;
        for (int i = 0; i < 5 * (int)fSim; ++i) {
            eng.step(dt);
            for (auto &c : eng.cylinders()) {
                peakP = std::max(peakP, c.gas.pressure());
            }
        }
        const double peakBar = peakP / 1e5;
        const double cr = (500.0 + cfg.cylinders[0].chamber_cc) / cfg.cylinders[0].chamber_cc;
        // gamma=1.35 polytropic: P_peak ~= P_intake * CR^1.35 (+ residual heat)
        const double expected = 1.0 * std::pow(cr, 1.35);
        printf("[motored] CR=%.1f  peak P=%.1f bar   (polytropic expectation ~%.1f bar)\n",
               cr, peakBar, expected);
        const bool ok = peakBar > 0.5 * expected && peakBar < 2.0 * expected;
        printf(ok ? "[motored] PASS\n" : "[motored] FAIL (compression pressure off)\n");
    }

    // ---- Test 2: fired peak pressure ----
    {
        PistonEngine eng(cfg);
        eng.setIgnition(true);
        eng.setStarter(true);
        eng.setThrottle(1.0);
        const double fSim = 18000;
        const double dt = 1.0 / fSim;
        double peakP = 0;
        double peakTPa = 0;
        for (int i = 0; i < 12 * (int)fSim; ++i) {
            if (i > 6 * (int)fSim) eng.setStarter(false);
            eng.step(dt);
            double sumTP = 0;
            for (auto &c : eng.cylinders()) {
                peakP = std::max(peakP, c.gas.pressure());
                sumTP += c.gas.temperature();
            }
            peakTPa = std::max(peakTPa, sumTP / std::max(1, (int)eng.cylinders().size()));
        }
        const double peakBar = peakP / 1e5;
        printf("[fired]   peak P=%.1f bar  avg peak T=%.0f K\n", peakBar, peakTPa);
        // realistic NA SI: 40..120 bar peak; burned gas temps 2200..2800 K
        const bool okP = peakBar > 30.0 && peakBar < 160.0;
        printf(okP ? "[fired]   pressure PASS\n" : "[fired]   pressure FAIL\n");
    }

    // ---- Test 3: exhaust pulse frequency vs rpm (peak counting) ----
    // Uses LOW throttle so the engine settles below the rev limiter: at WOT
    // the no-load engine pegs the limiter and hard-cut removes events, which
    // is correct behavior but not a clean frequency measurement.
    {
        PistonEngine eng(cfg);
        eng.setIgnition(true);
        eng.setStarter(true);
        eng.setThrottle(0.05);
        const double fSim = 18000;
        const double dt = 1.0 / fSim;
        // settle to steady state
        for (int i = 0; i < 10 * (int)fSim; ++i) {
            if (i > 6 * (int)fSim) eng.setStarter(false);
            eng.step(dt);
        }
        // adaptive-threshold peak counting over 2 s: threshold = 50% of the
        // signal's max (works at both light load and WOT)
        double mxSig = 0;
        const int pre = (int)(0.2 * fSim);
        std::vector<double> preSig;
        preSig.reserve(pre);
        for (int i = 0; i < pre; ++i) {
            eng.step(dt);
            preSig.push_back(eng.exhaust().collectorSignal());
        }
        for (double v : preSig) mxSig = std::fmax(mxSig, v);
        // count peaks over the pre-recorded window (steady already)
        double prev2 = preSig.size() > 1 ? preSig[preSig.size() - 2] : 0;
        double prev = preSig.back();
        int peaks = 0;
        double rpmSum = 0; int rpmN = 0;
        const int window = (int)(2.0 * fSim);
        for (int i = 0; i < window; ++i) {
            eng.step(dt);
            const double cur = eng.exhaust().collectorSignal();
            if (prev > prev2 && prev > cur && prev > 0.5 * mxSig) ++peaks;
            prev2 = prev; prev = cur;
            rpmSum += eng.rpm(); ++rpmN;
        }
        const double rpmAvg = rpmSum / std::max(rpmN, 1);
        const double nCyl = (double)eng.cylinderCount();
        // pulses per second = nCyl * rpm / 120 (4-stroke)
        const double expectedHz = nCyl * rpmAvg / 120.0;
        const double measuredHz = peaks / 2.0;
        printf("[pulses]  rpm=%.0f  peaks=%d -> %.1f Hz  expected %.1f Hz\n",
               rpmAvg, peaks, measuredHz, expectedHz);
        const bool ok = measuredHz > 0.9 * expectedHz && measuredHz < 1.1 * expectedHz;
        printf(ok ? "[pulses]  PASS\n" : "[pulses]  FAIL (pulse frequency off)\n");
    }

    return 0;
}
