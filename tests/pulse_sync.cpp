// tests/pulse_sync.cpp — verify each combustion event produces exactly one
// collector-tap pulse, and measure pulse spacing vs firing interval.
#include "../core/piston_engine.h"
#include "../core/config.h"
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace ice;

int main(int argc, char **argv) {
    std::string path = argc > 1 ? argv[1] : "assets/engines/baseline_i4.json";
    EngineConfig cfg;
    std::string err;
    if (!loadEngineJson(path, cfg, err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }

    PistonEngine eng(cfg);
    eng.setIgnition(true); eng.setStarter(true);
    const double thr = argc > 2 ? atof(argv[2]) : 1.0;
    eng.setThrottle(thr);
    const double fSim = 18000, dt = 1.0 / fSim;

    for (int i = 0; i < 10 * (int)fSim; ++i) {
        if (i > 6 * (int)fSim) eng.setStarter(false);
        eng.step(dt);
    }
    printf("# steady rpm=%.0f (throttle %.0f%%)\n", eng.rpm(), thr * 100);

    // record: collector signal, and per-cyl combustion start events (lit rising)
    std::vector<double> sig;
    std::vector<int> litEvents;  // cylinder index at combustion start
    sig.reserve(4 * (int)fSim);
    litEvents.reserve(64);
    int prevLit[8] = {0,0,0,0,0,0,0,0};
    const double T = 2.0;
    for (int i = 0; i < (int)(T * fSim); ++i) {
        eng.step(dt);
        sig.push_back(eng.exhaust().collectorSignal());
        for (int c = 0; c < eng.cylinderCount(); ++c) {
            const int lit = eng.cylinders()[c].lit ? 1 : 0;
            if (lit && !prevLit[c]) litEvents.push_back(i);
            prevLit[c] = lit;
        }
    }

    // combustion events: expect nCyl * rpm/120 * T
    const double expectedEvents = (double)eng.cylinderCount() * eng.rpm() / 120.0 * T;
    printf("combustion events: %d (expected ~%.0f)\n", (int)litEvents.size(), expectedEvents);

    // pulse peaks in collector signal (adaptive threshold: 50% of max)
    double mxSig = 0;
    for (double v : sig) mxSig = std::fmax(mxSig, v);
    const double thresh = 0.5 * mxSig;
    int peaks = 0;
    std::vector<int> peakIdx;
    for (size_t i = 1; i + 1 < sig.size(); ++i) {
        if (sig[i] > sig[i-1] && sig[i] > sig[i+1] && sig[i] > thresh) {
            ++peaks;
            peakIdx.push_back((int)i);
        }
    }
    printf("collector peaks: %d (threshold %.0f Pa)\n", peaks, thresh);

    // peak spacing stats
    if (peakIdx.size() > 2) {
        double mn = 1e9, mx = 0, sum = 0;
        for (size_t i = 1; i < peakIdx.size(); ++i) {
            const double g = (peakIdx[i] - peakIdx[i-1]) / fSim * 1000.0;
            mn = std::fmin(mn, g); mx = std::fmax(mx, g); sum += g;
        }
        printf("peak gaps ms: min=%.2f avg=%.2f max=%.2f (n=%d)\n",
               mn, sum / (peakIdx.size() - 1), mx, (int)peakIdx.size() - 1);
    }
    // event spacing
    if (litEvents.size() > 2) {
        double mn = 1e9, mx = 0, sum = 0;
        for (size_t i = 1; i < litEvents.size(); ++i) {
            const double g = (litEvents[i] - litEvents[i-1]) / fSim * 1000.0;
            mn = std::fmin(mn, g); mx = std::fmax(mx, g); sum += g;
        }
        printf("event gaps ms: min=%.2f avg=%.2f max=%.2f (n=%d)\n",
               mn, sum / (litEvents.size() - 1), mx, (int)litEvents.size() - 1);
    }
    return 0;
}
