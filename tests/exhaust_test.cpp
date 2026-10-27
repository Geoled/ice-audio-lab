// tests/exhaust_test.cpp — direct probe of the 1D exhaust network
#include "../core/exhaust1d.h"
#include <cstdio>
#include <cmath>

using namespace ice;

int main() {
    Exhaust1D::Config cfg;
    cfg.cylinders = 4;
    cfg.primary_L_m = 0.65;
    cfg.primary_d_m = 0.040;
    cfg.tail_L_m = 0.80;
    cfg.tail_d_m = 0.050;
    cfg.nodes_per_m = 12;

    Exhaust1D ex;
    ex.initialize(cfg);

    const double dt = 1.0 / 18000;
    double ports[4] = {P_ATM, P_ATM, P_ATM, P_ATM};

    // inject a strong 3 kPa pulse into port 0 for 200 steps (~11 ms)
    double lifts[4] = {1, 1, 1, 1};
    for (int i = 0; i < 4000; ++i) {
        ports[0] = (i < 200) ? P_ATM + 3000.0 : P_ATM;
        ports[1] = P_ATM; ports[2] = P_ATM; ports[3] = P_ATM;
        ex.step(dt, ports, nullptr, 4, lifts);
        if (i % 100 == 0 && i < 1200) {
            const auto &p0 = ex.primarySeg(0);
            printf("i=%5d  nodes:", i);
            for (int k = 0; k < (int)p0.nodes.size(); ++k) {
                printf(" %7.1f", p0.nodes[k].p - P_ATM);
            }
            printf("  | tail h=%7.1f e=%7.1f | col=%8.2f tail_sig=%9.4f\n",
                   ex.tailSeg().nodes.front().p - P_ATM,
                   ex.tailSeg().nodes.back().p - P_ATM,
                   ex.collectorSignal(), ex.tailSignal());
        }
    }
    printf("\n--- sustained blowdown 50 kPa (tail signal) ---\n");
    double maxTail = 0;
    for (int i = 0; i < 8000; ++i) {
        ports[0] = P_ATM + 50000.0;
        ex.step(dt, ports, nullptr, 4, lifts);
        if (std::abs(ex.tailSignal()) > maxTail) maxTail = std::abs(ex.tailSignal());
        if (i % 800 == 0) {
            printf("i=%5d  col=%9.2f  tail=%9.4f\n", i, ex.collectorSignal(), ex.tailSignal());
        }
    }
    printf("peak |tail| in blowdown = %.2f Pa\n", maxTail);
    return 0;
}
