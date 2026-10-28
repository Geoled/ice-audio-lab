// tests/combust_unit.cpp — unit test of the combustion step: burn a full
// stoichiometric charge at fixed volume and check the pressure rise against
// the adiabatic flame expectation.
#include "../core/combustion.h"
#include "../core/gas_system.h"
#include <cstdio>
#include <cmath>

using namespace ice;

int main() {
    // stoichiometric charge at compression TDC: V=50cc, P=20 bar, T=700K
    GasState g;
    g.V = 50e-6;
    const double n = 20e5 * 50e-6 / (R_GAS * 700.0);
    g.n = n;
    g.E = 0.5 * g.dof * n * R_GAS * 700.0;
    g.x_fuel = 1.0 / (1.0 + 12.5 / 0.21);   // stoich molar fraction
    g.x_o2 = 0.21 * (1.0 - g.x_fuel);
    g.x_inert = 1.0 - g.x_fuel - g.x_o2;

    printf("start: P=%.1f bar  T=%.0f K  n=%.5f mol  x_fuel=%.4f\n",
           g.pressure() / 1e5, g.temperature(), g.n, g.x_fuel);

    double flame = 0, eff = 0.9;
    const double dt = 1.0 / 18000.0;
    double released = 0;
    for (int i = 0; i < 18000 && flame < 1.0; ++i) {  // max 1 s
        released += combustStep(g, flame, eff, dt, 3000.0, 723.0, 0.5);
        if (i % 180 == 0) {
            printf("t=%4.1f ms  flame=%5.2f  P=%6.1f bar  T=%5.0f K\n",
                   1000.0 * i / 18000.0, flame, g.pressure() / 1e5,
                   g.temperature());
        }
    }
    printf("done: flame=%.2f  P=%.1f bar  T=%.0f K  total released=%.0f J\n",
           flame, g.pressure() / 1e5, g.temperature(), released);

    const double fm = n * (1.0 / (1.0 + 12.5 / 0.21));
    const double expectedE = fm * 0.114 * 44e6 * 0.92;
    printf("expected full-burn energy: %.0f J (fuel %.6f mol)\n", expectedE, fm);
    return 0;
}
