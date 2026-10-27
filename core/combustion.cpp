// core/combustion.cpp
#include "combustion.h"
#include <cmath>
#include <algorithm>

namespace ice {

double combustStep(GasState &gas, double &flamePos, double &eff,
                   double dt, double rpm, double T_wall, double rng01) {
    if (flamePos >= 1.0) return 0;
    if (gas.x_fuel <= 0) { flamePos = 1.0; return 0; }

    const double T = gas.temperature();
    const double P = gas.pressure();

    // turbulence factor from rpm (mean piston speed proxy)
    const double turb = 1.0 + rpm / 3000.0;
    double sL = laminarFlameSpeed(T, P) * turb;
    sL *= (0.7 + 0.3 * rng01);  // cycle variability

    // flame front sweeps the characteristic chamber length
    const double L = std::cbrt(std::max(gas.V, 1e-9)) * 1.2;
    const double dPos = sL * dt / std::max(L, 1e-3);
    const double newPos = std::min(flamePos + dPos, 1.0);
    const double dVfrac = newPos - flamePos;
    if (dVfrac <= 0) return 0;

    // fuel available in the newly-swept fraction
    const double nFuel = gas.n * gas.x_fuel;
    const double burned = nFuel * dVfrac * eff;

    // stoichiometry: C8H16 + 12.5 O2 -> 8 CO2 + 8 H2O
    const double o2Avail = gas.n * gas.x_o2;
    const double o2Needed = burned * 12.5;
    const double actualBurn = std::min(burned, o2Avail / 12.5);
    if (actualBurn <= 0) {
        flamePos = newPos;
        return 0;
    }

    // energy release: 44 MJ/kg, 0.114 kg/mol -> ~5.0 MJ/mol fuel
    const double dE = actualBurn * 0.114 * 44e6;

    // composition update (molar)
    // reactants: 1 fuel + 12.5 O2 = 13.5 mol in; products: 16 mol out
    const double nReact = actualBurn * 13.5;
    const double nProd  = actualBurn * 16.0;
    const double dnNet  = nProd - nReact;

    gas.n += dnNet;
    gas.x_fuel = std::max(0.0, gas.x_fuel - actualBurn / std::max(gas.n, 1e-12));
    gas.x_o2   = std::max(0.0, gas.x_o2   - o2Needed / std::max(gas.n, 1e-12));
    gas.x_inert = std::min(1.0, gas.x_inert + nProd / std::max(gas.n, 1e-12));

    // wall heat loss (T_wall in K assumed; small relaxation toward wall)
    const double nCv = 0.5 * gas.dof * gas.n * R_GAS; // J/K
    const double dT_loss = (T_wall - T) * 0.02 * dt * 60.0;
    gas.E += dE + dT_loss * nCv * 0.01;

    flamePos = newPos;
    return dE;
}

} // namespace ice
