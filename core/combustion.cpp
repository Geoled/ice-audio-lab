// core/combustion.cpp
#include "combustion.h"
#include <cmath>
#include <algorithm>

namespace ice {

double combustStep(GasState &gas, double &flamePos, double &eff,
                   double dt, double rpm, double T_wall, double rng01) {
    if (flamePos >= 1.0) return 0;

    const double T = gas.temperature();
    const double P = gas.pressure();

    // turbulence factor from rpm (mean piston speed ~ rpm)
    const double turb = 1.0 + rpm / 3000.0;
    double sL = laminarFlameSpeed(T, P) * turb;
    sL *= (0.7 + 0.3 * rng01);  // cycle variability

    // flame travels across characteristic length ~ sqrt(V) in 3D-ish way
    const double L = std::cbrt(std::max(gas.V, 1e-9)) * 1.2;
    const double dPos = sL * dt / std::max(L, 1e-3);
    const double newPos = std::min(flamePos + dPos, 1.0);
    const double dVfrac = newPos - flamePos;

    // burnable fuel in the swept fraction
    const double nFuel = gas.n * gas.x_fuel;
    const double burned = nFuel * dVfrac * eff;

    // stoichiometry: 25 O2 per 2 fuel (C8H16) — like engine-sim
    const double o2Needed = burned * 12.5;
    const double o2Avail = gas.n * gas.x_o2;
    const double actualBurn = std::min(burned, o2Needed > 0 ? std::min(o2Avail / 12.5, burned) : burned);

    if (actualBurn <= 0) {
        flamePos = newPos;
        return 0;
    }

    // energy release: 44 MJ/kg fuel, 0.114 kg/mol
    const double dE = actualBurn * 0.114 * 44e6 * eff;

    // adjust composition: fuel+O2 -> inert
    const double dnTotal = gas.n;
    const double fFuel = actualBurn / std::max(dnTotal, 1e-12);
    const double fO2 = (actualBurn * 12.5) / std::max(dnTotal, 1e-12);
    gas.x_fuel = std::max(0.0, gas.x_fuel - fFuel);
    gas.x_o2 = std::max(0.0, gas.x_o2 - fO2);
    // products add moles: 34/27 expansion
    gas.x_inert = std::min(1.0, gas.x_inert + fFuel * (34.0 / 2.0));
    gas.n += actualBurn * (34.0 / 2.0 - 13.5) * 0 + actualBurn * 5.0; // net +5 per mol fuel

    // wall heat loss
    const double dT_loss = (T_wall - T) * 0.02 * dt * 60.0;
    gas.E += dE + dT_loss * 0.5 * gas.dof * gas.n * R_GAS * 0.01;

    flamePos = newPos;
    return dE;
}

} // namespace ice
