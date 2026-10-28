// core/combustion.cpp
#include "combustion.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ice {

double combustStep(GasState &gas, double &flamePos, double &eff,
                   double dt, double rpm, double T_wall, double rng01) {
    if (flamePos >= 1.0) return 0;
    if (gas.x_fuel <= 0) { flamePos = 1.0; return 0; }

    const double T = gas.temperature();
    const double P = gas.pressure();

    // turbulence factor: real SI engines complete the burn within ~40-60 deg
    // of crank at ANY speed, which needs turbulent flame speeds of ~8-12x
    // laminar (the flame wrinkle factor grows with engine speed).
    const double turb = 8.0 + std::min(rpm, 7000.0) / 2000.0;
    // Metghalchi-Keck expects UNBURNED gas temperature; clamp the correlation
    // input to the physically sensible unburned range.
    const double Teff = std::clamp(T, 500.0, 1100.0);
    double sL = laminarFlameSpeed(Teff, P) * turb;
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

    // energy release: 44 MJ/kg, 0.114 kg/mol -> ~5.0 MJ/mol fuel.
    // Combustion efficiency <1: incomplete burn, dissociation at high T
    // (real NA SI converts ~90-95% of the fuel LHV into gas internal energy).
    const double dE = actualBurn * 0.114 * 44e6 * 0.92;

    // composition update (molar)
    // reactants: 1 fuel + 12.5 O2 = 13.5 mol in; products: 16 mol out
    const double nReact = actualBurn * 13.5;
    const double nProd  = actualBurn * 16.0;
    const double dnNet  = nProd - nReact;

    gas.n += dnNet;
    gas.x_fuel = std::max(0.0, gas.x_fuel - actualBurn / std::max(gas.n, 1e-12));
    gas.x_o2   = std::max(0.0, gas.x_o2   - o2Needed / std::max(gas.n, 1e-12));
    gas.x_inert = std::min(1.0, gas.x_inert + nProd / std::max(gas.n, 1e-12));

    // *** release the heat into the gas ***
    gas.E += dE;

    // ---- wall heat loss (Woschni-style Newton model) ----
    // Q = h*A*(Tg-Tw), applied only while the flame front is sweeping the
    // chamber (the burn window), which is a fraction of the cycle. Total
    // wall losses in a real SI engine: ~15% of the fuel energy.
    {
        const double bore = std::cbrt(gas.V) * 0.9;          // approx from V
        const double area = M_PI * bore * bore;              // crown+head
        const double h = 500.0;                              // W/m2K, peak
        const double qLoss = h * area * std::max(T - T_wall, 0.0);
        gas.E -= qLoss * dt * 0.20;                          // burn-window share
    }

    // energy ceiling: effective adiabatic flame temperature of gasoline-air
    // with dissociation is ~2400 K; clamp the gas energy there to bound
    // peak pressure at a realistic 60-100 bar.
    {
        const double Tmax = 2400.0;
        const double Emax = 0.5 * gas.dof * gas.n * R_GAS * Tmax;
        if (gas.E > Emax) gas.E = Emax;
    }
    gas.sanitize();

    flamePos = newPos;
    return dE;
}

} // namespace ice
