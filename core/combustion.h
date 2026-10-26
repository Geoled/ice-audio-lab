// core/combustion.h — flame event model (per cylinder)
#pragma once
#include "gas_system.h"

namespace ice {

// advances the flame through the chamber; returns energy released (J)
double combustStep(GasState &gas, double &flamePos, double &eff,
                   double dt, double rpm, double T_wall,
                   double rng01);

// Wiebe-style burn fraction helper (used for 2T/diesel later); S0 uses flame
// front propagation like engine-sim.
inline double laminarFlameSpeed(double T, double P) {
    // Metghalchi-Keck-ish: gasoline
    const double Tref = 298.0, Pref = 101325.0;
    const double base = 0.35; // m/s
    return base * std::pow(std::max(T, 200.0) / Tref, 2.18)
                 * std::pow(std::max(P, 1000.0) / Pref, -0.17);
}

} // namespace ice
