// core/gas_system.h — 0D gas volume (inspired by Ange Yaghi's engine-sim, MIT)
#pragma once
#include <cmath>
#include <algorithm>

namespace ice {

constexpr double R_GAS = 8.314462618;       // J/(mol·K)
constexpr double P_ATM = 101325.0;          // Pa
constexpr double T_REF = 298.15;            // K
constexpr double GAMMA_AIR = 1.4;

// Simple gas mixture state: total moles n, internal energy E (J), volume V (m^3)
struct GasState {
    double n = 0;     // mol
    double V = 1e-3;  // m^3
    double E = 0;     // J internal (translational+rotational, dof/2 nRT)
    double dof = 5;  // degrees of freedom
    // mixture composition (molar fractions)
    double x_fuel = 0, x_o2 = 0, x_inert = 0;

    double pressure() const {
        return (2.0 / dof) * (E / std::max(V, 1e-12));
    }
    double temperature() const {
        return (2.0 / dof) * (E / std::max(n * R_GAS, 1e-12));
    }
    double mass() const {
        // approx molar mass: fuel 0.114 kg/mol, O2 0.032, inert(N2) 0.028
        return n * (x_fuel * 0.114 + x_o2 * 0.032 + x_inert * 0.028);
    }
    double soundSpeed() const {
        const double T = temperature();
        const double g = heatCapacityRatio();
        const double M = std::max((x_fuel * 0.114 + x_o2 * 0.032 + x_inert * 0.028), 1e-9);
        return std::sqrt(g * R_GAS * std::max(T, 1.0) / M);
    }
    double heatCapacityRatio() const {
        // gamma from dof (5 -> 1.4 approx for diatomic)
        return (dof + 2.0) / dof;
    }
    double density() const { return mass() / std::max(V, 1e-12); }
};

// Isoentropic flow between two volumes through a restriction with
// coefficient k (m^3/s at reference conditions, simplified from engine-sim's
// k_flow calibration). Returns dmol (mol transferred 0->1 this step).
double flowMoles(GasState &a, GasState &b, double k, double dt);

// Fractions of a mixture after moving dn mol from src to dst.
void transferMix(GasState &src, GasState &dst, double dn);

// Adiabatic-ish volume change with work transfer (poly via E conservation).
void setVolume(GasState &s, double newV);

} // namespace ice
