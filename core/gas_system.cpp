// core/gas_system.cpp
#include "gas_system.h"
#include <cassert>

namespace ice {

double flowMoles(GasState &a, GasState &b, double k, double dt) {
    const double Pa = a.pressure(), Pb = b.pressure();
    if (std::abs(Pa - Pb) < 1e-6) return 0;
    if (k <= 0) return 0;

    const bool forward = Pa > Pb;
    GasState &src = forward ? a : b;
    GasState &dst = forward ? b : a;

    const double P0 = src.pressure();
    const double P1 = dst.pressure();
    const double ratio = P1 / std::max(P0, 1e-6);
    const double g = src.heatCapacityRatio();
    const double T0 = std::max(src.temperature(), 200.0);

    double mdot;  // mol/s through restriction
    const double crit = std::pow(2.0 / (g + 1.0), g / (g - 1.0));
    if (ratio <= crit) {
        // choked flow
        mdot = k * P0 / std::sqrt(R_GAS * T0) *
               std::sqrt(g) * std::pow(2.0 / (g + 1.0), (g + 1.0) / (2.0 * (g - 1.0)));
    } else {
        const double term = std::pow(ratio, 2.0 / g) - std::pow(ratio, (g + 1.0) / g);
        mdot = k * P0 / std::sqrt(R_GAS * T0) *
               std::sqrt((2.0 * g / (g - 1.0)) * std::max(term, 0.0));
    }

    double dn = mdot * dt;
    if (dn <= 0) return 0;

    // energy per mol of the source charge
    const double e_per_mol = src.E / std::max(src.n, 1e-12);

    // never move more than what is available, and never overshoot
    // pressure equilibrium between the two volumes
    dn = std::min(dn, 0.5 * src.n);
    if (dn <= 0) return 0;

    // equilibrium cap: dn such that src.P == dst.P afterwards (approx)
    // P = 2E/(dof*V); moving dn mol at e_per_mol:
    //   src: E' = E - e*dn, n' = n - dn, P_src' = 2(E-e*dn)/(dof*(n-dn)R... ) via T
    // simplified: use pressure-equivalent moles
    const double srcP = src.pressure(), dstP = dst.pressure();
    const double T_src = src.temperature();
    // moles that would equalize pressures if T stayed:
    const double dnEq = (srcP - dstP) * dst.V /
                        (R_GAS * std::max(0.5 * (T_src + dst.temperature()), 200.0));
    if (dnEq > 0) dn = std::min(dn, dnEq);
    if (dn <= 0) return 0;

    // --- transfer: composition, moles, energy (single authoritative move) ---
    const double sn = src.n;
    dst.x_fuel  = (dst.x_fuel  * dst.n + src.x_fuel  * dn) / std::max(dst.n + dn, 1e-12);
    dst.x_o2    = (dst.x_o2    * dst.n + src.x_o2    * dn) / std::max(dst.n + dn, 1e-12);
    dst.x_inert = (dst.x_inert * dst.n + src.x_inert * dn) / std::max(dst.n + dn, 1e-12);
    dst.n += dn;
    dst.E += e_per_mol * dn;

    src.n = sn - dn;
    src.E -= e_per_mol * dn;
    if (src.E < 0) src.E = 0;
    // normalize source fractions (they are ratios, renormalize defensively)
    const double sum = src.x_fuel + src.x_o2 + src.x_inert;
    if (sum > 1.0 + 1e-9) {
        src.x_fuel /= sum; src.x_o2 /= sum; src.x_inert /= sum;
    }
    src.sanitize();
    dst.sanitize();

    const double dir = forward ? 1.0 : -1.0;
    return dir * dn;
}

void transferMix(GasState &src, GasState &dst, double dn) {
    // kept for compatibility; the logic now lives in flowMoles
    const double e_per_mol = src.E / std::max(src.n, 1e-12);
    dst.x_fuel  = (dst.x_fuel  * dst.n + src.x_fuel  * dn) / std::max(dst.n + dn, 1e-12);
    dst.x_o2    = (dst.x_o2    * dst.n + src.x_o2    * dn) / std::max(dst.n + dn, 1e-12);
    dst.x_inert = (dst.x_inert * dst.n + src.x_inert * dn) / std::max(dst.n + dn, 1e-12);
    dst.n += dn;
    dst.E += e_per_mol * dn;
    src.n -= dn;
    src.E -= e_per_mol * dn;
}

void setVolume(GasState &s, double newV) {
    // E stays; compression heating comes from the piston work term in the engine.
    s.V = std::max(newV, 1e-9);
}

} // namespace ice
