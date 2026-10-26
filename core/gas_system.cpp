// core/gas_system.cpp
#include "gas_system.h"
#include <cassert>

namespace ice {

double flowMoles(GasState &a, GasState &b, double k, double dt) {
    const double Pa = a.pressure(), Pb = b.pressure();
    if (std::abs(Pa - Pb) < 1e-9) return 0;

    const bool forward = Pa > Pb;
    GasState &src = forward ? a : b;
    GasState &dst = forward ? b : a;

    const double P0 = src.pressure();
    const double P1 = dst.pressure();
    const double ratio = P1 / std::max(P0, 1e-6);
    const double g = src.heatCapacityRatio();
    const double T0 = std::max(src.temperature(), 1.0);

    double mdot;  // mol/s through restriction
    const double crit = std::pow(2.0 / (g + 1.0), g / (g - 1.0));
    if (ratio <= crit) {
        // choked
        mdot = k * P0 / std::sqrt(R_GAS * T0) *
               std::sqrt(g) * std::pow(2.0 / (g + 1.0), (g + 1.0) / (2.0 * (g - 1.0)));
    } else {
        const double term = std::pow(ratio, 2.0 / g) - std::pow(ratio, (g + 1.0) / g);
        mdot = k * P0 / std::sqrt(R_GAS * T0) *
               std::sqrt((2.0 * g / (g - 1.0)) * std::max(term, 0.0));
    }

    double dn = mdot * dt;

    // energy per mol of source
    const double e_per_mol = src.E / std::max(src.n, 1e-12);

    // do not move more than available or more than equilibrates pressure
    const double n_eq = (dst.pressure() >= src.pressure())
        ? 0.0
        : std::min(src.n,
            (src.pressure() - dst.pressure()) * dst.V * dst.dof / (2.0 * e_per_mol) );
    dn = std::min(dn, 0.9 * src.n);
    dn = std::min(dn, n_eq);
    if (dn <= 0) return 0;

    transferMix(src, dst, dn);
    src.n -= dn;
    src.E -= e_per_mol * dn;

    const double dir = forward ? 1.0 : -1.0;
    return dir * dn;
}

void transferMix(GasState &src, GasState &dst, double dn) {
    const double f = dn / std::max(src.n, 1e-12);
    const double sf = std::clamp(1.0 - f, 0.0, 1.0);
    const double e_per_mol = src.E / std::max(src.n, 1e-12);

    dst.x_fuel  = (dst.x_fuel  * dst.n + src.x_fuel  * dn) / std::max(dst.n + dn, 1e-12);
    dst.x_o2    = (dst.x_o2    * dst.n + src.x_o2    * dn) / std::max(dst.n + dn, 1e-12);
    dst.x_inert = (dst.x_inert * dst.n + src.x_inert * dn) / std::max(dst.n + dn, 1e-12);
    dst.n += dn;
    dst.E += e_per_mol * dn;

    src.x_fuel  *= sf; src.x_o2 *= sf; src.x_inert *= sf;
    (void)src; (void)dst;
}

void setVolume(GasState &s, double newV) {
    // E stays (adiabatic free expansion approximation used by engine-sim too);
    // compression heating emerges from the work term added by the piston.
    s.V = std::max(newV, 1e-9);
}

} // namespace ice
