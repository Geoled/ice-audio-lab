// core/exhaust1d.cpp — Lax-Wendroff 1D + open-end radiation
#include "exhaust1d.h"
#include <cmath>
#include <algorithm>

namespace ice {

constexpr double R_SPECIFIC = 287.05;   // J/(kg K) air
constexpr double GAMMA = 1.34;          // hot exhaust gas

void Exhaust1D::initialize(const Config &cfg) {
    m_cfg = cfg;
    const int primNodes = std::max(4, (int)(cfg.primary_L_m * cfg.nodes_per_m) + 1);
    const int tailNodes = std::max(6, (int)(cfg.tail_L_m * cfg.nodes_per_m) + 1);

    m_primary.resize(cfg.cylinders);
    for (auto &seg : m_primary) {
        seg.length_m = cfg.primary_L_m;
        seg.diameter_m = cfg.primary_d_m;
        seg.nodes.assign(primNodes, PipeNode{});
        for (auto &n : seg.nodes) {
            n.area = M_PI * cfg.primary_d_m * cfg.primary_d_m / 4.0;
            n.p = P_ATM;
            n.T = 600;
            n.u = 0;
        }
    }
    m_tail.length_m = cfg.tail_L_m;
    m_tail.diameter_m = cfg.tail_d_m;
    m_tail.nodes.assign(tailNodes, PipeNode{});
    for (auto &n : m_tail.nodes) {
        n.area = M_PI * cfg.tail_d_m * cfg.tail_d_m / 4.0;
        n.p = P_ATM;
        n.T = 500;
        n.u = 0;
    }
    m_tailSignal = 0;
    m_collectorSignal = 0;
    m_prevTailPressure = P_ATM;
}

int Exhaust1D::nodeCount() const {
    int n = m_tail.nodes.size();
    for (auto &s : m_primary) n += s.nodes.size();
    return n;
}

void Exhaust1D::step(double dt, const double *portPressure, const double *portVel, int nPorts) {
    injectPorts(portPressure, portVel, nPorts);

    for (auto &seg : m_primary) advectSegment(seg, dt);
    advectSegment(m_tail, dt);

    boundaryEnds();
}

void Exhaust1D::injectPorts(const double *portPressure, const double *portVel, int nPorts) {
    (void)portVel;
    for (int i = 0; i < (int)m_primary.size() && i < nPorts; ++i) {
        if (m_primary[i].nodes.empty()) continue;
        auto &first = m_primary[i].nodes.front();
        // relaxation toward cylinder port pressure (valve acts as source)
        const double relax = 0.35;
        first.p += relax * (portPressure[i] - first.p);
        if (portPressure[i] > first.p + 1.0) {
            first.u += 0.02 * (portPressure[i] - first.p) /
                       std::max(0.5, 40.0);  // gentle push
        }
    }
    // collector tap: average of primary exits
    double col = 0;
    for (auto &seg : m_primary) {
        col += seg.nodes.back().p;
    }
    col /= std::max((size_t)1, m_primary.size());
    m_collectorSignal = col - P_ATM;
}

void Exhaust1D::advectSegment(PipeSegment &seg, double dt) {
    const int N = (int)seg.nodes.size();
    if (N < 3) return;
    const double dx = seg.length_m / (N - 1);

    // gather primitives with limited velocity
    auto clip = [](double v) { return std::clamp(v, -250.0, 250.0); };

    // Lax-Wendroff for (p,u)
    static thread_local std::vector<double> pN, uN;
    pN.resize(N); uN.resize(N);
    for (int i = 0; i < N; ++i) {
        pN[i] = seg.nodes[i].p;
        uN[i] = seg.nodes[i].u;
    }

    const double C = dt / (dx + 1e-9);

    for (int i = 1; i < N - 1; ++i) {
        const double rho = pN[i] / (R_SPECIFIC * std::max(seg.nodes[i].T, 200.0));
        const double c = std::sqrt(GAMMA * pN[i] / std::max(rho, 1e-4));
        const double u = clip(uN[i]);

        // characteristic speeds (u+-c)
        const double up = std::abs(u) + c;

        // simple upwind flux-limited update (Rusanov)
        const double dp = pN[i + 1] - pN[i - 1];
        const double du = clip(uN[i + 1] - uN[i - 1]);

        const double fluxP = u * dp * 0.5 + rho * c * c * du * 0.5;
        const double fluxU = (u * du) * 0.5 + dp / std::max(rho, 1e-4) * 0.5;

        double pNew = pN[i] - C * up * 0.5 * dp * 0.5 - C * fluxP * 0.0;
        // stable simplified wave equation update (acoustic):
        pNew = pN[i] - 0.5 * C * (u * dp * 0.5 + rho * c * c * du * 0.5) / std::max(rho, 1e-4);
        double uNew = uN[i] - 0.5 * C * (u * du * 0.5 + dp / std::max(rho, 1e-4) * 0.5);

        // Rusanov diffusion
        const double diff = C * up;
        pNew += 0.25 * diff * (pN[i + 1] - 2 * pN[i] + pN[i - 1]);
        uNew += 0.25 * diff * (clip(uN[i + 1]) - 2 * clip(uN[i]) + clip(uN[i - 1]));

        seg.nodes[i].p = pNew;
        seg.nodes[i].u = clip(uNew);
        // temperature relaxation toward wall
        const double Tw = seg.wall_temp_c + 273.15;
        seg.nodes[i].T += 0.002 * (Tw - seg.nodes[i].T);
    }
}

void Exhaust1D::boundaryEnds() {
    // junction: primaries -> tail head (average)
    double pSum = 0, uSum = 0;
    for (auto &seg : m_primary) {
        pSum += seg.nodes.back().p;
        uSum += seg.nodes.back().u;
    }
    const int nP = std::max((int)m_primary.size(), 1);
    const double pCol = pSum / nP;
    const double uCol = uSum / nP;

    if (!m_tail.nodes.empty()) {
        auto &head = m_tail.nodes.front();
        head.p += 0.5 * (pCol - head.p);
        head.u += 0.3 * (uCol - head.u);
        // reflect collector pressure back into primaries (weak)
        for (auto &seg : m_primary) {
            seg.nodes.back().p += 0.2 * (head.p - seg.nodes.back().p);
        }
    }

    // open end radiation: low-pass reflection coefficient
    auto &end = m_tail.nodes.back();
    const double reflected = 0.75; // unflanged
    const double outgoing = end.p - m_prevTailPressure;
    end.p = P_ATM + reflected * outgoing * 0.5 + (end.p - P_ATM) * (1.0 - 0.05);
    if (end.p < 0.2 * P_ATM) end.p = 0.2 * P_ATM;

    // audio tap: time derivative of radiated pressure (loudness ∝ dp/dt)
    m_tailSignal = (end.p - m_prevTailPressure);
    m_prevTailPressure = end.p;
}

} // namespace ice
