// core/exhaust1d.cpp — 1D pulse propagation + open-end radiation.
// Works at simulation rate (5–44 kHz): each step advances the wave field by
// dt along the pipe; pressure pulses injected at valve events travel at c
// and radiate at the open end.
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
        const double A = M_PI * cfg.primary_d_m * cfg.primary_d_m / 4.0;
        seg.nodes.assign(primNodes, PipeNode{});
        for (auto &n : seg.nodes) {
            n.area = A;
            n.p = P_ATM;
            n.T = 600;
            n.u = 0;
        }
    }
    m_tail.length_m = cfg.tail_L_m;
    m_tail.diameter_m = cfg.tail_d_m;
    const double At = M_PI * cfg.tail_d_m * cfg.tail_d_m / 4.0;
    m_tail.nodes.assign(tailNodes, PipeNode{});
    for (auto &n : m_tail.nodes) {
        n.area = At;
        n.p = P_ATM;
        n.T = 500;
        n.u = 0;
    }
    m_tailSignal = 0;
    m_collectorSignal = 0;
    m_prevTailPressure = P_ATM;
}

int Exhaust1D::nodeCount() const {
    int n = (int)m_tail.nodes.size();
    for (auto &s : m_primary) n += (int)s.nodes.size();
    return n;
}

void Exhaust1D::step(double dt, const double *portPressure, const double *portVel, int nPorts,
                     const double *portLift) {
    injectPorts(portPressure, portVel, nPorts, portLift);
    for (auto &seg : m_primary) advectSegment(seg, dt);
    advectSegment(m_tail, dt);
    boundaryEnds();
}

void Exhaust1D::injectPorts(const double *portPressure, const double *portVel, int nPorts,
                             const double *portLift) {
    (void)portVel;
    // inject the cylinder port pressure at the head of each primary while the
    // exhaust valve is open; a closed valve decouples the pipe (weak leak only)
    for (int i = 0; i < (int)m_primary.size() && i < nPorts; ++i) {
        if (m_primary[i].nodes.empty()) continue;
        auto &first = m_primary[i].nodes.front();
        const double lift = portLift ? std::clamp(portLift[i], 0.0, 1.0) : 1.0;
        // coupling strength follows valve lift; closed valve -> tiny leak
        const double relax = 0.05 + 0.85 * lift;
        first.p += relax * (portPressure[i] - first.p);
        // blowdown gust velocity from the pressure jump (valve open only)
        if (lift > 0.01) {
            const double dp = portPressure[i] - first.p;
            if (dp > 200.0) {
                first.u = std::clamp(first.u + dp * 2e-5 * lift, -200.0, 200.0);
            }
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
    const double C = dt / std::max(dx, 1e-9);
    if (C > 1.2) return; // CFL guard

    // snapshot primitives
    static thread_local std::vector<double> pN, uN;
    pN.resize(N); uN.resize(N);
    for (int i = 0; i < N; ++i) {
        pN[i] = seg.nodes[i].p;
        uN[i] = std::clamp(seg.nodes[i].u, -250.0, 250.0);
    }

    // first-order characteristic (upwind on Riemann invariants), all nodes.
    // Boundary nodes use one-sided neighbor values (mirror across the end).
    for (int i = 0; i < N; ++i) {
        const int iL = (i > 0) ? i - 1 : i + 1;      // mirror at left end
        const int iR = (i < N - 1) ? i + 1 : i - 1;  // mirror at right end
        const double T = std::max(seg.nodes[i].T, 200.0);
        const double rho = pN[i] / (R_SPECIFIC * T);
        const double c = std::sqrt(GAMMA * R_SPECIFIC * T);

        // right-going wave (v = u + c > 0) brings info from the left node;
        // left-going wave (v = u - c < 0) brings info from the right node.
        const double Jplus  = pN[i] + rho * c * uN[i];
        const double Jminus = pN[i] - rho * c * uN[i];

        // fraction of the step the right-going wave covers
        const double vPlus = uN[i] + c;
        const double aR = std::clamp(vPlus * C, 0.0, 1.0);
        // left-going
        const double vMinus = uN[i] - c;
        const double aL = std::clamp(-vMinus * C, 0.0, 1.0);

        const double rhoL = pN[iL] / (R_SPECIFIC * T);
        const double rhoR = pN[iR] / (R_SPECIFIC * T);
        const double JplusIn  = pN[iL] + rhoL * c * uN[iL];
        const double JminusIn = pN[iR] - rhoR * c * uN[iR];

        const double JplusNew  = Jplus  * (1.0 - aR) + JplusIn  * aR;
        const double JminusNew = Jminus * (1.0 - aL) + JminusIn * aL;

        const double pNew = 0.5 * (JplusNew + JminusNew);
        const double uNew = 0.5 * (JplusNew - JminusNew) / (rho * c);

        seg.nodes[i].p = pNew;
        seg.nodes[i].u = std::clamp(uNew, -250.0, 250.0);
    }
}

void Exhaust1D::boundaryEnds() {
    // junction: primaries -> tail head (average, collector merge)
    double pSum = 0;
    for (auto &seg : m_primary) {
        pSum += seg.nodes.back().p;
    }
    const int nP = std::max((int)m_primary.size(), 1);
    const double pCol = pSum / nP;

    if (!m_tail.nodes.empty()) {
        auto &head = m_tail.nodes.front();
        head.p += 0.7 * (pCol - head.p);
        // merge back-pressure into primaries (weak)
        for (auto &seg : m_primary) {
            seg.nodes.back().p += 0.3 * (head.p - seg.nodes.back().p);
        }
    }

    // open end: partial reflection (unflanged pipe, R ~ 0.3..0.5)
    auto &end = m_tail.nodes.back();
    const double pExcess = end.p - P_ATM;
    const double keep = 0.3;
    end.p = P_ATM + pExcess * keep;

    // audio tap: radiated pressure is the non-reflected part of the wave
    const double radiated = pExcess * (1.0 - keep);
    m_tailSignal = radiated;
    m_prevTailPressure = end.p;
}

} // namespace ice
