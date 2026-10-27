// core/exhaust1d.h — 1D gas dynamics on the exhaust network (S0: primary->collector->tail)
#pragma once
#include "gas_system.h"
#include <vector>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ice {

// Lax-Wendroff 1D solver on a staggered layout of pipes.
// State per node: pressure p, velocity u, area A, temp T derived from p/rho.
struct PipeNode {
    double p = P_ATM;
    double u = 0;
    double T = 600;
    double area = 1e-3;
};

struct PipeSegment {
    std::vector<PipeNode> nodes;
    double length_m = 1.0;
    double diameter_m = 0.04;
    double wall_temp_c = 500;
};

// S0 exhaust: each cylinder feeds a short primary into one collector pipe
// that ends in an unflanged radiating open end.
class Exhaust1D {
public:
    struct Config {
        int cylinders = 4;
        double primary_L_m = 0.65, primary_d_m = 0.040;
        double tail_L_m = 0.80, tail_d_m = 0.050;
        int nodes_per_m = 12;
    };

    void initialize(const Config &cfg);
    void step(double dt, const double *portPressure, const double *portVel, int nPorts);

    // radiation pressure signal at the tail end (audio tap)
    double tailSignal() const { return m_tailSignal; }
    // collector tap
    double collectorSignal() const { return m_collectorSignal; }

    // debug access for tests
    const PipeSegment &primarySeg(int i) const { return m_primary[i]; }
    const PipeSegment &tailSeg() const { return m_tail; }

    const Config &config() const { return m_cfg; }
    int nodeCount() const;

private:
    void advectSegment(PipeSegment &seg, double dt);
    void injectPorts(const double *portPressure, const double *portVel, int nPorts);
    void boundaryEnds();

    Config m_cfg;
    std::vector<PipeSegment> m_primary; // one per cylinder
    PipeSegment m_tail;
    double m_tailSignal = 0;
    double m_collectorSignal = 0;
    double m_prevTailPressure = P_ATM;
};

} // namespace ice
