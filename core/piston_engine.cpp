// core/piston_engine.cpp — the 4T piston engine step
#include "piston_engine.h"
#include "combustion.h"
#include <cmath>
#include <algorithm>

namespace ice {

// ---------------------------------------------------------------------------
// helpers
static double lobeLift(const CamProfile &cam, double cycleAngle, bool exhaust) {
    // cycleAngle: 0..4pi (720°). Lobe centered at lobe_sep for intake (after TDC
    // overlap), exhaust centered before TDC.
    constexpr double PI2 = 2.0 * M_PI;
    const double center = exhaust
        ? (2.0 * M_PI - cam.lobe_sep_deg * M_PI / 180.0)   // before TDC exhaust
        : (cam.lobe_sep_deg * M_PI / 180.0);               // after TDC intake
    double d = std::fmod(cycleAngle - center + 4.0 * M_PI, 2.0 * PI2);
    if (d > PI2) d -= PI2; // wrap to -2pi..2pi
    const double half = (cam.duration_050_deg * 0.5) * M_PI / 180.0;
    const double x = std::abs(d);
    if (x >= half) return 0;
    // harmonic lobe (engine-sim style): cos-shaped with gamma
    const double t = x / half;
    const double g = 0.9;
    return std::pow(std::cos(t * M_PI / 2), 1.0 / std::max(g, 0.05));
}

LiftFn CamProfile::liftFn() const { return {}; } // unused (we use lobeLift directly)

double PistonEngine::pistonPos(int cyl) const {
    const auto &cc = m_cfg.crank;
    const double jDeg = cc.journal_deg[std::min(cyl, (int)cc.journal_deg.size() - 1)];
    const double phase = m_theta + jDeg * M_PI / 180.0;
    const double r = cc.throw_m;
    const double l = m_cfg.cylinders[cyl].rod_length_m;
    // x from TDC along the bore
    const double cosT = std::cos(phase);
    const double sinT = std::sin(phase);
    const double x = (r + l) - (r * cosT + std::sqrt(std::max(l * l - r * r * sinT * sinT, 0.0)));
    return x / (2.0 * r); // 0..1
}

double PistonEngine::chamberVolume(int cyl) const {
    const auto &cy = m_cfg.cylinders[cyl];
    const double area = M_PI * cy.bore_m * cy.bore_m / 4.0;
    const double disp = area * cy.stroke_m;
    const double vMin = cy.chamber_cc * 1e-6;
    const double pos = pistonPos(cyl); // 0 at TDC
    return vMin + pos * disp;
}

PistonEngine::PistonEngine(const EngineConfig &cfg) : m_cfg(cfg) {
    m_cyl.resize(cfg.cylinders.size());
    m_wires.resize(cfg.cylinders.size());

    // plenum gas (air at atm)
    m_plenum.V = cfg.intake.plenum_L * 1e-3;
    m_plenum.n = P_ATM * m_plenum.V / (R_GAS * T_REF);
    m_plenum.E = 0.5 * m_plenum.dof * m_plenum.n * R_GAS * T_REF;
    m_plenum.x_o2 = 0.21; m_plenum.x_inert = 0.79;

    m_atm.V = 1000.0;
    m_atm.n = P_ATM * m_atm.V / (R_GAS * T_REF);
    m_atm.E = 0.5 * m_atm.dof * m_atm.n * R_GAS * T_REF;
    m_atm.x_o2 = 0.21; m_atm.x_inert = 0.79;

    // cylinder initial gas
    for (auto &c : m_cyl) {
        c.gas.V = 1e-3;
        c.gas.n = P_ATM * c.gas.V / (R_GAS * T_REF);
        c.gas.E = 0.5 * c.gas.dof * c.gas.n * R_GAS * T_REF;
        c.gas.x_o2 = 0.21; c.gas.x_inert = 0.79;
    }

    Exhaust1D::Config ec;
    ec.cylinders = (int)cfg.cylinders.size();
    ec.primary_L_m = cfg.primary_L_m;
    ec.primary_d_m = cfg.primary_d_m;
    ec.tail_L_m = cfg.tail_L_m;
    ec.tail_d_m = cfg.tail_d_m;
    ec.nodes_per_m = 12;
    m_exhaust.initialize(ec);
}

double PistonEngine::ignitionAdvanceDeg(double rpm) const {
    const auto &tc = m_cfg.ignition.timing_curve;
    if (tc.empty()) return 20;
    double a = tc.front().second;
    for (size_t i = 1; i < tc.size(); ++i) {
        if (rpm <= tc[i].first) {
            const double t = (rpm - tc[i-1].first) / std::max(1.0, tc[i].first - tc[i-1].first);
            return tc[i-1].second + t * (tc[i].second - tc[i-1].second);
        }
        a = tc[i].second;
    }
    return a;
}

void PistonEngine::step(double dt) {
    const int nCyl = (int)m_cyl.size();

    // starter motor torque
    double externalTorque = 0;
    if (m_starter) {
        const double stallRpm = 250;
        const double base = 120.0; // Nm at 0 rpm
        externalTorque += base * std::clamp(1.0 - rpm() / stallRpm, 0.0, 1.0);
    }
    // engine friction (viscous + constant)
    externalTorque -= m_cfg.crank.friction_nm * std::clamp(std::abs(m_omega) / 50.0, 0.0, 1.0);
    externalTorque -= 0.02 * m_omega; // viscous Nm/(rad/s)

    // ---- per-cylinder gas exchange & combustion, apply torque ----
    double gasTorque = 0;
    for (int i = 0; i < nCyl; ++i) {
        auto &cy = m_cfg.cylinders[i];
        auto &st = m_cyl[i];

        // piston position & volume
        const double pos = pistonPos(i);
        const double prevV = st.gas.V;
        const double newV = chamberVolume(i);
        const double dV = newV - prevV;
        const double area = M_PI * cy.bore_m * cy.bore_m / 4.0;

        // compression work: dE = -P dV (work done BY gas is +P dV -> E decreases)
        const double P_before = st.gas.pressure();
        st.gas.E -= P_before * dV * 0.5; // half-explicit to keep stable
        setVolume(st.gas, newV);

        st.volume = newV;
        st.piston_x_m = pos;

        // valve lift
        st.intake_lift = lobeLift(cy.cam_intake, m_theta, false);
        st.exhaust_lift = lobeLift(cy.cam_exhaust, m_theta, true);

        // gas exchange
        doValves(i, dt);

        // ignition: fire when the plug angle arrives
        const double advanceDeg = ignitionAdvanceDeg(rpm());
        const double fireAngle = std::fmod(4.0 * M_PI + (2.0 * M_PI - advanceDeg * M_PI / 180.0), 4.0 * M_PI);
        const double cyclePos = std::fmod(m_theta, 4.0 * M_PI);
        const double jDeg = m_cfg.crank.journal_deg[std::min(i, (int)m_cfg.crank.journal_deg.size()-1)];
        const double cylCycle = std::fmod(cyclePos + jDeg * M_PI / 180.0, 4.0 * M_PI);
        bool sparkNow = false;
        if (m_ignition && rpm() < m_cfg.ignition.rev_limit_rpm) {
            static thread_local double lastFire[64] = {};
            const double tol = std::max(dt * std::abs(m_omega), 0.02);
            const double diff = std::abs(cylCycle - fireAngle);
            const double diff2 = std::abs(diff - 4.0 * M_PI);
            if ((diff < tol || diff2 < tol) && m_theta - lastFire[i] > 3.0 * M_PI) {
                sparkNow = true;
                lastFire[i] = m_theta;
            }
        }
        if (sparkNow && !st.lit && st.gas.x_fuel > 1e-6) {
            // equivalence ratio gate
            const double afr = st.gas.x_fuel > 1e-9 ? (st.gas.x_o2 / st.gas.x_fuel) : 0;
            if (afr > 6.0 && afr < 40.0) {
                st.lit = true;
                st.flame_pos = 0;
                // burn efficiency with randomness
                m_rng = m_rng * 1664525u + 1013904223u;
                const double r01 = (m_rng >> 8) / 16777216.0;
                st.burn_eff = std::clamp(0.75 + 0.15 * (r01 - 0.5), 0.3, 0.95);
            }
        }

        // combustion
        if (st.lit) {
            m_rng = m_rng * 1664525u + 1013904223u;
            const double r01 = (m_rng >> 8) / 16777216.0;
            const double dE = combustStep(st.gas, st.flame_pos, st.burn_eff, dt, rpm(), 450.0, r01);
            (void)dE;
            if (st.flame_pos >= 1.0) st.lit = false;
        }

        // pressure ring telemetry (0..719 over 720°)
        {
            const double a = cylCycle / (4.0 * M_PI) * 720.0;
            const int idx = std::clamp((int)a, 0, 719);
            st.pressure_ring[idx] = st.gas.pressure();
            st.volume_ring[idx] = st.gas.V;
        }

        // torque on crank from gas pressure (approximate: P*A*throw*sin)
        const double P = st.gas.pressure();
        const double jRad = jDeg * M_PI / 180.0;
        const double sinTerm = std::sin(m_theta + jRad);
        const double torque = area * (P - P_ATM) * m_cfg.crank.throw_m * sinTerm;
        gasTorque += torque;
        st.cycle_torque_j += torque * std::abs(m_omega) * dt;

        // exhaust/intake blowdown resets lit on overlap
        if (st.exhaust_lift > 0.5 && pos > 0.9) st.lit = false;
    }

    // ---- integrate crankshaft ----
    const double totalTorque = gasTorque + externalTorque;
    const double inertia = m_cfg.crank.inertia_kgm2
        + 0.005 * nCyl; // rod/piston reflected inertia (rough)
    m_omega += totalTorque / std::max(inertia, 0.01) * dt;

    // rev limiter hard cut (spark already gated above)
    if (rpm() > m_cfg.ignition.rev_limit_rpm + 300) {
        m_omega = (m_cfg.ignition.rev_limit_rpm + 300) * 2.0 * M_PI / 60.0;
    }
    if (rpm() < 0) m_omega = 0;

    m_theta += m_omega * dt;
    if (m_theta >= 4.0 * M_PI) {
        m_theta -= 4.0 * M_PI;
        // cycle complete: update average torque from work
        m_torqueSmooth = m_torqueSmooth * 0.7 + 0.3 * (m_cycleWork / (4.0 * M_PI));
        m_cycleWork = 0;
    }
    m_cycleWork += gasTorque * m_omega * dt;

    // ---- 1D exhaust: port pressures -> pipes ----
    static thread_local std::vector<double> pPorts, vPorts;
    pPorts.resize(nCyl); vPorts.resize(nCyl);
    for (int i = 0; i < nCyl; ++i) {
        const auto &st = m_cyl[i];
        const double ported = st.exhaust_lift > 0.01
            ? st.gas.pressure()
            : P_ATM + 0.1 * (st.gas.pressure() - P_ATM) * st.exhaust_lift;
        pPorts[i] = ported;
        vPorts[i] = 0;
    }
    m_exhaust.step(dt, pPorts.data(), vPorts.data(), nCyl);

    // ---- plenum fill (throttle) ----
    const double throttleArea = M_PI * std::pow(m_cfg.intake.throttle_bore_m * 0.5, 2)
        * std::clamp(m_throttle, 0.0, 1.0);
    // flow from atmosphere into plenum
    {
        GasState virtualSource = m_atm;
        // inject small fuel vapor into plenum for AFR modeling (port injection)
        const double k = throttleArea * 4e-4 + m_cfg.intake.idle_flow_k * 1e3;
        const double dn = flowMoles(m_atm, m_plenum, k, dt);
        // enrich plenum with fuel vapor to reach ~14.7 AFR molar-ish
        if (dn > 0) {
            const double fuelMol = dn / 60.0; // stoich mol ratio ~ 60:1 for C8H16 (12.5 O2 per fuel, air 21% O2)
            m_plenum.x_fuel = std::min(0.05, m_plenum.x_fuel + fuelMol / std::max(m_plenum.n, 1e-9));
        }
        (void)virtualSource;
    }

    // ---- telemetry ----
    m_tel.rpm = rpm();
    m_tel.torque_nm = gasTorque;
    m_tel.torque_avg_nm = m_torqueSmooth;
    m_tel.power_kw = m_torqueSmooth * std::abs(m_omega) / 1000.0;
    m_tel.map_kpa = m_plenum.pressure() / 1000.0;
    m_tel.afr = 0;
    m_tel.throttle = m_throttle;
    m_tel.starter = m_starter;
    m_tel.ignition = m_ignition;
}

void PistonEngine::doValves(int cyl, double dt) {
    auto &cy = m_cfg.cylinders[cyl];
    auto &st = m_cyl[cyl];

    // intake: plenum -> cylinder, limited by lift and runner
    if (st.intake_lift > 0.01) {
        const double k = cy.intake_flow_k * st.intake_lift;
        double dn = flowMoles(m_plenum, st.gas, k, dt);
        // fuel comes with the charge
        (void)dn;
    }

    // exhaust: cylinder -> a virtual collector volume at 1 atm (boundary),
    // we don't model return flow into cylinder from pipes in S0 except through
    // exhaust1d injection; use a big reservoir.
    if (st.exhaust_lift > 0.01) {
        static thread_local GasState exhaustReservoir = []() {
            GasState g;
            g.V = 10.0;
            g.n = P_ATM * g.V / (R_GAS * 800.0);
            g.E = 0.5 * g.dof * g.n * R_GAS * 800.0;
            g.x_o2 = 0.05; g.x_inert = 0.95;
            return g;
        }();
        const double k = cy.exhaust_flow_k * st.exhaust_lift;
        flowMoles(st.gas, exhaustReservoir, k, dt);
        // keep reservoir near atmospheric (open end)
        const double nEq = P_ATM * exhaustReservoir.V / (R_GAS * 800.0);
        exhaustReservoir.n = nEq;
        exhaustReservoir.E = 0.5 * exhaustReservoir.dof * nEq * R_GAS * 800.0;
    }
}

void PistonEngine::computeAudioTap(double *outTaps, int nTaps) const {
    // taps: 0 = header (collector), 1 = tail
    if (nTaps >= 1) outTaps[0] = m_exhaust.collectorSignal();
    if (nTaps >= 2) outTaps[1] = m_exhaust.tailSignal();
}

} // namespace ice
