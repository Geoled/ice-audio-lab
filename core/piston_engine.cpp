// core/piston_engine.cpp — the 4T piston engine step
#include "piston_engine.h"
#include "combustion.h"
#include <cmath>
#include <algorithm>

namespace ice {

// ---------------------------------------------------------------------------
// helpers
// 4T cycle, theta 0..4pi per cylinder:
//   0      : TDC overlap (end of exhaust / start of intake)
//   0..pi  : intake stroke
//   pi..2pi: compression
//   2pi    : firing TDC (spark)
//   2pi..3pi: power stroke
//   3pi..4pi: exhaust stroke
// Intake lobe centered lobe_sep AFTER the overlap TDC; exhaust lobe centered
// lobe_sep BEFORE the next overlap TDC (i.e. at 4pi - lobe_sep).
static double lobeLift(const CamProfile &cam, double cycleAngle, bool exhaust) {
    const double center = exhaust
        ? (4.0 * M_PI - cam.lobe_sep_deg * M_PI / 180.0)   // before overlap TDC
        : (cam.lobe_sep_deg * M_PI / 180.0);               // after overlap TDC
    // shortest signed angular distance, wrapped to [-2pi, 2pi]
    double d = std::fmod(cycleAngle - center, 4.0 * M_PI);
    if (d > 2.0 * M_PI) d -= 4.0 * M_PI;
    if (d < -2.0 * M_PI) d += 4.0 * M_PI;
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
    m_portPressure.resize(cfg.cylinders.size(), P_ATM);
    m_portLift.assign(cfg.cylinders.size(), 0.0);

    // firing slots: firing_order lists cylinder numbers (1-based) in the order
    // they should fire; map each to an evenly spaced slot in the 4pi cycle.
    // Unlisted cylinders get sequential leftover slots.
    {
        for (int i = 0; i < 64; ++i) {
            m_lastFire[i] = -1.0;   // all plugs armed
            m_firingSlot[i] = 0;
        }
        const int n = (int)cfg.cylinders.size();
        std::vector<int> slotOf(std::min(n, 64), -1);
        int nextSlot = 0;
        for (int ord : cfg.ignition.firing_order) {
            const int idx = ord - 1;
            if (idx >= 0 && idx < n && idx < 64 && slotOf[idx] < 0) {
                slotOf[idx] = nextSlot++;
            }
        }
        for (int i = 0; i < n && i < 64; ++i) {
            if (slotOf[i] < 0) slotOf[i] = nextSlot++;
        }
        for (int i = 0; i < n && i < 64; ++i) m_firingSlot[i] = slotOf[i];
    }

    // plenum gas (air at atm)
    m_plenum.V = cfg.intake.plenum_L * 1e-3;
    m_plenum.n = P_ATM * m_plenum.V / (R_GAS * T_REF);
    m_plenum.E = 0.5 * m_plenum.dof * m_plenum.n * R_GAS * T_REF;
    m_plenum.x_o2 = 0.21; m_plenum.x_inert = 0.79;

    m_atm.V = 1000.0;
    m_atm.n = P_ATM * m_atm.V / (R_GAS * T_REF);
    m_atm.E = 0.5 * m_atm.dof * m_atm.n * R_GAS * T_REF;
    m_atm.x_o2 = 0.21; m_atm.x_inert = 0.79;

    // cylinder initial gas: match theta=0 (TDC) volume so the first step has
    // no synthetic compression jump
    for (size_t i = 0; i < m_cyl.size(); ++i) {
        const double v0 = cfg.cylinders[i].chamber_cc * 1e-6; // TDC volume at theta 0
        m_cyl[i].gas.V = v0;
        m_cyl[i].volume = v0;
        m_cyl[i].gas.n = P_ATM * v0 / (R_GAS * T_REF);
        m_cyl[i].gas.E = 0.5 * m_cyl[i].gas.dof * m_cyl[i].gas.n * R_GAS * T_REF;
        m_cyl[i].gas.x_o2 = 0.21; m_cyl[i].gas.x_inert = 0.79;
    }

    Exhaust1D::Config ec;
    ec.cylinders = (int)cfg.cylinders.size();
    ec.primary_L_m = cfg.primary_L_m;
    ec.primary_d_m = cfg.primary_d_m;
    ec.tail_L_m = cfg.tail_L_m;
    ec.tail_d_m = cfg.tail_d_m;
    ec.nodes_per_m = 12;
    m_exhaust.initialize(ec);

    // virtual collector reservoir
    m_exhaustReservoir.V = 10.0;
    m_exhaustReservoir.n = P_ATM * m_exhaustReservoir.V / (R_GAS * 800.0);
    m_exhaustReservoir.E = 0.5 * m_exhaustReservoir.dof * m_exhaustReservoir.n * R_GAS * 800.0;
    m_exhaustReservoir.x_o2 = 0.05; m_exhaustReservoir.x_inert = 0.95;
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

    // starter motor torque (geared starter: high torque at low speed)
    double externalTorque = 0;
    if (m_starter) {
        const double stallRpm = 300;
        const double base = 450.0; // Nm at the crank (geared)
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
        const double jDeg = m_cfg.crank.journal_deg[std::min(i, (int)m_cfg.crank.journal_deg.size()-1)];
        const double jRad = jDeg * M_PI / 180.0;
        // cylinder-local cycle angle (piston and valves share this phase)
        const double cylCycle = std::fmod(std::fmod(m_theta, 4.0 * M_PI) + jRad, 4.0 * M_PI);

        // piston position & volume
        const double pos = pistonPos(i);
        const double newV = chamberVolume(i);
        const double prevV = st.volume;   // last stored volume
        const double area = M_PI * cy.bore_m * cy.bore_m / 4.0;

        // --- compression/expansion work (trapezoid: symmetric, no energy drift) ---
        const double dV = newV - prevV;
        if (std::abs(dV) > 1e-15) {
            const double P0 = st.gas.pressure();
            const double g = st.gas.heatCapacityRatio();
            const double scale = std::pow(std::max(prevV, 1e-9) / std::max(newV, 1e-9), g - 1.0);
            const double P1 = P0 * scale;
            st.gas.E -= 0.5 * (P0 + P1) * dV;
        }
        setVolume(st.gas, newV);

        st.volume = newV;
        st.piston_x_m = pos;

        // valve lift (phased by the cylinder's own cycle angle)
        st.intake_lift = lobeLift(cy.cam_intake, cylCycle, false);
        st.exhaust_lift = lobeLift(cy.cam_exhaust, cylCycle, true);

        // gas exchange
        doValves(i, dt);

        // blowby: rings leak high cylinder pressure into the crankcase
        {
            const double crankcaseP = P_ATM;
            if (st.gas.pressure() > crankcaseP * 1.05) {
                const double leakK = 3e-7;
                GasState crank;
                crank.V = 5e-3;
                crank.n = P_ATM * crank.V / (R_GAS * 320.0);
                crank.E = 0.5 * crank.dof * crank.n * R_GAS * 320.0;
                crank.x_o2 = 0.2; crank.x_inert = 0.8;
                flowMoles(st.gas, crank, leakK, dt);
            }
        }

        // ignition: each cylinder has its own firing slot in the global 4pi
        // engine cycle: slot k fires at (2pi - advance + k*4pi/nCyl).
        // The spark angle is evaluated against the GLOBAL cycle position
        // (m_theta mod 4pi), because cylinders sharing a crank journal have
        // identical kinematic phase but must fire in different cycles.
        // With one spark per slot per 4pi, an n-cyl 4T engine fires evenly
        // every (4pi/nCyl) of crank rotation.
        const double advanceDeg = ignitionAdvanceDeg(rpm());
        const double slotAngle = (4.0 * M_PI) * m_firingSlot[i] / std::max(nCyl, 1);
        const double fireAngle = std::fmod(
            4.0 * M_PI + (2.0 * M_PI - advanceDeg * M_PI / 180.0) + slotAngle,
            4.0 * M_PI);
        const double globalCycle = std::fmod(m_theta, 4.0 * M_PI);
        bool sparkNow = false;
        if (m_ignition && rpm() < m_cfg.ignition.rev_limit_rpm) {
            const double tol = std::max(dt * std::abs(m_omega), 0.05);
            double diff = std::abs(globalCycle - fireAngle);
            diff = std::min(diff, std::abs(diff - 4.0 * M_PI));
            if (diff < tol && m_lastFire[i] < 0.0) {   // armed: one spark per pass
                sparkNow = true;
                m_lastFire[i] = 1.0;
                ++m_sparkCount[i];
            }
        }
        // re-arm the spark plug once the cycle moves away from the fire angle
        {
            double diff = std::abs(globalCycle - fireAngle);
            diff = std::min(diff, std::abs(diff - 4.0 * M_PI));
            if (diff > 0.5 && m_lastFire[i] > 0.0) m_lastFire[i] = -1.0;
        }
        // gate by the cylinder's own compression stroke: fuel must be present
        // near TDC compression of THIS cylinder (its intake valve has just
        // closed), otherwise the slot spark lands on the wrong stroke
        (void)cylCycle;
        if (sparkNow && !st.lit && st.gas.x_fuel > 1e-6) {
            // the spark slot must coincide with THIS cylinder's compression
            // TDC (cylCycle near 2pi), i.e. its piston near TDC
            const double nearCompTdc = std::abs(cylCycle - 2.0 * M_PI);
            const double nearCompTdc2 = std::abs(nearCompTdc - 4.0 * M_PI);
            const double compDist = std::min(nearCompTdc, nearCompTdc2);
            (void)compDist;
            const double afr = st.gas.x_fuel > 1e-9 ? (st.gas.x_o2 / st.gas.x_fuel) : 0;
            if (afr > 6.0 && afr < 40.0 && compDist < 1.0) {
                st.lit = true;
                st.flame_pos = 0;
                m_rng = m_rng * 1664525u + 1013904223u;
                const double r01 = (m_rng >> 8) / 16777216.0;
                st.burn_eff = std::clamp(0.75 + 0.15 * (r01 - 0.5), 0.3, 0.95);
            }
        }

        // combustion
        if (st.lit) {
            m_rng = m_rng * 1664525u + 1013904223u;
            const double r01 = (m_rng >> 8) / 16777216.0;
            combustStep(st.gas, st.flame_pos, st.burn_eff, dt, rpm(), 450.0 + 273.15, r01);
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
    if (m_omega < 0) m_omega = 0;

    m_theta += m_omega * dt;
    m_cycleWork += gasTorque * m_omega * dt;
    if (m_theta >= 4.0 * M_PI) {
        m_theta -= 4.0 * M_PI;
        // cycle complete: average torque over the cycle
        m_torqueSmooth = m_torqueSmooth * 0.7 + 0.3 * (m_cycleWork / (4.0 * M_PI));
        m_cycleWork = 0;
    }

    // ---- 1D exhaust: port pressures -> pipes ----
    for (int i = 0; i < nCyl; ++i) {
        const auto &st = m_cyl[i];
        const double lift = st.exhaust_lift;
        const double ported = lift > 0.01
            ? st.gas.pressure()
            : P_ATM;
        m_portPressure[i] = ported;
        m_portLift[i] = lift;
    }
    m_exhaust.step(dt, m_portPressure.data(), nullptr, nCyl, m_portLift.data());

    // ---- plenum fill (throttle) ----
    {
        const double throttleArea = M_PI * std::pow(m_cfg.intake.throttle_bore_m * 0.5, 2)
            * std::clamp(m_throttle, 0.0, 1.0);
        // throttle k calibrated like a carb of the same bore area
        const double kThrottle = throttleArea * 3.0;
        const double k = kThrottle + m_cfg.intake.idle_flow_k;
        const double dn = flowMoles(m_atm, m_plenum, k, dt);
        // port injection: fuel vapor proportional to charge (stoich ~60:1 molar)
        if (dn > 0) {
            const double fuelMol = dn / 60.0;
            const double nNew = m_plenum.n; // flowMoles already added air
            m_plenum.x_fuel = (m_plenum.x_fuel * nNew + fuelMol) / std::max(nNew, 1e-9);
            m_plenum.x_fuel = std::min(m_plenum.x_fuel, 0.04);
        }
    }
    // burn off stale fuel in plenum slowly when no flow (prevents infinite build-up)
    m_plenum.x_fuel *= (1.0 - 0.02 * dt);

    // ---- telemetry ----
    m_tel.rpm = rpm();
    m_tel.torque_nm = gasTorque;
    m_tel.torque_avg_nm = m_torqueSmooth;
    m_tel.power_kw = m_torqueSmooth * std::abs(m_omega) / 1000.0;
    m_tel.map_kpa = m_plenum.pressure() / 1000.0;
    m_tel.throttle = m_throttle;
    m_tel.starter = m_starter;
    m_tel.ignition = m_ignition;

    // NaN hunter: flag broken states for the UI
    m_nanDetected = false;
    if (!m_plenum.sane()) { m_plenum.sanitize(); m_nanDetected = true; }
    for (auto &c : m_cyl) {
        if (!c.gas.sane()) { c.gas.sanitize(); m_nanDetected = true; }
    }
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

    // exhaust: cylinder -> virtual collector volume held near atmospheric
    // (models the open boundary; the 1D pipes read port pressure separately)
    if (st.exhaust_lift > 0.01) {
        GasState &exhaustReservoir = m_exhaustReservoir;
        const double k = cy.exhaust_flow_k * st.exhaust_lift;
        flowMoles(st.gas, exhaustReservoir, k, dt);
        // relax reservoir back toward ambient so it never pressurizes
        const double nEq = P_ATM * exhaustReservoir.V / (R_GAS * 800.0);
        const double relax = 0.05;
        exhaustReservoir.n += relax * (nEq - exhaustReservoir.n);
        const double eEq = 0.5 * exhaustReservoir.dof * nEq * R_GAS * 800.0;
        exhaustReservoir.E += relax * (eEq - exhaustReservoir.E);
    }
}

void PistonEngine::computeAudioTap(double *outTaps, int nTaps) const {
    // taps: 0 = header (collector), 1 = tail
    if (nTaps >= 1) outTaps[0] = m_exhaust.collectorSignal();
    if (nTaps >= 2) outTaps[1] = m_exhaust.tailSignal();
}

} // namespace ice
