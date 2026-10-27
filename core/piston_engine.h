// core/piston_engine.h — the running engine model
#pragma once
#include "engine.h"
#include "exhaust1d.h"

namespace ice {

struct EngineTelemetry {
    double rpm = 0;
    double torque_nm = 0;         // filtered instantaneous
    double torque_avg_nm = 0;     // cycle-average
    double power_kw = 0;
    double map_kpa = 0;           // plenum pressure
    double afr = 0;
    double sim_freq = 0;
    int sim_steps_per_frame = 0;
    double throttle = 0;          // 0..1
    bool starter = false;
    bool ignition = false;
    double load_cpu_pct = 0;
    double idleThrottle = 0;  // governor contribution (telemetry)
};

class PistonEngine {
public:
    explicit PistonEngine(const EngineConfig &cfg);

    // one physics step of dt seconds
    void step(double dt);

    // audio tap: exhaust-port flow signal for this step (per cylinder into taps)
    void computeAudioTap(double *outTaps, int nTaps) const;

    // air/induction noise level (0..1) for the synth's intake layer:
    // rpm-weighted, always audible when the engine spins
    double inductionNoise() const;

    const EngineConfig &config() const { return m_cfg; }
    EngineTelemetry &telemetry() { return m_tel; }
    const std::vector<CylinderState> &cylinders() const { return m_cyl; }

    void setThrottle(double t01) { m_throttle = std::clamp(t01, 0.0, 1.0); }
    void setStarter(bool on) { m_starter = on; }
    void setIgnition(bool on) { m_ignition = on; }
    void setIdleControl(bool on) { m_idleControl = on; }  // idle governor
    double idleThrottle() const { return m_idleThrottle; }  // what the governor adds

    double crankAngle() const { return m_theta; }   // rad, 0..4pi cycle
    double rpm() const { return m_omega * 60.0 / (2 * M_PI); }

    Exhaust1D &exhaust() { return m_exhaust; }
    bool nanDetected() const { return m_nanDetected; }

    // spark counters per cylinder (debug/firing-evenness checks)
    const int *sparkCounts() const { return m_sparkCount; }
    int cylinderCount() const { return (int)m_cyl.size(); }

    // scratch buffer for port pressures (avoids allocation in step)
    std::vector<double> m_portPressure;
    std::vector<double> m_portLift;

private:
    double pistonPos(int cyl) const;      // 0 at TDC .. 1 full
    double chamberVolume(int cyl) const;  // m^3
    void doValves(int cyl, double dt);
    void doCombustion(int cyl, double dt);
    double ignitionAdvanceDeg(double rpm) const;

    EngineConfig m_cfg;
    std::vector<CylinderState> m_cyl;
    std::vector<IgnitionWire> m_wires;

    double m_theta = 0;      // crank angle within cycle (0..4pi)
    double m_omega = 0;      // rad/s
    double m_throttle = 0;
    bool m_starter = false;
    bool m_ignition = false;
    bool m_idleControl = true; // idle governor on by default
    double m_idleThrottle = 0; // governor's extra throttle (telemetry)
    double m_idleRpmTarget = 850;

    GasState m_plenum;
    GasState m_atm;
    GasState m_exhaustReservoir;  // virtual collector at ~1 atm

    Exhaust1D m_exhaust;
    EngineTelemetry m_tel;

    // cycle torque accumulation
    double m_cycleWork = 0;
    double m_cycleAngleStart = 0;
    double m_torqueSmooth = 0;
    double m_lastFire[64];        // spark arm state per cylinder (-1 armed, +1 fired)
    int m_firingSlot[64] = {0};    // per-cylinder slot in the 720-deg firing order
    int m_cycleOffset[64] = {0};  // 0 or 1: opposite-stroke pair on shared journal

    uint32_t m_rng = 123456789u;
    bool m_nanDetected = false;
    int m_sparkCount[64] = {0};   // per-cylinder spark counter (firing balance)
};

} // namespace ice
