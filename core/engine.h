// core/engine.h — shared types: cylinder, ignition, telemetry
#pragma once
#include "gas_system.h"
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ice {

// valve lift profile, 0..1 over cycle angle (0..4pi for 4T)
using LiftFn = std::function<double(double)>;

struct CamProfile {
    double duration_050_deg = 226;
    double lift_m = 0.0098;
    double lobe_sep_deg = 110;
    // harmonic lobe like engine-sim
    LiftFn liftFn() const;
};

struct CylinderConfig {
    double bore_m = 0.086;
    double stroke_m = 0.086;
    double rod_length_m = 0.145;
    double chamber_cc = 45;
    double piston_mass_kg = 0.42;
    double rod_mass_kg = 0.55;
    double comp_height_m = 0.030;
    int journal = 0;            // index into crank journals
    int ignition_channel = 0;   // wire index
    CamProfile cam_intake;
    CamProfile cam_exhaust;
    double intake_flow_k = 5e-9;   // m^3/s/Pa-ish (calibrated below)
    double exhaust_flow_k = 5e-9;
    double runner_len_m = 0.18;
};

struct CrankConfig {
    double throw_m = 0.043;
    double inertia_kgm2 = 0.12;   // crank + flywheel
    double friction_nm = 9.0;
    std::vector<double> journal_deg;
};

struct IntakeConfig {
    double plenum_L = 3.5;
    double throttle_bore_m = 0.065;
    double idle_flow_k = 2e-10;
};

struct IgnitionConfig {
    std::vector<int> firing_order;
    std::vector<std::pair<double,double>> timing_curve; // rpm -> deg BTDC
    double rev_limit_rpm = 6800;
};

struct AudioConfig {
    double sim_freq_hz = 18000;   // 0 => auto
    double sample_rate_hz = 48000;
    double jitter = 0.4;
    double air_noise = 0.65;
    double hf_gain = 0.012;
    double target_latency_s = 0.10;
};

struct EngineConfig {
    std::string name = "I4";
    std::string engine_type = "piston4t";
    CrankConfig crank;
    std::vector<CylinderConfig> cylinders;
    IntakeConfig intake;
    IgnitionConfig ignition;
    AudioConfig audio;
    // exhaust: single collector pipe description (S0)
    double primary_L_m = 0.65;
    double primary_d_m = 0.040;
    double tail_L_m = 0.80;   // collector+middle+tail lumped
    double tail_d_m = 0.050;
};

// one cylinder's live state
struct CylinderState {
    GasState gas;
    double piston_x_m = 0;    // 0 at TDC
    double volume = 0;
    double intake_lift = 0, exhaust_lift = 0;
    bool lit = false;
    double flame_pos = 0;    // 0..1 fraction burned
    double burn_eff = 0.7;
    double peak_pressure = 0;
    // telemetry ring (per cycle angle, 720 samples)
    std::array<double, 720> pressure_ring{};
    std::array<double, 720> volume_ring{};
    // last-cycle integrated values
    double cycle_torque_j = 0;
};

struct IgnitionWire {
    bool energized = false;
};

} // namespace ice
