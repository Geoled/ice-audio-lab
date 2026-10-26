// core/config.cpp
#include "config.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <cmath>
#include <map>

namespace ice {

using nlohmann::json;

// SCFM -> our k units (mol/s per sqrt(J/mol) style). Engine-sim calibrates
// k through a flow formula; we map linearly and tune by ear against engine-sim.
double kCarb(double scfm) {
    // empirical scale: 100 SCFM carb -> usable k for our formula
    return scfm * 5e-11;
}
double k28inH2O(double scfm) {
    return scfm * 5e-11 * 0.8;
}

static double parseFlowK(const json &j) {
    if (j.is_number()) return j.get<double>();
    if (j.is_string()) {
        const std::string s = j.get<std::string>();
        // patterns: k_carb(123), k_28inH2O(123)
        auto grab = [&](const char *tag) -> double {
            const std::string t = tag;
            const auto pos = s.find(t);
            if (pos == std::string::npos) return -1;
            const auto a = s.find('(', pos);
            const auto b = s.find(')', pos);
            if (a == std::string::npos || b == std::string::npos || b <= a) return -1;
            try { return std::stod(s.substr(a + 1, b - a - 1)); } catch (...) { return -1; }
        };
        double v;
        if ((v = grab("k_carb")) >= 0) return kCarb(v);
        if ((v = grab("k_28inH2O")) >= 0) return k28inH2O(v);
        try { return std::stod(s); } catch (...) { return 1e-9; }
    }
    return 1e-9;
}

bool loadEngineJson(const std::string &path, EngineConfig &out, std::string &err) {
    std::ifstream f(path);
    if (!f) { err = "cannot open file: " + path; return false; }
    json j;
    try { f >> j; } catch (const std::exception &e) { err = e.what(); return false; }

    try {
        out = EngineConfig{};
        if (j.contains("meta")) {
            out.name = j["meta"].value("name", "engine");
            out.engine_type = j["meta"].value("engine_type", "piston4t");
        }

        // crank
        if (j.contains("crankshafts") && !j["crankshafts"].empty()) {
            const auto &c = j["crankshafts"][0];
            out.crank.throw_m = c.value("throw_m", 0.043);
            out.crank.inertia_kgm2 = c.value("inertia_kgm2", 0.12);
            out.crank.friction_nm = c.value("friction_nm", 9.0);
            for (const auto &jn : c["journals"]) {
                out.crank.journal_deg.push_back(jn.value("angle_deg", 0.0));
            }
        }

        // cam profiles
        std::map<std::string, CamProfile> cams;
        if (j.contains("camshafts") && j["camshafts"].contains("profiles")) {
            for (auto it = j["camshafts"]["profiles"].begin(); it != j["camshafts"]["profiles"].end(); ++it) {
                CamProfile p;
                const auto &obj = it.value();
                p.duration_050_deg = obj.value("duration_050_deg", 226);
                p.lift_m = obj.value("lift_m", 0.0098);
                p.lobe_sep_deg = obj.value("lobe_sep_deg", 110);
                cams[it.key()] = p;
            }
        }

        // banks -> cylinders (flatten in bank order)
        if (j.contains("banks")) {
            for (const auto &b : j["banks"]) {
                const double bore = b.value("bore_m", 0.086);
                const double stroke = b.value("stroke_m", 0.086);
                const double deck = b.value("deck_height_m", 0.196);
                if (b.contains("cylinders")) {
                    for (const auto &cj : b["cylinders"]) {
                        CylinderConfig cc;
                        cc.bore_m = bore;
                        cc.stroke_m = stroke;
                        cc.journal = cj.value("journal", 0);
                        cc.rod_length_m = cj.value("rod/length_m", deck - stroke / 2 - 0.03);
                        if (cj.contains("rod")) cc.rod_length_m = cj["rod"].value("length_m", cc.rod_length_m);
                        cc.piston_mass_kg = cj.value("piston/mass_kg", 0.42);
                        if (cj.contains("piston")) cc.piston_mass_kg = cj["piston"].value("mass_kg", cc.piston_mass_kg);
                        cc.chamber_cc = 45;
                        if (cj.contains("head")) {
                            const auto &h = cj["head"];
                            cc.chamber_cc = h.value("chamber_cc", 45);
                            if (h.contains("cam")) {
                                const auto &cam = h["cam"];
                                std::string ip = cam.value("profile", "street");
                                std::string ep = cam.value("profile", "street");
                                if (cams.count(ip)) cc.cam_intake = cams[ip];
                                if (cams.count(ep)) cc.cam_exhaust = cams[ep];
                                if (cam.contains("intake")) {
                                    const std::string p2 = cam["intake"].value("profile", ip);
                                    if (cams.count(p2)) cc.cam_intake = cams[p2];
                                }
                                if (cam.contains("exhaust")) {
                                    const std::string p2 = cam["exhaust"].value("profile", ep);
                                    if (cams.count(p2)) cc.cam_exhaust = cams[p2];
                                }
                            }
                            if (h.contains("intake_port")) {
                                cc.intake_flow_k = parseFlowK(h["intake_port"]["flow_k"]);
                                cc.runner_len_m = h["intake_port"].value("runner_len_m", 0.18);
                            }
                            if (h.contains("exhaust_port")) {
                                cc.exhaust_flow_k = parseFlowK(h["exhaust_port"]["flow_k"]);
                            }
                        }
                        out.cylinders.push_back(cc);
                    }
                }
            }
        }

        // ignition
        if (j.contains("ignition")) {
            const auto &ig = j["ignition"];
            for (const auto &fo : ig.value("firing_order", std::vector<int>{})) {
                out.ignition.firing_order.push_back(fo);
            }
            if (ig.contains("timing_curve")) {
                for (const auto &pt : ig["timing_curve"]) {
                    out.ignition.timing_curve.push_back({pt[0], pt[1]});
                }
            }
            out.ignition.rev_limit_rpm = ig.value("rev_limit_rpm", 6800);
        }

        // intake
        if (j.contains("induction") && j["induction"].contains("plenum")) {
            const auto &p = j["induction"]["plenum"];
            out.intake.plenum_L = p.value("volume_L", 3.5);
            if (p.contains("throttle")) {
                out.intake.throttle_bore_m = p["throttle"].value("bore_m", 0.065);
            }
            if (p.contains("idle_circuit")) {
                out.intake.idle_flow_k = parseFlowK(p["idle_circuit"]["flow_k"]);
            }
        }

        // exhaust (S0: collector node drives pipes)
        if (j.contains("exhaust") && j.contains("exhaust/nodes")) {
            // noop - handled below via j["exhaust"]["nodes"]
        }
        if (j.contains("exhaust")) {
            const auto &ex = j["exhaust"];
            if (ex.contains("nodes")) {
                for (const auto &nd : ex["nodes"]) {
                    const std::string t = nd.value("type", "");
                    if (t == "collector") {
                        out.primary_L_m = nd.value("primary_L_m", 0.65);
                        out.primary_d_m = nd.value("primary_d_m", 0.040);
                    } else if (t == "tailpipe") {
                        out.tail_L_m = nd.value("length_m", 0.45);
                        out.tail_d_m = nd.value("diameter_m", 0.055);
                    }
                }
            }
        }

        // audio
        if (j.contains("audio")) {
            const auto &a = j["audio"];
            if (a.value("sim_freq_hz", "auto") == std::string("auto") ||
                a.value("sim_freq_hz", "auto") == "auto") {
                // auto: 22*firing + 18000 clamp; firing unknown until first run
                out.audio.sim_freq_hz = 0; // marker: compute at runtime
            } else {
                out.audio.sim_freq_hz = a.value("sim_freq_hz", 18000.0);
            }
            out.audio.sample_rate_hz = a.value("sample_rate_hz", 48000.0);
            out.audio.jitter = a.value("jitter", 0.4);
            out.audio.air_noise = a.value("air_noise", 0.65);
            out.audio.hf_gain = a.value("hf_gain", 0.012);
            out.audio.target_latency_s = a.value("target_latency_s", 0.10);
        }

        // sanity
        if (out.cylinders.empty()) { err = "no cylinders defined"; return false; }
        if (out.crank.journal_deg.empty()) {
            out.crank.journal_deg.assign(out.cylinders.size(), 0.0);
        }
        return true;
    } catch (const std::exception &e) {
        err = e.what();
        return false;
    }
}

} // namespace ice
