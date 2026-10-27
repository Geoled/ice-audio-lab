// app/ui.cpp — dashboard, plots, controls
#include "app.h"
#include "imgui.h"
#include "implot.h"
#include <cmath>
#include <algorithm>

namespace ice {

void App::drawUI() {
    drawDashboard();
    drawPlots();
    drawControls();
    drawDebug();
}

void App::drawDashboard() {
    auto &tel = m_engine->telemetry();
    ImGui::SetNextWindowPos(ImVec2(8, 8), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(420, 0), ImGuiCond_Once);
    ImGui::Begin("Dashboard");
    ImGui::Text("Engine: %s", m_cfg.name.c_str());
    ImGui::Separator();

    ImGui::Text("RPM        %8.0f", tel.rpm);
    ImGui::Text("Torque     %8.1f Nm (avg %.1f)", tel.torque_nm, tel.torque_avg_nm);
    ImGui::Text("Power      %8.1f kW", tel.power_kw);
    ImGui::Text("MAP       %8.1f kPa", tel.map_kpa);
    ImGui::Text("Throttle   %8.0f %% (idle ctrl %+.0f%%)", tel.throttle * 100, tel.idleThrottle * 100);
    ImGui::Text("Starter    %s   Ignition %s",
                tel.starter ? "ON" : "off",
                tel.ignition ? "ON" : "off");
    ImGui::Separator();
    ImGui::Text("sim: %.0f Hz, %d steps/frame, 1D nodes: %d",
                 m_simFreq, m_stepsThisFrame, m_engine->exhaust().nodeCount());
    ImGui::Text("audio backlog: %zu frames  (%.0f ms)",
                 m_synth.pendingInput(),
                 1000.0 * (double)m_synth.pendingInput() / std::max(m_simFreq, 1.0));
    if (m_engine->nanDetected()) {
        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "WARNING: NaN detected (auto-sanitized)");
    }
    if (!m_reloadError.empty()) {
        ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "reload error: %s", m_reloadError.c_str());
    }
    ImGui::End();
}

void App::drawPlots() {
    auto &tel = m_engine->telemetry();

    ImGui::SetNextWindowPos(ImVec2(440, 8), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(1140, 560), ImGuiCond_Once);
    ImGui::Begin("Plots");

    if (ImPlot::BeginPlot("RPM / Torque / Power", ImVec2(-1, 260))) {
        ImPlot::SetupAxes("t [s]", "RPM / Nm");
        static double tHist[256];
        static double rpmHist[256], tqHist[256], pwHist[256];
        static int histPos = 0;
        static double lastPush = 0;
        const double now = ImGui::GetTime();
        if (now - lastPush > 0.05) {
            lastPush = now;
            tHist[histPos] = now;
            rpmHist[histPos] = tel.rpm;
            tqHist[histPos] = tel.torque_avg_nm;
            pwHist[histPos] = tel.power_kw;
            histPos = (histPos + 1) % 256;
        }
        ImPlot::PlotLine("RPM", tHist, rpmHist, 256);
        ImPlot::PlotLine("Torque Nm", tHist, tqHist, 256);
        ImPlot::PlotLine("Power kW", tHist, pwHist, 256);
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Cylinder pressure (720 deg)", ImVec2(-1, 240))) {
        ImPlot::SetupAxes("deg", "Pa");
        static double angles[720];
        static bool init = false;
        if (!init) {
            for (int i = 0; i < 720; ++i) angles[i] = i;
            init = true;
        }
        const auto &cyl = m_engine->cylinders();
        double pres[720];
        double vol[720];
        for (int i = 0; i < 720; ++i) {
            pres[i] = cyl.empty() ? 0 : cyl[0].pressure_ring[i] / 1000.0; // kPa
            vol[i] = cyl.empty() ? 0 : cyl[0].volume_ring[i] * 1e6;        // cc
        }
        ImPlot::PlotLine("P [kPa]", angles, pres, 720);
        ImPlot::PlotLine("V [cc]", angles, vol, 720);
        ImPlot::EndPlot();
    }

    ImGui::End();

    // ---------- wave + spectrum + perf ----------
    ImGui::SetNextWindowPos(ImVec2(440, 580), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(1140, 380), ImGuiCond_Once);
    ImGui::Begin("Audio & Performance");

    if (ImPlot::BeginPlot("Waveform", ImVec2(-1, 130))) {
        ImPlot::SetupAxes("sample", "amp", ImPlotAxisFlags_NoLabel);
        static double idx[Synth::WAVE_CAP];
        static double wavg[Synth::WAVE_CAP];
        static bool init = false;
        if (!init) {
            for (int i = 0; i < Synth::WAVE_CAP; ++i) idx[i] = i;
            init = true;
        }
        for (int i = 0; i < Synth::WAVE_CAP; ++i) wavg[i] = m_synth.waveBuf[i];
        ImPlot::PlotLine("out", idx, wavg, Synth::WAVE_CAP);
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Spectrum (dB)", ImVec2(-1, 130), ImPlotFlags_Crosshairs)) {
        ImPlot::SetupAxes("bin", "dB");
        ImPlot::SetupAxisLimits(ImAxis_Y1, -60, 10, ImPlotCond_Once);
        static double bins[256];
        static double db[256];
        static bool sinit = false;
        if (!sinit) { for (int i = 0; i < 256; ++i) bins[i] = i; sinit = true; }
        for (int i = 0; i < 256; ++i) db[i] = m_spectrum[i];
        ImPlot::PlotBars("spec", bins, db, 256, 0.9);
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Frame time / FPS", ImVec2(-1, 130))) {
        static double idx[PerfGraph::N];
        static double msHist[PerfGraph::N], loadHist[PerfGraph::N];
        static bool init = false;
        if (!init) { for (int i = 0; i < PerfGraph::N; ++i) idx[i] = i; init = true; }
        for (int i = 0; i < PerfGraph::N; ++i) {
            msHist[i] = m_frameGraph.values[i];
            loadHist[i] = m_simLoadGraph.values[i];
        }
        ImPlot::PlotLine("frame ms", idx, msHist, PerfGraph::N);
        ImPlot::PlotLine("sim load %", idx, loadHist, PerfGraph::N);
        ImPlot::EndPlot();
    }

    ImGui::Text("FPS %.1f  |  frame %.2f ms  |  sim %.2f ms/frame", m_fpsGraph.latest, m_frameGraph.latest, m_simEwmaMs);
    ImGui::End();
}

void App::drawControls() {
    auto &tel = m_engine->telemetry();
    ImGui::SetNextWindowPos(ImVec2(8, 640), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(420, 360), ImGuiCond_Once);
    ImGui::Begin("Controls");

    ImGui::TextUnformatted("Throttle (hold W for 100%)");
    ImGui::SliderFloat("##throttle", &m_throttleUiF, 0.0f, 1.0f, "%.0f%%");
    ImGui::SameLine();
    if (ImGui::Button("Idle##btn")) m_throttleUiF = 0;
    ImGui::SameLine();
    if (ImGui::Button("50%##btn")) m_throttleUiF = 0.5f;
    ImGui::SameLine();
    if (ImGui::Button("WOT##btn")) m_throttleUiF = 1.0f;

    if (ImGui::Button(tel.starter ? "Starter ON (S)" : "Starter OFF (S)")) {
        m_engine->setStarter(!tel.starter);
    }
    ImGui::SameLine();
    if (ImGui::Button(tel.ignition ? "Ignition ON (A)" : "Ignition OFF (A)")) {
        m_engine->setIgnition(!tel.ignition);
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Audio");
    float vol = (float)m_synth.volume();
    if (ImGui::SliderFloat("volume", &vol, 0.0f, 2.0f)) m_synth.setVolume(vol);
    float jit = (float)m_synth.jitter();
    if (ImGui::SliderFloat("jitter", &jit, 0.0f, 1.0f)) m_synth.setJitter(jit);

    ImGui::Separator();
    ImGui::TextWrapped("Keys: W throttle 100%% | S starter | A ignition | F5 reload config");
    if (ImGui::Button("Reload engine (F5)")) reloadEngine();
    ImGui::End();
}

void App::drawDebug() {
    ImGui::SetNextWindowPos(ImVec2(8, 1010), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(420, 260), ImGuiCond_Once);
    ImGui::Begin("Debug (per-cylinder)");

    const auto &cyls = m_engine->cylinders();
    const double theta = m_engine->crankAngle();
    for (size_t i = 0; i < cyls.size(); ++i) {
        const auto &c = cyls[i];
        ImGui::PushID((int)i);
        ImGui::Text("cyl %d: P=%8.1f kPa  V=%7.1f cc  lift i=%.2f e=%.2f  %s",
                    (int)i,
                    c.gas.pressure() / 1000.0,
                    c.gas.V * 1e6,
                    c.intake_lift, c.exhaust_lift,
                    c.lit ? "LIT" : "   ");
        ImGui::PopID();
    }
    ImGui::Separator();
    ImGui::Text("theta = %.3f rad (%.1f deg)", theta, theta * 180.0 / M_PI);
    ImGui::Text("tapCollector = %+.2f Pa   tapTail = %+.2f Pa",
                 m_engine->exhaust().collectorSignal(),
                 m_engine->exhaust().tailSignal());
    const double latMs = 1000.0 * (double)m_synth.pendingInput() / std::max(m_simFreq, 1.0);
    ImGui::Text("audio latency: %.1f ms (target %.0f ms)",
                 latMs, m_cfg.audio.target_latency_s * 1000.0);
    ImGui::End();
}

} // namespace ice
