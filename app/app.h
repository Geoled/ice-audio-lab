// app/app.h — application shell: window, GL, audio device, sim thread
#pragma once
#include "../core/piston_engine.h"
#include "../core/config.h"
#include "../audio/synth.h"
#include <memory>
#include <string>
#include <vector>
#include <chrono>

// windows types needed by this header (kept out of the global namespace spam)
// WIN32_LEAN_AND_MEAN is defined by the build; it hides CS_CLASSDCS etc.,
// so pull the class styles back in explicitly.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#ifndef CS_CLASSDCS
#define CS_CLASSDCS 0x0001
#endif

namespace ice {

struct PerfGraph {
    static constexpr int N = 256;
    float values[N] = {};
    int pos = 0;
    double latest = 0;
    void push(double v) {
        values[pos] = (float)v;
        pos = (pos + 1) % N;
        latest = v;
    }
};

class App {
public:
    bool init(const std::string &engineJson);
    void run();
    void shutdown();

    // input state from WndProc
    bool keys[256] = {};
    bool keysPressed[256] = {};
    bool wantTextInput() const;  // true when an ImGui text field has focus

private:
    static LRESULT WINAPI WndProc(HWND, UINT, WPARAM, LPARAM);
    void simulateFrame(double dt);
    void drawUI();

    // window/GL
    HWND m_hwnd = nullptr;
    HDC m_hdc = nullptr;
    HGLRC m_hglrc = nullptr;
    void *m_glp = nullptr; // dummy context pointer

    // audio
    void *m_device = nullptr;     // ma_device*
    void *m_context = nullptr;    // ma_context*

    // sim
    std::unique_ptr<PistonEngine> m_engine;
    EngineConfig m_cfg;
    Synth m_synth;
    double m_simFreq = 18000;
    float m_throttleUiF = 0;       // ImGui slider (source of truth)
    double m_throttle = 0;         // engine target (W overrides while held)
    std::string m_enginePath;

    // perf
    PerfGraph m_fpsGraph, m_frameGraph, m_simLoadGraph, m_audioLoadGraph;
    double m_lastFrameTime = 0;
    double m_simEwmaMs = 0;
    int m_stepsThisFrame = 0;
    std::chrono::steady_clock::time_point m_lastStepTime;

    // live spectrum for UI (calculated on UI thread from waveBuf)
    float m_spectrum[512] = {};
    std::string m_reloadError;

    // ui helpers
    void drawDashboard();
    void drawPlots();
    void drawControls();
    void drawDebug();

    // hot-reload the engine config (F5)
    bool reloadEngine();

    // compute the spectrum from the synth waveform (Radix-2 FFT, 512 bins)
    void computeSpectrum();
};

} // namespace ice
