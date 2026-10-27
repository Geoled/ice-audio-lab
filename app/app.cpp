// app/app.cpp — Win32 + OpenGL3 + miniaudio shell, sim loop
#include "app.h"
#include "imgui.h"
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_opengl3.h"
#include "implot.h"
#include <miniaudio.h>
#include <cmath>
#include <thread>
#include <algorithm>

// bring in the imgui-owned minimal GL loader (defines glClearColor etc.)
#include "imgui_impl_opengl3_loader.h"

// forward-declared per imgui_impl_win32.h instructions
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

namespace ice {

static App *g_app = nullptr;

// ---------------------------------------------------------------------------
// audio callback
static void audioCallback(ma_device *device, void *output, const void *, ma_uint32 frames) {
    auto *synth = (Synth *)device->pUserData;
    synth->renderBlock((float *)output, (int)frames);
}

// ---------------------------------------------------------------------------
bool App::init(const std::string &engineJson) {
    g_app = this;
    m_enginePath = engineJson;

    // load engine
    std::string err;
    if (!loadEngineJson(engineJson, m_cfg, err)) {
        MessageBoxA(nullptr, err.c_str(), "ice-audio-lab: config error", MB_ICONERROR);
        return false;
    }

    // auto sim frequency
    m_simFreq = m_cfg.audio.sim_freq_hz;
    if (m_simFreq <= 0) {
        const double firing = (double)m_cfg.cylinders.size() * 4000.0 / 120.0; // rough: 4000rpm
        m_simFreq = std::clamp(std::max(22.0 * firing * 2.0, 18000.0), 8000.0, 44000.0);
    }

    // ---------------- window ----------------
    WNDCLASSEXA wc = { sizeof(WNDCLASSEXA), CS_CLASSDCS, WndProc, 0L, 0L,
                      GetModuleHandle(nullptr), nullptr, nullptr, nullptr, nullptr,
                      "ice-audio-lab", nullptr };
    RegisterClassExA(&wc);    m_hwnd = CreateWindowA(wc.lpszClassName, "ice-audio-lab — procedural engine audio",
                          WS_OVERLAPPEDWINDOW, 100, 100, 1600, 1000,
                          nullptr, nullptr, wc.hInstance, this);
    if (!m_hwnd) return false;

    // GL context
    m_hdc = GetDC(m_hwnd);
    PIXELFORMATDESCRIPTOR pfd = {};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DOUBLEBUFFER | PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    const int pf = ChoosePixelFormat(m_hdc, &pfd);
    SetPixelFormat(m_hdc, pf, &pfd);
    HGLRC tmp = wglCreateContext(m_hdc);
    wglMakeCurrent(m_hdc, tmp);

    // modern context
    typedef HGLRC (WINAPI *PFNWGLCREATEATTRIB)(HDC, const int *);
    auto wglCreateAttrib = (PFNWGLCREATEATTRIB)wglGetProcAddress("wglCreateContextAttribsARB");
    if (wglCreateAttrib) {
        const int attribs[] = { 0x2091, 3, 0x2092, 3, 0 }; // WGL_CONTEXT_MAJOR/_MINOR
        m_hglrc = wglCreateAttrib(m_hdc, attribs);
        wglMakeCurrent(m_hdc, m_hglrc);
        wglDeleteContext(tmp);
    } else {
        m_hglrc = tmp;
    }
    typedef BOOL (WINAPI *PFNWGLSWAPINTERVAL)(int);
    auto swapFn = (PFNWGLSWAPINTERVAL)wglGetProcAddress("wglSwapIntervalEXT");
    if (swapFn) swapFn(0); // uncapped: frame pacing in run()

    // ---------------- ImGui ----------------
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableKeyboard; // keep W/S/A/D for the app
    ImGui::StyleColorsDark();
    ImGui_ImplWin32_InitForOpenGL(m_hwnd);
    ImGui_ImplOpenGL3_Init("#version 130");

    // ---------------- engine + synth ----------------
    m_engine = std::make_unique<PistonEngine>(m_cfg);
    Synth::Params sp;
    sp.sample_rate = m_cfg.audio.sample_rate_hz;
    sp.jitter = m_cfg.audio.jitter;
    sp.air_noise = m_cfg.audio.air_noise;
    sp.hf_gain = m_cfg.audio.hf_gain;
    sp.volume = 1.0;
    m_synth.initialize(sp, 2);
    m_synth.setSimRate(m_simFreq);

    // ---------------- audio device ----------------
    ma_device *dev = new ma_device();
    ma_device_config dc = ma_device_config_init(ma_device_type_playback);
    dc.playback.format = ma_format_f32;
    dc.playback.channels = 2;
    dc.sampleRate = (int)m_cfg.audio.sample_rate_hz;
    dc.dataCallback = audioCallback;
    dc.pUserData = &m_synth;
    if (ma_device_init(nullptr, &dc, dev) != MA_SUCCESS) {
        MessageBoxA(m_hwnd, "audio init failed", "ice-audio-lab", MB_ICONERROR);
        return false;
    }
    ma_device_start(dev);
    m_device = dev;

    m_lastStepTime = std::chrono::steady_clock::now();
    ShowWindow(m_hwnd, SW_SHOWDEFAULT);
    UpdateWindow(m_hwnd);
    return true;
}

bool App::wantTextInput() const {
    return ImGui::GetCurrentContext() != nullptr &&
           ImGui::GetIO().WantTextInput;
}

void App::run() {
    MSG msg;
    ZeroMemory(&msg, sizeof(msg));
    const double dtFrame = 1.0 / 60.0;

    while (msg.message != WM_QUIT) {
        const auto t0 = std::chrono::steady_clock::now();

        while (PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
            if (msg.message == WM_QUIT) break;
        }
        if (msg.message == WM_QUIT) break;

        // throttle: W wins while held (but not when typing into a field)
        const bool typing = wantTextInput();
        if (!typing && keys['W']) {
            m_throttle = 1.0;          // momentary WOT
        } else {
            m_throttle = m_throttleUiF; // slider is the source of truth
        }

        if (!typing && keysPressed['S']) {
            m_engine->setStarter(!m_engine->telemetry().starter);
        }
        if (!typing && keysPressed['A']) {
            m_engine->setIgnition(!m_engine->telemetry().ignition);
        }
        std::fill(keysPressed, keysPressed + 256, false);

        m_engine->setThrottle(m_throttle);

        // ---------------- simulate ----------------
        simulateFrame(dtFrame);

        // ---------------- UI ----------------
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        drawUI();

        ImGui::Render();
        int w, h;
        RECT rc; GetClientRect(m_hwnd, &rc);
        w = rc.right - rc.left; h = rc.bottom - rc.top;
        glViewport(0, 0, w, h);
        glClearColor(0.06f, 0.06f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        SwapBuffers(m_hdc);

        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        const double fps = 1000.0 / std::max(ms, 0.01);
        m_fpsGraph.push(fps);
        m_frameGraph.push(ms);
        if (ms < 100) std::this_thread::sleep_for(std::chrono::milliseconds(
            (int)std::max(0.0, dtFrame * 1000.0 - ms)));
    }
}

void App::simulateFrame(double dt) {
    if (!m_engine) return;

    // adaptive step count to keep synth fed at target latency
    const double targetLatency = m_cfg.audio.target_latency_s;
    const double backlog = (double)m_synth.pendingInput() / 2.0 / m_simFreq; // seconds of sim audio
    double steps = dt * m_simFreq;
    if (backlog < targetLatency) steps *= 1.25;
    else if (backlog > targetLatency * 2.0) steps *= 0.8;

    const double dtSim = 1.0 / m_simFreq;
    const int n = (int)steps;
    const auto t0 = std::chrono::steady_clock::now();

    double taps[2] = {0, 0};
    for (int i = 0; i < n; ++i) {
        m_engine->step(dtSim);
        m_engine->computeAudioTap(taps, 2);
        m_synth.writeTaps(taps, 2, dtSim);
    }
    m_stepsThisFrame = n;

    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    m_simEwmaMs = m_simEwmaMs * 0.9 + ms * 0.1;
    m_simLoadGraph.push(100.0 * m_simEwmaMs / (dt * 1000.0));
    (void)targetLatency;
}

void App::shutdown() {
    if (m_device) {
        ma_device_uninit((ma_device *)m_device);
        delete (ma_device *)m_device;
        m_device = nullptr;
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    if (m_hglrc) { wglMakeCurrent(nullptr, nullptr); wglDeleteContext(m_hglrc); }
    if (m_hdc) ReleaseDC(m_hwnd, m_hdc);
    if (m_hwnd) DestroyWindow(m_hwnd);
}

LRESULT WINAPI App::WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    if (ImGui_ImplWin32_WndProcHandler(hwnd, msg, wp, lp)) return 1;
    App *app = g_app;
    switch (msg) {
    case WM_SIZE: return 0;
    case WM_KEYDOWN: {
        if (app && wp < 256) {
            if (!app->keys[wp]) app->keysPressed[wp] = true;
            app->keys[wp] = true;
        }
        return 0;
    }
    case WM_KEYUP:
        if (app && wp < 256) app->keys[wp] = false;
        return 0;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

} // namespace ice
