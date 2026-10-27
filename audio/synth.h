// audio/synth.h — synthesis pipeline: exhaust pulses + intake noise + AGC
#pragma once
#include "ring_buffer.h"
#include "../core/engine.h"
#include <atomic>

namespace ice {

class Synth {
public:
    struct Params {
        double sample_rate = 48000;
        double jitter = 0.4;
        double air_noise = 0.65;
        double hf_gain = 0.012;
        double volume = 1.0;
    };

    void initialize(const Params &p, int nTaps);
    void setVolume(double v) { m_p.volume = std::clamp(v, 0.0, 2.0); }
    void setJitter(double v) { m_p.jitter = std::clamp(v, 0.0, 1.0); }
    double volume() const { return m_p.volume; }
    double jitter() const { return m_p.jitter; }

    // AGC peak decay: set so the peak tracker falls to ~37% in ~0.5 s
    void setSampleRate(double sr) {
        m_p.sample_rate = sr;
        m_peakDecay = std::pow(0.3679, 1.0 / (0.5 * std::max(sr, 8000.0)));
    }

    // sim thread: engine context for the noise layers (lock-free)
    void setEngineContext(double rpm, double induction01, int cylinderCount) {
        m_rpm.store(rpm, std::memory_order_relaxed);
        m_induction.store(std::clamp(induction01, 0.0, 1.0), std::memory_order_relaxed);
        m_cylCount = std::max(1, cylinderCount);
    }

    // simulation thread: pushes tap samples at sim rate (nTaps per step)
    void writeTaps(const double *taps, int nTaps, double sim_dt);

    // tell the synth the simulation rate so it can resample correctly
    void setSimRate(double hz) { m_simRateEstimate = std::max(hz, 8000.0); }

    // audio thread: fills interleaved stereo block
    void renderBlock(float *out, int frames);

    // telemetry for UI
    double lastOutPeak() const { return m_peak; }
    size_t pendingInput() const { return m_ring.size() / std::max(1, m_nTaps); }

    // waveform capture for the UI (mono, recent); power-of-two size
    static constexpr int WAVE_CAP = 4096;
    float waveBuf[WAVE_CAP] = {};
    int waveHead() const { return m_wavePos; }  // for spectrum windowing

private:
    struct TapFilter {
        double dc_prev_in = 0, dc_prev_out = 0;
        double last = 0;
    };

    Params m_p;
    int m_nTaps = 2;
    SpscRing<double> m_ring;      // sim-rate tap frames (nTaps interleaved)
    std::vector<TapFilter> m_filters;
    double m_prev[8] = {}, m_cur[8] = {};
    double m_phase = 0;
    double m_rateErr = 0;
    double m_peak = 0;
    double m_agcGain = 1.0;
    double m_peakDecay = 0.99995;
    double m_simRateEstimate = 18000.0;

    // engine context (atomics: sim thread writes, audio thread reads)
    std::atomic<double> m_rpm{0};
    std::atomic<double> m_induction{0};
    int m_cylCount = 4;

    // intake-noise layer state
    double m_noiseLp = 0;
    double m_noiseState = 0;
    double m_firingPhase = 0;
    double m_fireLp = 0;

    uint32_t m_rng = 88172645;
    int m_wavePos = 0;
};

} // namespace ice
