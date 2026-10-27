// audio/synth.h — synthesis pipeline: engine taps -> 48 kHz output
#pragma once
#include "ring_buffer.h"
#include "../core/engine.h"

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
    double m_simRateEstimate = 18000.0;
    uint32_t m_rng = 88172645;
    int m_wavePos = 0;
};

} // namespace ice
