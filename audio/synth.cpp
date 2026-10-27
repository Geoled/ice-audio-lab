// audio/synth.cpp — resampling, shaping, AGC
#include "synth.h"
#include <cmath>
#include <algorithm>

namespace ice {

static inline double rng01(uint32_t &s) {
    s = s * 1664525u + 1013904223u;
    return (s >> 8) / 16777216.0;
}

void Synth::initialize(const Params &p, int nTaps) {
    m_p = p;
    m_nTaps = std::min(std::max(1, nTaps), 8);
    m_ring.initialize(1 << 16);
    m_filters.assign(m_nTaps, TapFilter{});
    for (int t = 0; t < m_nTaps; ++t) {
        m_prev[t] = 0.0;
        m_cur[t] = 0.0;
    }
    m_phase = 1.0; // force initial fill
    m_peak = 0;
    m_agcGain = 0.02;
    m_rateErr = 0;
}

void Synth::writeTaps(const double *taps, int nTaps, double sim_dt) {
    (void)sim_dt;
    if (nTaps != m_nTaps) return;
    m_ring.push(taps, nTaps);
}

void Synth::renderBlock(float *out, int frames) {
    // fixed resample ratio: one tap frame per (sampleRate/simRate) samples.
    // the sim thread adapts its production rate to hold latency, so a fixed
    // ratio is correct and glitch-free.
    const double step = std::max(m_p.sample_rate / std::max(m_simRateEstimate, 8000.0), 1.0);

    for (int f = 0; f < frames; ++f) {
        m_phase += 1.0;
        if (m_phase >= step) {
            m_phase -= step;
            double frame[8] = {0};
            if (m_ring.size() >= (size_t)m_nTaps) {
                m_ring.pop(frame, m_nTaps);
                for (int t = 0; t < m_nTaps; ++t) {
                    m_prev[t] = m_cur[t];
                    m_cur[t] = frame[t];
                }
            }
        }

        // linear interpolation between previous and current tap frame
        const double a = m_phase / step; // 0..1
        double mono = 0;
        for (int t = 0; t < m_nTaps; ++t) {
            auto &fl = m_filters[t];
            const double in = m_prev[t] + (m_cur[t] - m_prev[t]) * a;

            // jitter: multiplicative noise on the raw signal
            const double jit = 1.0 + m_p.jitter * (rng01(m_rng) * 2.0 - 1.0) * 0.25;

            // DC block
            const double dc = in - fl.dc_prev_in + 0.995 * fl.dc_prev_out;
            fl.dc_prev_in = in; fl.dc_prev_out = dc;

            // derivative component (HF presence)
            const double deriv = (in - fl.last) * m_p.hf_gain * 4.0;
            fl.last = in;

            // air noise AM
            const double air = 1.0 + m_p.air_noise * (rng01(m_rng) * 2.0 - 1.0) * 0.35;

            const double tapGain = (t == 1) ? 0.8 : 1.0;
            mono += (dc * jit * air + deriv) * tapGain;
        }

        // AGC (attack/release asymmetric) with a noise floor so the engine
        // is audible from the very first firing pulse
        const double target = 0.25;
        const double mag = std::abs(mono);
        m_peak = std::max(m_peak * 0.9995, mag);
        double desired = 3.0;
        if (m_peak > 1e-9) {
            desired = std::min(3.0, target / m_peak);
        }
        const double k = (desired < m_agcGain) ? 0.05 : 0.002; // fast down, slow up
        m_agcGain += k * (desired - m_agcGain);
        m_agcGain = std::clamp(m_agcGain, 1e-4, 4.0);
        double s = mono * m_agcGain * m_p.volume;
        s = std::tanh(s);

        out[f * 2 + 0] = (float)std::clamp(s * 0.9, -1.0, 1.0);
        out[f * 2 + 1] = (float)std::clamp(s * 0.9, -1.0, 1.0);

        waveBuf[m_wavePos] = (float)s;
        m_wavePos = (m_wavePos + 1) & (WAVE_CAP - 1);
    }
}

} // namespace ice
