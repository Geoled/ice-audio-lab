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
    m_nTaps = std::max(1, nTaps);
    m_ring.initialize(1 << 16);
    m_filters.assign(m_nTaps, TapFilter{});
    m_prevFrame.assign(m_nTaps, 0.0);
    m_curFrame.assign(m_nTaps, 0.0);
    m_phase = 1.0; // force initial fill
    m_peak = 0;
    m_agcGain = 0.02;
}

void Synth::writeTaps(const double *taps, int nTaps, double sim_dt) {
    (void)sim_dt;
    if (nTaps != m_nTaps) return;
    m_ring.push(taps, nTaps);
}

void Synth::renderBlock(float *out, int frames) {
    double acc[8] = {0};

    for (int f = 0; f < frames; ++f) {
        // consume tap frames from the ring at a smoothed rate; backlog-based
        // ratio keeps latency near target without knowing sim rate here.
        double mono = 0;

        const size_t backlog = m_ring.size() / std::max(1, m_nTaps);
        // nominal: one tap frame per ~2.7 audio samples (18k sim / 48k audio)
        const double nominal = 48000.0 / std::max(m_simRateEstimate, 8000.0);
        double ratio = nominal;
        if (backlog > 4000) ratio = std::max(0.4, nominal * 0.5);     // catch up
        else if (backlog < 400) ratio = nominal * 2.0;                // slow down
        m_phase += 1.0 / std::max(ratio, 0.1);
        if (m_phase >= 1.0 && backlog > 0) {
            m_phase -= 1.0;
            double frame[8] = {0};
            if (m_ring.pop(frame, m_nTaps) == (size_t)m_nTaps) {
                m_prevFrame = m_curFrame;
                for (int t = 0; t < m_nTaps; ++t) m_curFrame[t] = frame[t];
            }
        }

        // mix taps with per-tap shaping
        for (int t = 0; t < m_nTaps && t < 8; ++t) {
            auto &fl = m_filters[t];
            const double in = m_curFrame[t];

            // jitter: small random delay-ish amplitude noise
            const double jit = 1.0 + m_p.jitter * (rng01(m_rng) * 2.0 - 1.0) * 0.25;

            // DC block
            const double dc = in - fl.dc_prev_in + 0.995 * fl.dc_prev_out;
            fl.dc_prev_in = in; fl.dc_prev_out = dc;

            // derivative component (HF "presence")
            const double deriv = (in - fl.last) * m_p.hf_gain * 4.0;
            fl.last = in;

            // air noise AM
            const double air = 1.0 + m_p.air_noise * (rng01(m_rng) * 2.0 - 1.0) * 0.35;

            mono += (dc * jit * air + deriv) * (t == 1 ? 0.8 : 1.0);
        }

        // soft clip & AGC
        const double target = 0.25;
        const double mag = std::abs(mono);
        m_peak = std::max(m_peak * 0.999, mag);
        if (m_peak > 1e-9) {
            const double desired = std::min(3.0, target / m_peak);
            m_agcGain += 0.02 * (desired - m_agcGain);
        }
        double s = mono * m_agcGain * m_p.volume;
        s = std::tanh(s);

        // out
        out[f * 2 + 0] = (float)std::clamp(s * 0.9, -1.0, 1.0);
        out[f * 2 + 1] = (float)std::clamp(s * 0.9, -1.0, 1.0);

        // waveform capture
        waveBuf[m_wavePos] = (float)s;
        m_wavePos = (m_wavePos + 1) % WAVE_CAP;
    }
}

} // namespace ice
