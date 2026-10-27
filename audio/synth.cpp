// audio/synth.cpp — resampling, shaping, AGC, intake-noise layer
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
    m_rpm = 0;
    m_noiseLp = 0;
    m_noiseState = 0;
    m_firingPhase = 0;
    m_fireLp = 0;
}

void Synth::writeTaps(const double *taps, int nTaps, double sim_dt) {
    (void)sim_dt;
    if (nTaps != m_nTaps) return;
    m_ring.push(taps, nTaps);
}

void Synth::renderBlock(float *out, int frames) {
    // fixed resample ratio: one tap frame per (sampleRate/simRate) samples.
    const double step = std::max(m_p.sample_rate / std::max(m_simRateEstimate, 8000.0), 1.0);

    // engine context (atomic, written by the sim thread)
    const double rpm = m_rpm.load(std::memory_order_relaxed);
    const double indNoise = m_induction.load(std::memory_order_relaxed);

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

        // ---- layer 1: exhaust pulses (linear-interpolated tap) ----
        const double a = m_phase / step;
        double pulse = 0;
        for (int t = 0; t < m_nTaps; ++t) {
            auto &fl = m_filters[t];
            const double in = m_prev[t] + (m_cur[t] - m_prev[t]) * a;

            const double jit = 1.0 + m_p.jitter * (rng01(m_rng) * 2.0 - 1.0) * 0.25;

            // DC block
            const double dc = in - fl.dc_prev_in + 0.995 * fl.dc_prev_out;
            fl.dc_prev_in = in; fl.dc_prev_out = dc;

            // derivative (HF presence)
            const double deriv = (in - fl.last) * m_p.hf_gain * 4.0;
            fl.last = in;

            const double air = 1.0 + m_p.air_noise * (rng01(m_rng) * 2.0 - 1.0) * 0.35;

            const double tapGain = (t == 1) ? 0.8 : 1.0;
            pulse += (dc * jit * air + deriv) * tapGain;
        }

        // ---- layer 2: intake / induction noise (band-limited, rpm-scaled) ----
        // white noise -> one-pole LP (intake roar ~ 200-1200 Hz band)
        const double white = rng01(m_rng) * 2.0 - 1.0;
        m_noiseLp += 0.12 * (white - m_noiseLp);
        const double intakeGain =
            0.06 * indNoise * (1.0 + 0.5 * indNoise);   // up to ~0.09
        const double intake = m_noiseLp * intakeGain * 2.0;

        // ---- layer 3: firing-order body tone (gives idle its "heartbeat") ----
        // sub-audio AM of the noise at the firing frequency: each firing event
        // knocks the intake noise a little -> perceivable idle cadence even
        // when exhaust taps are tiny.
        const double firingHz = std::max(m_cylCount * rpm / 120.0, 0.0);
        if (firingHz > 1e-3) {
            m_firingPhase += firingHz / m_p.sample_rate;
            if (m_firingPhase >= 1.0) m_firingPhase -= 1.0;
        }
        const double knock = std::exp(-6.0 * m_firingPhase) * (firingHz > 1.0 ? 1.0 : 0.0);
        const double body = intake * (1.0 + 2.5 * knock);

        double mono = pulse + body;

        // ---- AGC: peak follower with fast attack and ~0.3 s release. ----
        // The gain tracks target/peak continuously (no long integrator lag):
        // when the signal weakens, peak decays within ~0.3 s and the gain
        // comes back up immediately — no silence gap, no "white noise only"
        // recovery tail.
        const double target = 0.25;
        const double mag = std::abs(mono);
        if (mag > m_peak) {
            m_peak = mag;                       // instant attack
        } else {
            m_peak *= m_peakDecay;              // release ~0.3 s
            m_peak = std::fmax(m_peak, mag);
        }
        m_peak = std::clamp(m_peak, 1e-4, 10.0);
        double desired = std::clamp(target / m_peak, 0.05, 8.0);
        // smooth the gain toward desired (fast both ways, ~10 ms)
        const double kUp = std::clamp(2.0 / m_p.sample_rate * 48000.0, 0.02, 0.5);
        const double kDown = kUp * 0.5;
        const double kk = (desired < m_agcGain) ? kDown : kUp;
        m_agcGain += kk * (desired - m_agcGain);
        m_agcGain = std::clamp(m_agcGain, 0.05, 8.0);
        double s = mono * m_agcGain * m_p.volume;
        s = std::tanh(s);

        out[f * 2 + 0] = (float)std::clamp(s * 0.9, -1.0, 1.0);
        out[f * 2 + 1] = (float)std::clamp(s * 0.9, -1.0, 1.0);

        waveBuf[m_wavePos] = (float)s;
        m_wavePos = (m_wavePos + 1) & (WAVE_CAP - 1);
    }
}

} // namespace ice
