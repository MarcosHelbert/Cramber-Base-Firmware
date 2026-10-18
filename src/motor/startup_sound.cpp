#include "startup_sound.h"

#include <algorithm>
#include <cmath>

#include <FastMath.h>
#include <utils.h>

namespace
{
    constexpr float kTotalDurationS = 1.9f;
    constexpr float kTailFadeS = 0.25f;
    constexpr float kAttackS = 0.003f;
    constexpr float kHarmonicLevel = 0.30f;
    constexpr float kHarmonicDecayRatio = 2.5f;
    constexpr float kPitchShift = 4.0f;
    constexpr float kMaxSoundCurrentA = 2.0f;
}

const StartupSound::Voice StartupSound::kVoices[] = {
    {440.00f, 0, 0.55f, 450},   // A4
    {554.37f, 145, 0.55f, 450}, // C#5
    {659.25f, 285, 0.60f, 450}, // E5
    {554.37f, 500, 0.40f, 350}, // C#5
    {659.25f, 615, 0.45f, 380}, // E5
    {880.00f, 760, 0.75f, 650}, // A5
    {440.00f, 765, 0.30f, 700}, // A4
};
const uint32_t StartupSound::kVoiceCount = sizeof(kVoices) / sizeof(kVoices[0]);

bool StartupSound::start()
{
    if (!config_.motor_calibrated || config_.phase_resistance <= 0.0f)
        return false;
    if (kVoiceCount > kMaxVoices)
        return false;

    dt_ = 1.0f / static_cast<float>(config_.pwm_freq_hz);
    time_s_ = 0.0f;
    peak_voltage_v_ = 0.0f;
    peak_current_a_ = 0.0f;

    const float current = std::clamp(config_.startup_sound_volume_a, 0.0f, kMaxSoundCurrentA);
    auto impedance = [&](float f)
    {
        const float wl = TWO_PI * f * config_.phase_inductance;
        return std::sqrt(square(config_.phase_resistance) + square(wl));
    };

    for (uint32_t i = 0; i < kVoiceCount; i++)
    {
        const Voice &voice = kVoices[i];
        const float tau = static_cast<float>(voice.decay_ms) * 0.001f;
        const float freq = voice.freq_hz * kPitchShift;

        phase_[i] = 0.0f;
        phase_step_[i] = TWO_PI * freq * dt_;
        fund_v_[i] = voice.gain * current * impedance(freq);
        harm_v_[i] = voice.gain * kHarmonicLevel * current * impedance(2.0f * freq);
        env_[i] = 1.0f;
        env_harm_[i] = 1.0f;
        decay_k_[i] = std::exp(-dt_ / tau);
        decay_harm_k_[i] = std::exp(-dt_ * kHarmonicDecayRatio / tau);
    }

    playing_ = true;
    return true;
}

float StartupSound::update(float vbus)
{
    if (!playing_)
        return 0.0f;

    float v = 0.0f;
    for (uint32_t i = 0; i < kVoiceCount; i++)
    {
        const float age = time_s_ - static_cast<float>(kVoices[i].start_ms) * 0.001f;
        if (age < 0.0f)
            continue;

        float s, c;
        FastMath::fast_sincos(phase_[i], &s, &c);
        const float attack = (age < kAttackS) ? (age / kAttackS) : 1.0f;
        v += attack * (fund_v_[i] * env_[i] * s + harm_v_[i] * env_harm_[i] * 2.0f * s * c);

        phase_[i] += phase_step_[i];
        if (phase_[i] >= TWO_PI)
            phase_[i] -= TWO_PI;
        env_[i] *= decay_k_[i];
        env_harm_[i] *= decay_harm_k_[i];
    }

    const float remaining = kTotalDurationS - time_s_;
    if (remaining < kTailFadeS)
        v *= std::max(remaining / kTailFadeS, 0.0f);

    const float v_limit = 0.95f * vbus * INV_SQRT3;
    v = std::clamp(v, -v_limit, v_limit);
    peak_voltage_v_ = std::max(peak_voltage_v_, std::abs(v));

    time_s_ += dt_;
    if (time_s_ >= kTotalDurationS)
        playing_ = false;

    return v;
}
