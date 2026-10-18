#ifndef STARTUP_SOUND_H
#define STARTUP_SOUND_H

#include <algorithm>
#include <cstdint>

#include <persistence/config_store.h>

class StartupSound
{
public:
    explicit StartupSound(const PersistedConfig &config) : config_(config) {}

    bool start();
    void stop() { playing_ = false; }
    bool isPlaying() const { return playing_; }

    float update(float vbus);

    void notePhaseCurrent(float ia) { peak_current_a_ = std::max(peak_current_a_, ia < 0.0f ? -ia : ia); }
    float getPeakVoltage() const { return peak_voltage_v_; }
    float getPeakCurrent() const { return peak_current_a_; }

private:
    struct Voice
    {
        float freq_hz;
        uint16_t start_ms;
        float gain;
        uint16_t decay_ms;
    };

    static constexpr uint32_t kMaxVoices = 8;
    static const Voice kVoices[];
    static const uint32_t kVoiceCount;

    const PersistedConfig &config_;
    float dt_ = 0.0f;

    volatile bool playing_ = false;
    float time_s_ = 0.0f;
    float peak_voltage_v_ = 0.0f;
    float peak_current_a_ = 0.0f;

    float phase_[kMaxVoices] = {};
    float phase_step_[kMaxVoices] = {};
    float fund_v_[kMaxVoices] = {};
    float harm_v_[kMaxVoices] = {};
    float env_[kMaxVoices] = {};
    float env_harm_[kMaxVoices] = {};
    float decay_k_[kMaxVoices] = {};
    float decay_harm_k_[kMaxVoices] = {};
};

#endif // STARTUP_SOUND_H
