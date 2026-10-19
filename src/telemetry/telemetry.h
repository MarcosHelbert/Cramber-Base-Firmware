#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <cstdint>

constexpr uint8_t TELEMETRY_REPORT_ID = 25;
constexpr uint8_t TELEMETRY_PAYLOAD = 63;

enum TelemetryChannel : uint8_t
{
    TLM_IA = 0,      // A
    TLM_IB,          // A
    TLM_IC,          // A
    TLM_ID,          // A
    TLM_IQ,          // A
    TLM_IQ_REF,      // A
    TLM_VD,          // V
    TLM_VQ,          // V
    TLM_VBUS,        // V
    TLM_IBUS,        // A (estimated)
    TLM_ELEC_ANGLE,  // rad
    TLM_WHEEL_ANGLE, // deg
    TLM_VELOCITY,    // rad/s
    TLM_COUNT
};

class Telemetry
{
public:
    static constexpr uint8_t kMaxChannels = 8;

    bool start(uint16_t mask, uint32_t pwm_hz, uint32_t now_ms);
    void stop();
    void keepalive(uint32_t now_ms) { last_keepalive_ms_ = now_ms; }

    bool active() const { return active_; }
    float sampleRateHz() const { return sample_rate_hz_; }

    inline bool due()
    {
        if (!active_)
            return false;
        if (++decim_counter_ < decimation_)
            return false;
        decim_counter_ = 0;
        return true;
    }

    void push(const float (&values)[TLM_COUNT]);

    void poll(uint32_t now_ms);

private:
    static constexpr uint16_t kFrames = 512;
    static constexpr uint32_t kKeepaliveTimeoutMs = 3000;
    static constexpr uint8_t kHeader = 6;

    volatile bool active_ = false;
    uint16_t mask_ = 0;
    uint8_t channels_[kMaxChannels] = {};
    uint8_t channel_count_ = 0;
    uint8_t frames_per_packet_ = 0;
    uint32_t decimation_ = 1;
    uint32_t decim_counter_ = 0;
    float sample_rate_hz_ = 0.0f;
    uint32_t last_keepalive_ms_ = 0;

    int16_t ring_[kFrames][kMaxChannels] = {};
    volatile uint16_t head_ = 0;
    volatile uint16_t tail_ = 0;
    volatile bool overflow_ = false;
    uint16_t seq_ = 0;
};

#endif // TELEMETRY_H
