#include "telemetry.h"

#include <algorithm>
#include <cstring>

#include <tusb.h>

namespace
{
    constexpr float kScale[TLM_COUNT] = {
        1000.0f, // Ia  (mA,  +-32 A)
        1000.0f, // Ib  (mA,  +-32 A)
        1000.0f, // Ic  (mA,  +-32 A)
        1000.0f, // Id  (mA)
        1000.0f, // Iq  (mA)
        1000.0f, // Iq* (mA)
        100.0f,  // Vd  (10 mV, +-327 V)
        100.0f,  // Vq  (10 mV, +-327 V)
        100.0f,  // Vbus
        1000.0f, // Ibus    (mA)
        5000.0f, // elec angle (0.2 mrad, +-6.5 rad)
        10.0f,   // wheel angle (0.1 deg, +-3276 deg)
        100.0f,  // velocity   (0.01 rad/s)
    };

    constexpr uint8_t kVendorHidInstance = 1;
    constexpr uint32_t kPacketsPerSecond = 1000;

    inline int16_t toFixed(float v, float scale)
    {
        float s = v * scale;
        s = std::clamp(s, -32767.0f, 32767.0f);
        return static_cast<int16_t>(s);
    }
}

bool Telemetry::start(uint16_t mask, uint32_t pwm_hz, uint32_t now_ms)
{
    stop();

    uint8_t count = 0;
    for (uint8_t ch = 0; ch < TLM_COUNT; ch++)
    {
        if (mask & (1u << ch))
        {
            if (count == kMaxChannels)
                return false;
            channels_[count++] = ch;
        }
    }
    if (count == 0 || pwm_hz == 0)
        return false;

    mask_ = mask;
    channel_count_ = count;
    frames_per_packet_ = static_cast<uint8_t>((TELEMETRY_PAYLOAD - kHeader) / (2u * count));

    const uint32_t frames_per_second = (kPacketsPerSecond * frames_per_packet_ * 9u) / 10u;
    decimation_ = std::max<uint32_t>(1u, (pwm_hz + frames_per_second - 1u) / frames_per_second);
    sample_rate_hz_ = static_cast<float>(pwm_hz) / static_cast<float>(decimation_);

    head_ = 0;
    tail_ = 0;
    overflow_ = false;
    decim_counter_ = 0;
    seq_ = 0;
    last_keepalive_ms_ = now_ms;
    active_ = true;
    return true;
}

void Telemetry::stop()
{
    active_ = false;
}

void Telemetry::push(const float (&values)[TLM_COUNT])
{
    const uint16_t next = static_cast<uint16_t>((head_ + 1u) % kFrames);
    if (next == tail_)
    {
        overflow_ = true;
        return;
    }
    int16_t *frame = ring_[head_];
    for (uint8_t i = 0; i < channel_count_; i++)
    {
        const uint8_t ch = channels_[i];
        frame[i] = toFixed(values[ch], kScale[ch]);
    }
    head_ = next;
}

void Telemetry::poll(uint32_t now_ms)
{
#ifdef DEBUG
    (void)now_ms;
    stop(); // no vendor interface in the debug build
#else
    if (!active_)
        return;

    if (now_ms - last_keepalive_ms_ > kKeepaliveTimeoutMs)
    {
        stop();
        return;
    }

    if (!tud_mounted() || !tud_hid_n_ready(kVendorHidInstance))
        return;

    const uint16_t available = static_cast<uint16_t>((head_ + kFrames - tail_) % kFrames);
    if (available == 0)
        return;

    const uint8_t frames = static_cast<uint8_t>(std::min<uint16_t>(available, frames_per_packet_));
    uint8_t packet[TELEMETRY_PAYLOAD] = {};
    packet[0] = static_cast<uint8_t>(seq_ & 0xFF);
    packet[1] = static_cast<uint8_t>(seq_ >> 8);
    packet[2] = frames;
    packet[3] = overflow_ ? 0x01 : 0x00;
    packet[4] = static_cast<uint8_t>(mask_ & 0xFF);
    packet[5] = static_cast<uint8_t>(mask_ >> 8);
    overflow_ = false;

    uint8_t *out = packet + kHeader;
    uint16_t t = tail_;
    for (uint8_t f = 0; f < frames; f++)
    {
        std::memcpy(out, ring_[t], 2u * channel_count_);
        out += 2u * channel_count_;
        t = static_cast<uint16_t>((t + 1u) % kFrames);
    }

    if (tud_hid_n_report(kVendorHidInstance, TELEMETRY_REPORT_ID, packet, sizeof(packet)))
    {
        tail_ = t;
        seq_++;
    }
#endif
}
