#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <cstdint>

struct SystemState
{
    // Encoder
    int32_t mechanical_offset = 0;

    // Motor parameters
    float encoder_offset_rad = 0.0f;
    float phase_resistance_ = 0.0f;
    float phase_inductance_ = 0.0f;

    // Bumpstops
    bool bumpstop_enabled = true;
    float bumpstop_stiffness = 5.0f;

    // Wheel parameters
    uint16_t steering_range_deg = 900;
};

extern SystemState g_state;

#endif // SYSTEM_STATE_H