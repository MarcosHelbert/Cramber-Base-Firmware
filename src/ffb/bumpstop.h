#ifndef BUMPSTOP_H
#define BUMPSTOP_H

#include <cstdint>

class BumpStop
{
public:
    void init();

    void updateLimits();

    float calculate(int32_t position_counts, float velocity_rad_s);

private:
    float counts_to_rad_ = 0.0f;

    float half_range_rad_ = 0.0f;
    float transition_rad_ = 0.0f;
    float inv_transition_rad_ = 0.0f;

    float stiffness_ = 0.0f;
    float damping_ = 0.0f;

    float wall_start_pos_ = 0.0f;
    float wall_start_neg_ = 0.0f;

    float torque_clamp_ = 0.0f;

    float enable_filtered_ = 0.0f;
};

#endif // BUMPSTOP_H