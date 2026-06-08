#include "bumpstop.h"

#include <config.h>
#include <utils.h>
#include <system_state.h>

void BumpStop::init()
{
    counts_to_rad_ = TWO_PI / ENCODER_CPR;
    updateLimits();
}

void BumpStop::updateLimits()
{
    half_range_rad_ = angleToRadians(static_cast<float>(g_state.steering_range_deg) * 0.5f);
    transition_rad_ = angleToRadians(BUMPSTOP_RANGE_DEG);

    if (transition_rad_ < 0.001f)
        transition_rad_ = 0.001f;
    inv_transition_rad_ = 1.0f / transition_rad_;

    stiffness_ = g_state.bumpstop_stiffness;
    damping_ = BUMPSTOP_DAMPING;

    wall_start_pos_ = half_range_rad_ - transition_rad_;
    wall_start_neg_ = -half_range_rad_ + transition_rad_;

    torque_clamp_ = MAX_CURRENT;
}

float BumpStop::calculate(int32_t position_counts, float velocity_rad_s)
{
    const float angle = static_cast<float>(position_counts) * counts_to_rad_;

    float torque = 0.0f;

    if (angle > wall_start_pos_)
    {
        float penetration = angle - wall_start_pos_;
        float x = penetration * inv_transition_rad_;
        x = clamp(x, 0.0f, 1.0f);

        float spring = stiffness_ * (x + 0.5f * cube(x));

        float damping_gain = 0.2f + 0.8f * x;
        float damping = damping_ * damping_gain * velocity_rad_s;

        torque = -spring - damping;
    }
    else if (angle < wall_start_neg_)
    {
        float penetration = wall_start_neg_ - angle;
        float x = penetration * inv_transition_rad_;
        x = clamp(x, 0.0f, 1.0f);

        float spring = stiffness_ * (x + 0.5f * cube(x));

        float damping_gain = 0.2f + 0.8f * x;
        float damping = damping_ * damping_gain * velocity_rad_s;

        torque = spring + damping;
    }

    if (abs(velocity_rad_s) < 0.3f && torque != 0.0f)
    {
        float sign = (torque > 0.0f) ? 1.0f : -1.0f;
        torque -= 0.02f * sign;
    }

    torque = clamp<float>(torque, -torque_clamp_, torque_clamp_);

    enable_filtered_ += 0.01f * (g_state.bumpstop_enabled - enable_filtered_);
    return enable_filtered_ * torque;
}