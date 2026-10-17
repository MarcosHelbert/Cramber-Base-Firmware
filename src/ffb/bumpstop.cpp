#include "bumpstop.h"

#include <algorithm>
#include <cmath>

#include <persistence/config_store.h>
#include <utils.h>

void BumpStop::init()
{
    updateLimits();
}

void BumpStop::updateLimits()
{
    enabled_ = g_config.bumpstop_enabled;

    half_range_rad_ = angleToRadians(g_config.steering_range_deg * 0.5f);
    transition_rad_ = angleToRadians(g_config.bumpstop_range_deg);

    if (transition_rad_ < 0.001f)
        transition_rad_ = 0.001f;
    inv_transition_rad_ = 1.0f / transition_rad_;

    stiffness_ = g_config.bumpstop_stiffness;
    damping_ = g_config.bumpstop_damping;

    wall_start_pos_ = half_range_rad_ - transition_rad_;
    wall_start_neg_ = -half_range_rad_ + transition_rad_;

    torque_clamp_ = g_config.max_current;
}

float BumpStop::calculate(float position_rad, float velocity_rad_s) const
{
    if (!enabled_)
        return 0.0f;

    float torque = 0.0f;

    if (position_rad > wall_start_pos_)
    {
        float penetration = position_rad - wall_start_pos_;
        float x = penetration * inv_transition_rad_;
        x = std::clamp(x, 0.0f, 1.0f);

        float spring = stiffness_ * (x + 0.5f * cube(x));

        float damping_gain = 0.2f + 0.8f * x;
        float damping = damping_ * damping_gain * velocity_rad_s;

        torque = -spring - damping;
    }
    else if (position_rad < wall_start_neg_)
    {
        float penetration = wall_start_neg_ - position_rad;
        float x = penetration * inv_transition_rad_;
        x = std::clamp(x, 0.0f, 1.0f);

        float spring = stiffness_ * (x + 0.5f * cube(x));

        float damping_gain = 0.2f + 0.8f * x;
        float damping = damping_ * damping_gain * velocity_rad_s;

        torque = spring - damping;
    }

    if (std::abs(velocity_rad_s) < 0.3f && torque != 0.0f)
    {
        float sign = (torque > 0.0f) ? 1.0f : -1.0f;
        torque -= 0.02f * sign;
    }

    return std::clamp(torque, -torque_clamp_, torque_clamp_);
}
