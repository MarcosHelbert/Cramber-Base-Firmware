#include "idle_spring.h"

#include <algorithm>

#include <persistence/config_store.h>

float IdleSpring::calculate(float position_rad, float velocity_rad_s, bool game_driving, float dt)
{
    constexpr float kFadeS = 0.7f;

    const float target = (g_config.idle_spring_enabled && !game_driving) ? 1.0f : 0.0f;
    const float step = dt / kFadeS;
    weight_ += std::clamp(target - weight_, -step, step);

    if (weight_ <= 0.0f)
        return 0.0f;

    float torque = -g_config.idle_spring_stiffness * position_rad - g_config.idle_spring_damping * velocity_rad_s;
    torque = std::clamp(torque, -g_config.max_current, g_config.max_current);
    return weight_ * torque;
}
