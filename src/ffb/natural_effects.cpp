#include "natural_effects.h"

#include <algorithm>
#include <cmath>

#include <persistence/config_store.h>

namespace
{
    constexpr float kMaxDampingA = 0.30f;
    constexpr float kMaxFrictionA = 0.80f;
    constexpr float kMaxInertiaA = 0.02f;

    constexpr float kFrictionVelocityRadS = 0.5f;
}

float NaturalEffects::calculate(float velocity_rad_s, float acceleration_rad_s2) const
{
    const float damping = std::clamp(g_config.natural_damping, 0.0f, 1.0f) * kMaxDampingA;
    const float friction = std::clamp(g_config.natural_friction, 0.0f, 1.0f) * kMaxFrictionA;
    const float inertia = std::clamp(g_config.natural_inertia, 0.0f, 1.0f) * kMaxInertiaA;

    float torque = -damping * velocity_rad_s;
    torque -= friction * std::tanh(velocity_rad_s / kFrictionVelocityRadS);
    torque -= inertia * acceleration_rad_s2;
    return torque;
}
