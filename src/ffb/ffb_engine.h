#ifndef FFB_ENGINE_H
#define FFB_ENGINE_H

#include <cstdint>
#include <array>

#include <ffb_protocol.h>

#include <utils.h>

class FFBEngine
{
public:
    FFBEngine() = default;

    float computeTorqueNorm(
        const std::array<Effect, MAX_EFFECTS> &effects,
        uint32_t usedMask,
        uint8_t globalGain,
        float positionRad,
        float velocityRadS,
        float accelerationRadS2,
        uint32_t nowMs);

private:
    int32_t conditionForce(const EffectCondition &c, float inputNorm);
    int32_t damper(const EffectCondition &c, float velocity);
    int32_t spring(const EffectCondition &c, float position);
    int32_t inertia(const EffectCondition &c, float velocity, float acceleration);
    int32_t friction(const EffectCondition &c, float velocity);

    int32_t applyEnvelope(const Effect &e, uint32_t elapsed, int32_t magnitude);
    int32_t constantForce(const Effect &e, uint32_t elapsed);
    int32_t rampForce(const Effect &e, uint32_t elapsed);
    int32_t periodicForce(const Effect &e, uint32_t elapsed);

    static constexpr float kInvVelMax = 1.0f / MAX_VELOCITY_RAD_S;
    static constexpr float kInvAccMax = 1.0f / MAX_ACCELERATION_RAD_S2;

    static constexpr float kInvPosMax = 1.0f / (360.0f * (PI / 180.0f) / 2.0f);

    static constexpr float kFrictionTransitionInv = 1.0f / 0.05f;
};

#endif // FFB_ENGINE_H