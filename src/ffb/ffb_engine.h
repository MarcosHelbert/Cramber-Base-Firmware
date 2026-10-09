#ifndef FFB_ENGINE_H
#define FFB_ENGINE_H

#include <cstdint>
#include <array>

#include <ffb_protocol.h>

#include <utils.h>

#include <persistence/config_store.h>

class FFBEngine
{
public:
    FFBEngine(const PersistedConfig &config) : config_(config) {};

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

    static float safeInv_(float value) { return (value > 0.0f) ? (1.0f / value) : 0.0f; }

    float invVelMax_() const { return safeInv_(config_.max_velocity_rad_s); }
    float invAccMax_() const { return safeInv_(config_.max_acceleration_rad_s2); }
    float invPosMax_() const { return safeInv_(angleToRadians(config_.steering_range_deg) / 2.0f); }

private:
    const PersistedConfig &config_;

    static constexpr float kFrictionTransitionInv = 1.0f / 0.05f;
};

#endif // FFB_ENGINE_H