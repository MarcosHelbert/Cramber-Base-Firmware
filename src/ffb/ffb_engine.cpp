#include "ffb_engine.h"

int32_t FFBEngine::conditionForce(const EffectCondition &c, float inputNorm)
{
    if (!c.isActive())
        return 0;

    float pos = smoothDeadband(inputNorm - c.offsetNorm, c.deadBandNorm);

    const bool positive = (pos > 0.0f);
    const float coeff = positive ? c.posCoeffNorm : c.negCoeffNorm;
    const float sat = positive ? c.posSatNorm : c.negSatNorm;

    float force = coeff * pos * 32767.0f;

    if (sat > 0.0f)
        force = clamp(force, -sat * 32767.0f, sat * 32767.0f);

    return (int32_t)force;
}

int32_t FFBEngine::damper(const EffectCondition &c, float velocity)
{
    if (!c.isActive())
        return 0;

    float v = velocity * kInvVelMax;

    const bool forward = (v >= 0.0f);
    const float coeff = forward ? c.posCoeffNorm : c.negCoeffNorm;
    const float sat = forward ? c.posSatNorm : c.negSatNorm;

    float force = -coeff * v * 32767.0f;
    return (int32_t)clamp(force, -sat * 32767.0f, sat * 32767.0f);
}

int32_t FFBEngine::spring(const EffectCondition &c, float position)
{
    if (!c.isActive())
        return 0;

    float pos = smoothDeadband(position * kInvPosMax, c.deadBandNorm);

    const bool positive = (pos >= 0.0f);
    const float coeff = positive ? c.posCoeffNorm : c.negCoeffNorm;
    const float sat = positive ? c.posSatNorm : c.negSatNorm;

    float force = -coeff * pos * 32767.0f;
    return (int32_t)clamp(force, -sat * 32767.0f, sat * 32767.0f);
}

int32_t FFBEngine::inertia(const EffectCondition &c, float velocity, float acceleration)
{
    if (!c.isActive())
        return 0;

    float a = acceleration * kInvAccMax;
    float v = velocity * kInvVelMax;

    float force = (-c.posCoeffNorm * a - c.negCoeffNorm * v) * 32767.0f;
    const float sat = c.posSatNorm * 32767.0f;
    return (int32_t)clamp(force, -sat, sat);
}

int32_t FFBEngine::friction(const EffectCondition &c, float velocity)
{
    if (!c.isActive())
        return 0;

    float v = velocity * kInvVelMax;

    static constexpr float kVStiction = 0.12f;

    float s = FastMath::fast_tanh(v / kVStiction);
    float viscous = -c.posCoeffNorm * v;
    float stribeck = -c.negCoeffNorm * s;
    float force = (viscous + stribeck) * 32767.0f;
    const float sat = c.posSatNorm * 32767.0f;
    return (int32_t)clamp(force, -sat, sat);
}

int32_t FFBEngine::applyEnvelope(const Effect &e, uint32_t elapsed, int32_t magnitude)
{
    if (!e.useEnvelope)
        return magnitude;

    float mag = (float)magnitude;

    if (e.attackTime > 0 && elapsed < e.attackTime)
    {
        float t = (float)elapsed * e.invAttackTime;
        return (int32_t)(e.attackLevel + (mag - e.attackLevel) * t);
    }

    if (e.duration != 0xFFFFu && e.fadeTime > 0)
    {
        uint32_t fadeStart = (e.duration > e.fadeTime) ? e.duration - e.fadeTime : 0;

        if (elapsed >= fadeStart)
        {
            float t = (float)(elapsed - fadeStart) * e.invFadeTime;
            t = clamp(t, 0.0f, 1.0f);

            return (int32_t)(mag + (e.fadeLevel - mag) * t);
        }
    }

    return magnitude;
}

int32_t FFBEngine::constantForce(const Effect &e, uint32_t elapsed)
{
    return applyEnvelope(e, elapsed, -e.magnitude);
}

int32_t FFBEngine::rampForce(const Effect &e, uint32_t elapsed)
{
    if (e.invDuration == 0.0f)
        return e.startLevel;

    const float t = clamp((float)elapsed * e.invDuration, 0.0f, 1.0f);
    return (int32_t)((float)e.startLevel + t * (float)(e.endLevel - e.startLevel));
}

int32_t FFBEngine::periodicForce(const Effect &e, uint32_t elapsed)
{
    if (e.period == 0)
        return 0;

    const int32_t env = applyEnvelope(e, elapsed, e.magnitude);
    const float amp = (float)env;

    float angle = normalizeAngle2Pi((float)elapsed * e.invPeriod * TWO_PI + e.phaseRad);

    float wave = 0.0f;

    switch (e.type)
    {
    case EFFECT_SINE:
        wave = FastMath::fast_sin(angle);
        break;

    case EFFECT_SQUARE:
    {
        wave = (angle < PI) ? 1.0f : -1.0f;
        break;
    }

    case EFFECT_TRIANGLE:
    {
        if (angle < PI)
            wave = -1.0f + (2.0f / PI) * angle;
        else
            wave = 3.0f - (2.0f / PI) * angle;
        break;
    }

    case EFFECT_SAW_UP:
    {
        wave = -1.0f + (2.0f / TWO_PI) * angle;
        break;
    }

    case EFFECT_SAW_DOWN:
    {
        wave = 1.0f - (2.0f / TWO_PI) * angle;
        break;
    }

    default:
        return 0;
    }

    return (int32_t)((float)e.offset + amp * wave);
}

float FFBEngine::computeTorqueNorm(
    const std::array<Effect, MAX_EFFECTS> &effects,
    uint32_t usedMask,
    uint8_t globalGain,
    float positionRad,
    float velocityRadS,
    float accelerationRadS2,
    uint32_t nowMs)
{
    if (!usedMask)
        return 0.0f;

    const float globalGainF = (float)globalGain / 255.0f;

    float torque = 0.0f;
    float comp = 0.0f;

    uint32_t mask = usedMask;

    while (mask)
    {
        const uint32_t index = (uint32_t)__builtin_ctz(mask);
        mask &= mask - 1u;

        const Effect &e = effects[index];
        if (!e.allocated || !e.active)
            continue;

        const int32_t elapsedSigned = (int32_t)(nowMs - e.startTime);
        if (elapsedSigned < 0)
            continue;

        const uint32_t elapsed = (uint32_t)elapsedSigned;

        if (e.duration != 0xFFFFu && elapsed >= (uint32_t)e.duration)
            continue;

        float result = 0.0f;

        switch (e.type)
        {
        case EFFECT_DAMPER:
            result = (float)damper(e.condition, velocityRadS);
            break;

        case EFFECT_FRICTION:
            result = (float)friction(e.condition, velocityRadS);
            break;

        case EFFECT_INERTIA:
            result = (float)inertia(e.condition, velocityRadS, accelerationRadS2);
            break;

        case EFFECT_SPRING:
            result = (float)spring(e.condition, positionRad);
            break;

        case EFFECT_CONSTANT:
            result = (float)constantForce(e, elapsed);
            break;

        case EFFECT_RAMP:
            result = (float)rampForce(e, elapsed);
            break;

        case EFFECT_SINE:
        case EFFECT_SQUARE:
        case EFFECT_TRIANGLE:
        case EFFECT_SAW_UP:
        case EFFECT_SAW_DOWN:
            result = (float)periodicForce(e, elapsed);
            break;

        default:
            break;
        }

        torque += result * e.gainNorm * globalGainF;
    };

    torque = clamp(torque, -32767.0f, 32767.0f);
    return torque * (1 / 32767.0f);
}