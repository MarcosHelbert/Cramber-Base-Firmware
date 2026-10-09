#ifndef FFB_PROTOCOL_H
#define FFB_PROTOCOL_H

#include <cstdint>

#define FFB_ID_OFFSET 0x00
#define MAX_EFFECTS 10

enum EffectType : uint8_t
{
    EFFECT_NONE = 0x00,
    EFFECT_CONSTANT = 0x01,
    EFFECT_RAMP = 0x02,
    EFFECT_SQUARE = 0x03,
    EFFECT_SINE = 0x04,
    EFFECT_TRIANGLE = 0x05,
    EFFECT_SAW_UP = 0x06,
    EFFECT_SAW_DOWN = 0x07,
    EFFECT_SPRING = 0x08,
    EFFECT_DAMPER = 0x09,
    EFFECT_INERTIA = 0x0A,
    EFFECT_FRICTION = 0x0B,
};

struct __attribute__((packed)) SetEffect_Data
{
    uint8_t effectBlockIndex;       // 1..MAX_EFFECTS
    uint8_t effectType;             // FFB_EFFECT_*
    uint16_t duration;              // 0..32767 ms (0 or 0xFFFF = infinite)
    uint16_t triggerRepeatInterval; // 0..32767 ms
    uint16_t samplePeriod;          // 0..32767 ms
    uint16_t startDelay;            // 0..32767 ms
    uint8_t gain;                   // 0..255
    uint8_t triggerButton;          // Unused
    uint8_t enableAxis;             // Bits: 0=Axis Enable, 1=DirectionEnable
    uint16_t direction;             // Angle (0..36000 = 0..360 degrees)
};

struct __attribute__((packed)) SetEnvelope_Data
{
    uint8_t effectBlockIndex; // 1..MAX_EFFECTS
    uint16_t attackLevel;     // Starting level
    uint16_t fadeLevel;       // Ending level
    uint32_t attackTime;      // Attack duration (ms)
    uint32_t fadeTime;        // Fade duration (ms)
};

struct __attribute__((packed)) SetCondition_Data
{
    uint8_t effectBlockIndex;     // 1..MAX_EFFECTS
    uint8_t parameterBlockOffset; // Axis index
    int16_t cpOffset;             // Center point offset
    int16_t positiveCoefficient;  // Coefficient for positive side
    int16_t negativeCoefficient;  // Coefficient for negative side
    uint16_t positiveSaturation;  // Max force for positive side
    uint16_t negativeSaturation;  // Max force for negative side
    uint16_t deadBand;            // Dead band around center
};

struct __attribute__((packed)) SetPeriodic_Data
{
    uint8_t effectBlockIndex; // 1..MAX_EFFECTS
    int16_t magnitude;        // Amplitude
    int16_t offset;           // DC offset
    uint16_t phase;           // Phase in degrees
    uint32_t period;          // Period in ms
};

struct __attribute__((packed)) SetConstantForce_Data
{
    uint8_t effectBlockIndex; // 1..MAX_EFFECTS
    int16_t magnitude;        // Force magnitude
};

struct __attribute__((packed)) SetRamp_Data
{
    uint8_t effectBlockIndex; // 1..MAX_EFFECTS
    int16_t startLevel;       // Initial magnitude
    int16_t endLevel;         // Final magnitude
};

struct __attribute__((packed)) EffectOperation_Data
{
    uint8_t effectBlockIndex; // 1..MAX_EFFECTS
    uint8_t state;            // 1=Start, 2=Start Solo, 3=Stop
    uint8_t loopCount;
};

struct __attribute__((packed)) NewEffect_Data
{
    uint8_t effectType; // FFB_EFFECT_*
    uint16_t byteCount; // Size of custom effects
};

struct __attribute__((packed)) BlockLoad_Data
{
    uint8_t effectBlockIndex;  // 1..MAX_EFFECTS
    uint8_t loadStatus;        // 1=Success, 2=Full, 3=Error
    uint16_t ramPoolAvailable; // Bytes available
};

struct __attribute__((packed)) PIDPool_Data
{
    uint16_t ramPoolSize;
    uint8_t maxSimultaneousEffects; // 1..MAX_EFFECTS
    uint8_t memoryManagement;       // Bits: 0=DeviceManagedPool, 1=SharedParameterBlocks
};

struct __attribute__((packed)) PIDStatusReport_Data
{
    uint8_t status;
};

struct EffectCondition
{
    int16_t cpOffset = 0;
    int16_t positiveCoefficient = 0;
    int16_t negativeCoefficient = 0;
    uint16_t positiveSaturation = 0;
    uint16_t negativeSaturation = 0;
    uint16_t deadBand = 0;

    // pré-computed values for efficiency
    float posCoeffNorm = 0.0f; // positiveCoefficient / 32767
    float negCoeffNorm = 0.0f; // negativeCoefficient / 32767
    float posSatNorm = 0.0f;   // positiveSaturation  / 32767 (0 = no limit)
    float negSatNorm = 0.0f;   // negativeSaturation  / 32767
    float offsetNorm = 0.0f;   // cpOffset  / 32767
    float deadBandNorm = 0.0f; // deadBand  / 32767

    bool isActive() const
    {
        return positiveCoefficient != 0 || negativeCoefficient != 0;
    }
};

struct Effect
{
    bool allocated = false;
    bool active = false;
    uint8_t type = EffectType::EFFECT_NONE;

    // ------------------------------------------------------------------------
    // Basic Parameters
    // ------------------------------------------------------------------------
    uint8_t gain = 255;    // Effect gain (0-255)
    int16_t magnitude = 0; // Base magnitude (-32767 to 32767)
    int16_t offset = 0;    // DC offset (for periodic effects)
    uint8_t enableAxis = 0;
    uint16_t direction = 0; // degrees * 100

    // ------------------------------------------------------------------------
    // Timing
    // ------------------------------------------------------------------------
    uint32_t duration = 0xFFFF; // Duration in ms
    uint16_t startDelay = 0;    // Delay before start (ms)
    uint32_t startTime = 0;     // Start timestamp (ms)
    uint16_t samplePeriod = 0;  // Update period (ms)

    // ------------------------------------------------------------------------
    // Periodic Effect Parameters
    // ------------------------------------------------------------------------
    uint16_t period = 0; // Wave period (ms)
    uint16_t phase = 0;  // Phase offset (degrees * 100)

    // ------------------------------------------------------------------------
    // Ramp Effect Parameters
    // ------------------------------------------------------------------------
    int16_t startLevel = 0; // Ramp start magnitude
    int16_t endLevel = 0;   // Ramp end magnitude

    // ------------------------------------------------------------------------
    // Envelope Parameters
    // ------------------------------------------------------------------------
    bool useEnvelope = false; // Enable envelope
    uint16_t attackLevel = 0; // Attack start level
    uint16_t fadeLevel = 0;   // Fade end level
    uint32_t attackTime = 0;  // Attack duration (ms)
    uint32_t fadeTime = 0;    // Fade duration (ms)

    // ------------------------------------------------------------------------
    // Condition Effect Parameters (Spring, Damper, Friction, Inertia)
    // ------------------------------------------------------------------------
    EffectCondition condition;

    // ------------------------------------------------------------------------
    // Internal state for envelope processing
    // ------------------------------------------------------------------------
    // periodic
    float invPeriod = 0.0f; // TWO_PI / period
    float phaseRad = 0.0f;  // (phase / 36000) * TWO_PI
    // envelope
    float invAttackTime = 0.0f; // 1.0f / attackTime  (0 if attackTime==0)
    float invFadeTime = 0.0f;   // 1.0f / fadeTime    (0 if fadeTime==0)
    // ramp/geral
    float invDuration = 0.0f; // 1.0f / duration    (0 if infinite)
    float gainNorm = 1.0f;
};

#endif // FFB_PROTOCOL_H