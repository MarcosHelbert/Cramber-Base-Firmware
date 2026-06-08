#include "ffb_manager.h"

#include <algorithm>
#include <cstring>
#include <libopencm3/cm3/cortex.h>
#include <config.h>

#include <debug.h>

#include <hardware/timer.h>

template <typename T>
bool decodeReport(const uint8_t *buffer, uint16_t size, T *out)
{
    if (size < sizeof(T))
        return false;

    memset(out, 0, sizeof(T));
    memcpy(out, buffer, sizeof(T));

    return true;
}

void FFBManager::recompute_(Effect &e)
{
    e.gainNorm = static_cast<float>(e.gain) / 255.0f;

    e.invDuration = (e.duration != 0 && e.duration != 0xFFFFu) ? (1.0f / static_cast<float>(e.duration)) : 0.0f;

    e.invAttackTime = (e.attackTime > 0u) ? (1.0f / static_cast<float>(e.attackTime)) : 0.0f;
    e.invFadeTime = (e.fadeTime > 0u) ? (1.0f / static_cast<float>(e.fadeTime)) : 0.0f;

    e.invPeriod = (e.period > 0u) ? (TWO_PI / static_cast<float>(e.period)) : 0.0f;
    e.phaseRad = static_cast<float>(e.phase) * (TWO_PI / 36000.0f);

    constexpr float inv32767 = 1.0f / 32767.0f;
    EffectCondition &c = e.condition;

    c.posCoeffNorm = static_cast<float>(c.positiveCoefficient) * inv32767;
    c.negCoeffNorm = static_cast<float>(c.negativeCoefficient) * inv32767;
    c.posSatNorm = static_cast<float>(c.positiveSaturation) * inv32767;
    c.negSatNorm = static_cast<float>(c.negativeSaturation) * inv32767;
    c.offsetNorm = static_cast<float>(c.cpOffset) * inv32767;
    c.deadBandNorm = static_cast<float>(c.deadBand) * inv32767;
}

FFBManager::FFBManager()
{
    blockLoad_.effectBlockIndex = 1;
    blockLoad_.ramPoolAvailable = effects_.size();
    blockLoad_.loadStatus = 1;

    pool_.ramPoolSize = effects_.size();
    pool_.maxSimultaneousEffects = effects_.size();
    pool_.memoryManagement = 1;
}

void FFBManager::resetFFB()
{
    cm_disable_interrupts();
    for (auto &e : effects_)
        e = Effect{};

    usedEffects_ = 0;
    activeMask_ = 0;
    ffbActive_ = false;
    devicePaused_ = false;
    gain_ = 255;

    cm_enable_interrupts();
}

void FFBManager::hidSet(uint8_t report_id, uint8_t const *buffer, uint16_t bufsize)
{
    if (!buffer || bufsize == 0)
        return;

    cdcLog("report_id=%u bufsize=%u\n", report_id, bufsize);

    for (uint16_t i = 0; i < bufsize; i++)
    {
        cdcLog("%02X ", buffer[i]);
    }

    cdcLog("\n");

    uint8_t id = report_id - FFB_ID_OFFSET;

    switch (id)
    {
    case 0x01: // Set Effect
    {
        SetEffect_Data d{};
        if (decodeReport(buffer, bufsize, &d))
            setEffect_(&d);
        break;
    }

    case 0x02: // Set Envelope
    {
        SetEnvelope_Data d{};
        if (decodeReport(buffer, bufsize, &d))
            setEnvelope_(&d);
        break;
    }

    case 0x03: // Set Condition
    {
        SetCondition_Data d{};
        if (decodeReport(buffer, bufsize, &d))
            setCondition_(&d);
        break;
    }

    case 0x04: // Set Periodic
    {
        SetPeriodic_Data d{};
        if (decodeReport(buffer, bufsize, &d))
            setPeriodic_(&d);
        break;
    }

    case 0x05: // Set Constant Force
    {
        SetConstantForce_Data d{};
        if (decodeReport(buffer, bufsize, &d))
            setConstantForce_(&d);
        break;
    }

    case 0x06: // Set Ramp
    {
        SetRamp_Data d{};
        if (decodeReport(buffer, bufsize, &d))
            setRamp_(&d);
        break;
    }

    case 0x0A: // Effect Operation
    {
        EffectOperation_Data d{};
        if (decodeReport(buffer, bufsize, &d))
            effectOperation_(&d);
        break;
    }

    case 0x0B: // Block Free
    {
        uint8_t i = buffer[0];
        if (i == 0)
            return;
        blockFree_(i - 1);
        break;
    }

    case 0x0C: // Device Control
    {
        uint8_t c = buffer[0];
        deviceControl_(c);
        break;
    }

    case 0x0D: // Set Global Gain
    {
        uint8_t g = buffer[0];
        deviceGain_(g);
        break;
    }

    case 0x11: // Create New Effect
    {
        NewEffect_Data ne{};
        if (decodeReport(buffer, bufsize, &ne))
            createNewEffect_(&ne);
        break;
    }

    default:
        break;
    }
}

uint16_t FFBManager::hidGet(uint8_t report_id, uint8_t *buffer)
{
    const uint8_t id = report_id - FFB_ID_OFFSET;

    if (id == 0x12)
    {
        memcpy(buffer, &blockLoad_, sizeof(blockLoad_));
        return sizeof(blockLoad_);
    }
    else if (id == 0x13)
    {
        memcpy(buffer, &pool_, sizeof(pool_));
        return sizeof(pool_);
    }

    return 0;
}

float FFBManager::getTorque(float positionRad, float velocityRadS, float accelerationRadS2, uint32_t nowMs)
{
    if (!ffbActive_ || devicePaused_)
        return 0.0f;

    float torque = engine_.computeTorqueNorm(
        effects_, activeMask_, gain_,
        positionRad, velocityRadS, accelerationRadS2,
        nowMs);
    float iq = torque * MAX_CURRENT;
    return std::clamp(iq, -MAX_CURRENT, MAX_CURRENT);
}

void FFBManager::sendStatusReport_()
{
    PIDStatusReport_Data r{};

    r.status = 0x08;

    if (ffbActive_ && !devicePaused_)
    {
        r.status |= 0x02;
        r.status |= 0x10;
    }
    else
    {
        r.status |= 0x01;
    }

    tud_hid_n_report(0, 0x02 + FFB_ID_OFFSET, &r, sizeof(r));
}

void FFBManager::setActive_(uint8_t idx, bool active)
{
    uint32_t bit = 1u << idx;

    if (active)
        activeMask_ |= bit;
    else
        activeMask_ &= ~bit;

    sendStatusReport_();
}

void FFBManager::setEffect_(SetEffect_Data *d)
{
    if (!isValidIndex_(d->effectBlockIndex))
        return;

    cdcLog(
        "\n=== SetEffect ===\n"
        "effectBlockIndex: %u\n"
        "effectType: %u\n"
        "duration: %u\n"
        "triggerRepeatInterval: %u\n"
        "samplePeriod: %u\n"
        "startDelay: %u\n"
        "gain: %u\n"
        "triggerButton: %u\n"
        "enableAxis: %u\n"
        "direction: %u\n",
        d->effectBlockIndex,
        d->effectType,
        d->duration,
        d->triggerRepeatInterval,
        d->samplePeriod,
        d->startDelay,
        d->gain,
        d->triggerButton,
        d->enableAxis,
        d->direction);

    Effect &e = effects_[d->effectBlockIndex - 1];

    cdcLog(
        "Effect slot=%u allocated=%u\n",
        d->effectBlockIndex - 1,
        e.allocated);

    if (!e.allocated)
        return;

    e.gain = d->gain;
    e.type = d->effectType;
    e.samplePeriod = d->samplePeriod;
    e.duration = (d->duration == 0u) ? 0xFFFFu : d->duration;
    e.startDelay = d->startDelay;
    e.enableAxis = d->enableAxis;
    e.direction = d->direction;

    recompute_(e);

    if (!ffbActive_)
        startFFB();

    sendStatusReport_();
}

void FFBManager::setEnvelope_(SetEnvelope_Data *d)
{
    if (!isValidIndex_(d->effectBlockIndex))
        return;

    Effect &e = effects_[d->effectBlockIndex - 1];
    if (!e.allocated)
        return;

    e.attackLevel = d->attackLevel;
    e.attackTime = d->attackTime;
    e.fadeLevel = d->fadeLevel;
    e.fadeTime = d->fadeTime;
    e.useEnvelope = true;

    recompute_(e);
}

void FFBManager::setCondition_(SetCondition_Data *d)
{
    if (!isValidIndex_(d->effectBlockIndex))
        return;

    Effect &e = effects_[d->effectBlockIndex - 1];
    if (!e.allocated)
        return;

    uint8_t axisIdx = d->parameterBlockOffset & 0x3F;
    (void)axisIdx;

    e.condition.cpOffset = d->cpOffset;
    e.condition.positiveCoefficient = d->positiveCoefficient;
    e.condition.negativeCoefficient = d->negativeCoefficient;
    e.condition.positiveSaturation = d->positiveSaturation;
    e.condition.negativeSaturation = d->negativeSaturation;
    e.condition.deadBand = d->deadBand;

    recompute_(e);
}

void FFBManager::setPeriodic_(SetPeriodic_Data *d)
{
    if (!isValidIndex_(d->effectBlockIndex))
        return;

    Effect &e = effects_[d->effectBlockIndex - 1];
    if (!e.allocated)
        return;

    e.magnitude = std::clamp<int16_t>(d->magnitude, -32767, 32767);
    e.offset = std::clamp<int16_t>(d->offset, -32767, 32767);
    e.phase = d->phase;
    e.period = std::clamp<uint32_t>(d->period, 1u, 0xFFFFu);

    recompute_(e);
}

void FFBManager::setConstantForce_(SetConstantForce_Data *d)
{
    if (!isValidIndex_(d->effectBlockIndex))
        return;

    Effect &e = effects_[d->effectBlockIndex - 1];

    if (!e.allocated)
        return;

    e.magnitude = std::clamp<int16_t>(d->magnitude, -32767, 32767);

    recompute_(e);

    sendStatusReport_();
}

void FFBManager::setRamp_(SetRamp_Data *d)
{
    if (!isValidIndex_(d->effectBlockIndex))
        return;

    Effect &e = effects_[d->effectBlockIndex - 1];
    if (!e.allocated)
        return;

    e.startLevel = std::clamp<int16_t>(d->startLevel, -32767, 32767);
    e.endLevel = std::clamp<int16_t>(d->endLevel, -32767, 32767);

    recompute_(e);
}

void FFBManager::effectOperation_(EffectOperation_Data *d)
{
    if (!isValidIndex_(d->effectBlockIndex))
        return;

    Effect &e = effects_[d->effectBlockIndex - 1];
    if (!e.allocated)
        return;

    const uint32_t now = millis();
    const uint8_t eidx = d->effectBlockIndex - 1;
    const uint32_t eidxBit = 1u << eidx;

    switch (d->state)
    {
    case 1: // Start
    case 2: // Start and Stop All Others
    {
        if (d->state == 2)
        {
            for (uint32_t i = 0; i < effects_.size(); i++)
            {
                if (i != eidx && effects_[i].active)
                {
                    effects_[i].active = false;
                    setActive_(i, false);
                }
            }
        }

        e.startTime = now + e.startDelay;
        e.active = true;
        setActive_(eidx, true);

        ffbActive_ = true;
        devicePaused_ = false;
        break;
    }

    case 3: // Stop
        e.active = false;
        setActive_(eidx, false);
        break;
    };

    sendStatusReport_();
}

void FFBManager::blockFree_(uint8_t idx)
{
    if (idx >= effects_.size())
        return;

    if (!effects_[idx].allocated)
        return;

    effects_[idx] = Effect{};

    if (usedEffects_ > 0)
        usedEffects_--;

    setActive_(idx, false);
}

void FFBManager::deviceControl_(uint8_t command)
{
    if (command & 0x01)
    {
        ffbActive_ = true;
        devicePaused_ = false;
    }

    if (command & 0x02)
    {
        ffbActive_ = false;
    }

    if (command & 0x04)
    {
        for (uint32_t i = 0; i < effects_.size(); i++)
        {
            if (effects_[i].active)
            {
                effects_[i].active = false;
                activeMask_ &= ~(1u << i);
            }
        }
    }

    if (command & 0x08)
    {
        resetFFB();
    }

    if (command & 0x10)
    {
        devicePaused_ = true;
    }

    if (command & 0x20)
    {
        devicePaused_ = false;
    }

    sendStatusReport_();
}

void FFBManager::deviceGain_(uint8_t gain)
{
    gain_ = gain;
}

void FFBManager::createNewEffect_(NewEffect_Data *d)
{
    int32_t idx = findFreeEffectIndex_(d->effectType);
    if (idx < 0)
    {
        updateBlockLoad_(0, 0x02);
        cdcLog("Can't allocate a new effect");
        return;
    }

    Effect &e = effects_[idx];
    e = Effect{};
    e.allocated = true;
    e.type = d->effectType;

    cdcLog(
        "\n=== CreateNewEffect ===\n"
        "Index: %d\n"
        "effectType: %u\n"
        "byteCount: %u\n",
        idx,
        d->effectType,
        d->byteCount);

    recompute_(e);

    usedEffects_ = std::min<uint32_t>(
        usedEffects_ + 1,
        effects_.size());

    updateBlockLoad_(static_cast<uint8_t>(idx + 1), 0x01);
    sendStatusReport_();
}

int32_t FFBManager::findFreeEffectIndex_(uint8_t type) const
{
    if (type == 0 || type > 0x0B)
        return -1;

    for (uint16_t i = 0; i < effects_.size(); i++)
    {
        if (!effects_[i].allocated)
            return static_cast<int32_t>(i);
    }

    return -1;
}

void FFBManager::updateBlockLoad_(uint8_t index, uint8_t status)
{
    blockLoad_.effectBlockIndex = index;
    blockLoad_.ramPoolAvailable = effects_.size() - usedEffects_;
    blockLoad_.loadStatus = status;
}