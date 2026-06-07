#include "anticogging.h"

#include <utils.h>

Anticogging::Anticogging()
{
    for (uint32_t i = 0; i < size_; i++)
        samples_[i] = 0.0f;
}

void Anticogging::setSample(uint32_t index, float value)
{
    if (index >= size_)
        return;

    samples_[index] = value;
}

float Anticogging::getCompensation(float mechanicalPositionRad) const
{
    float a = normalizeAngle2Pi(mechanicalPositionRad);

    float pos = (a / TWO_PI) * static_cast<float>(size_);
    float ipos = fastFloor(pos);
    uint32_t i0 = static_cast<uint32_t>(ipos) % size_;
    uint32_t i1 = (i0 + 1) % size_;
    float frac = pos - ipos;

    float v0 = samples_[i0];
    float v1 = samples_[i1];

    return (v0 + (v1 - v0) * frac);
}
