#ifndef ANTICOGGING_H
#define ANTICOGGING_H

#include <cstdint>

#include <config.h>

class Anticogging
{
public:
    Anticogging();

    void setSample(uint32_t index, float value);

    float getCompensation(float mechanicalPositionRad) const;

    const float *data() const { return samples_; }
    uint32_t size() const { return size_; }

private:
    static constexpr uint32_t size_ = COGGING_COMPENSATION_SAMPLING_POINTS;
    float samples_[COGGING_COMPENSATION_SAMPLING_POINTS];
};

#endif // ANTICOGGING_H