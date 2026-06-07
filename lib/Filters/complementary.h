#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include <cstdint>
#include <cmath>

class ComplementaryFilter
{
public:
    ComplementaryFilter(float alpha = 0.98f)
        : alpha_(alpha) {}

    float update(float gyroRate, float accelMeasurement, float dt)
    {
        angle_ += gyroRate * dt;

        angle_ = alpha_ * angle_ + (1.0f - alpha_) * accelMeasurement;

        return angle_;
    }

    void reset(float initial = 0.0f)
    {
        angle_ = initial;
    }

private:
    float angle_ = 0.0f;
    float alpha_;
};

#endif // COMPLEMENTARY_FILTER_H