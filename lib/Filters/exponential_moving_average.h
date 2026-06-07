#ifndef EXPONENTIAL_MOVING_AVERAGE_H
#define EXPONENTIAL_MOVING_AVERAGE_H

#include <cstdint>
#include <cmath>

class ExponentialMovingAverage
{
public:
    ExponentialMovingAverage(float smoothingFactor = 0.1f)
        : alpha_(smoothingFactor),
          value_(0.0f),
          initialized_(false) {}

    float update(float newValue)
    {
        if (!initialized_)
        {
            value_ = newValue;
            initialized_ = true;
            return value_;
        }

        value_ = alpha_ * newValue + (1.0f - alpha_) * value_;
        return value_;
    }

    void reset(float initial = 0.0f)
    {
        value_ = initial;
        initialized_ = false;
    }

private:
    float alpha_;
    float value_;
    bool initialized_;
};

#endif // EXPONENTIAL_MOVING_AVERAGE_H