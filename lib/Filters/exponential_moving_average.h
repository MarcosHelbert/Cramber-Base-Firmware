#ifndef EXPONENTIAL_MOVING_AVERAGE_H
#define EXPONENTIAL_MOVING_AVERAGE_H

#include <cstdint>
#include <cmath>

class ExponentialMovingAverage
{
public:
    /**
     * @brief Constructs an exponential moving average filter.
     *
     * @param smoothingFactor Weight given to each new sample (0-1). Higher
     * values track new samples faster but smooth less.
     */
    ExponentialMovingAverage(float smoothingFactor = 0.1f)
        : alpha_(smoothingFactor),
          value_(0.0f),
          initialized_(false) {}

    /**
     * @brief Filters one new sample. The first call after construction or
     * reset() snaps the filter directly to that sample instead of blending
     * from zero, avoiding startup transients.
     *
     * @param newValue New raw sample.
     * @return Smoothed output value.
     */
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

    /**
     * @brief Resets the filter, so the next update() snaps to that sample
     * instead of blending.
     *
     * @param initial Value to seed the filter with (overwritten by the next
     * update() call regardless of this parameter, since initialized_ is
     * cleared).
     */
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
