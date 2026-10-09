#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include <cstdint>
#include <cmath>

class ComplementaryFilter
{
public:
    /**
     * @brief Constructs a complementary filter that fuses a gyro-integrated
     * angle with an absolute (e.g. accelerometer) angle measurement.
     *
     * @param alpha Weight given to the gyro-integrated estimate (0-1).
     * Higher values trust the gyro more and the absolute measurement less.
     */
    ComplementaryFilter(float alpha = 0.98f)
        : alpha_(alpha) {}

    /**
     * @brief Fuses one gyro rate sample and one absolute angle measurement.
     *
     * @param gyroRate Angular rate from the gyro, in the same angle units
     * per second as accelMeasurement.
     * @param accelMeasurement Absolute angle estimate (e.g. from an
     * accelerometer), in the same units as the returned angle.
     * @param dt Time elapsed since the last update, in seconds.
     * @return Fused angle estimate.
     */
    float update(float gyroRate, float accelMeasurement, float dt)
    {
        angle_ += gyroRate * dt;

        angle_ = alpha_ * angle_ + (1.0f - alpha_) * accelMeasurement;

        return angle_;
    }

    /**
     * @brief Resets the fused angle estimate to a known value.
     *
     * @param initial Angle to reset to.
     */
    void reset(float initial = 0.0f)
    {
        angle_ = initial;
    }

private:
    float angle_ = 0.0f;
    float alpha_;
};

#endif // COMPLEMENTARY_FILTER_H
