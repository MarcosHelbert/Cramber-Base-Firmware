#ifndef KALMAN_1D_FILTER_H
#define KALMAN_1D_FILTER_H

#include <cstdint>
#include <cmath>

class KalmanFilter1D
{
public:
    /**
     * @brief Constructs a scalar (1-state) Kalman filter.
     *
     * @param processNoise Process noise covariance (Q). Higher values let
     * the estimate follow new measurements faster.
     * @param measurementNoise Measurement noise covariance (R). Higher
     * values make the filter trust measurements less.
     */
    KalmanFilter1D(float processNoise, float measurementNoise)
        : q_(processNoise),
          r_(measurementNoise)
    {
    }

    /**
     * @brief Runs one predict+update cycle with a new measurement.
     *
     * @param measurement Latest raw measurement of the tracked state.
     * @return Updated state estimate.
     */
    float update(float measurement)
    {
        p_ += q_;

        const float k = p_ / (p_ + r_);
        x_ += k * (measurement - x_);
        p_ *= (1.0f - k);

        return x_;
    }

    /**
     * @brief Resets the state estimate and error covariance.
     *
     * @param initial State value to reset to.
     */
    void reset(float initial = 0.0f)
    {
        x_ = initial;
        p_ = 1.0f;
    }

    /**
     * @brief Returns the current state estimate.
     */
    float state() const
    {
        return x_;
    }

private:
    float q_;
    float r_;

    float p_ = 1.0f;
    float x_ = 0.0f;
};

#endif // KALMAN_1D_FILTER_H
