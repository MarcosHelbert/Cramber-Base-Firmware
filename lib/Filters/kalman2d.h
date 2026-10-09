#ifndef KALMAN_2D_FILTER_H
#define KALMAN_2D_FILTER_H

#include <cstdint>
#include <cmath>

class KalmanFilter2D
{
public:
    /**
     * @brief Constructs a 2-state (position, velocity) Kalman filter with a
     * constant-velocity motion model.
     *
     * @param dt Fixed time step between predict() calls, in seconds.
     * @param processNoise0 Process noise covariance for the position state.
     * @param processNoise1 Process noise covariance for the velocity state.
     * @param measurementNoise Measurement noise covariance (R) for the
     * position measurement.
     */
    KalmanFilter2D(float dt, float processNoise0, float processNoise1, float measurementNoise)
        : dt_(dt),
          q0_(processNoise0),
          q1_(processNoise1),
          r_(measurementNoise)
    {
    }

    /**
     * @brief Advances the state estimate and error covariance by one time
     * step using the constant-velocity motion model, without incorporating
     * a new measurement.
     */
    void predict()
    {
        x0_ += x1_ * dt_;

        p00_ += dt_ * (p10_ + p01_) + dt_ * dt_ * p11_ + q0_;
        p01_ += dt_ * p11_;
        p10_ += dt_ * p11_;
        p11_ += q1_;
    }

    /**
     * @brief Corrects the predicted state with a new position measurement.
     * Must be called after predict() to complete one filter cycle.
     *
     * @param measurement Latest raw measurement of the position state.
     */
    void update(float measurement)
    {
        const float y = measurement - x0_;

        const float s = p00_ + r_;

        const float k0 = p00_ / s;
        const float k1 = p10_ / s;

        x0_ += k0 * y;
        x1_ += k1 * y;

        const float p00 = p00_;
        const float p01 = p01_;

        p00_ -= k0 * p00;
        p01_ -= k0 * p01;
        p10_ -= k1 * p00;
        p11_ -= k1 * p01;
    }

    /**
     * @brief Resets the state estimate and error covariance.
     *
     * @param state0 Position value to reset to.
     * @param state1 Velocity value to reset to.
     */
    void reset(float state0 = 0.0f, float state1 = 0.0f)
    {
        x0_ = state0;
        x1_ = state1;

        p00_ = 1.0f;
        p01_ = 0.0f;
        p10_ = 0.0f;
        p11_ = 1.0f;
    }

    /**
     * @brief Returns the current position estimate.
     */
    float state0() const { return x0_; }

    /**
     * @brief Returns the current velocity estimate.
     */
    float state1() const { return x1_; }

private:
    float dt_;

    float q0_;
    float q1_;
    float r_;

    float x0_ = 0.0f;
    float x1_ = 0.0f;

    float p00_ = 1.0f;
    float p01_ = 0.0f;
    float p10_ = 0.0f;
    float p11_ = 1.0f;
};

#endif // KALMAN_2D_FILTER_H
