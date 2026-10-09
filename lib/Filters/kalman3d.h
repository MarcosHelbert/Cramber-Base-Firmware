#ifndef KALMAN_3D_FILTER_H
#define KALMAN_3D_FILTER_H

#include <cstdint>
#include <cmath>

class KalmanFilter3D
{
public:
    /**
     * @brief Constructs a 3-state (position, velocity, acceleration)
     * Kalman filter with a constant-acceleration motion model.
     *
     * @param dt Fixed time step between predict() calls, in seconds.
     * @param processNoise0 Process noise covariance for the position state.
     * @param processNoise1 Process noise covariance for the velocity state.
     * @param processNoise2 Process noise covariance for the acceleration
     * state.
     * @param measurementNoise Measurement noise covariance (R) for the
     * position measurement.
     */
    KalmanFilter3D(float dt, float processNoise0, float processNoise1, float processNoise2, float measurementNoise)
        : dt_(dt),
          q0_(processNoise0),
          q1_(processNoise1),
          q2_(processNoise2),
          r_(measurementNoise)
    {
    }

    /**
     * @brief Advances the state estimate and error covariance by one time
     * step using the constant-acceleration motion model, without
     * incorporating a new measurement.
     */
    void predict()
    {
        x0_ += x1_ * dt_ + 0.5f * x2_ * dt_ * dt_;
        x1_ += x2_ * dt_;

        const float dt2 = dt_ * dt_;

        p00_ += dt_ * (p10_ + p01_) + 0.5f * dt2 * (p20_ + p02_) + dt2 * p11_ + q0_;

        p01_ += dt_ * p11_ + 0.5f * dt2 * p21_;

        p02_ += dt_ * p12_ + 0.5f * dt2 * p22_;

        p10_ += dt_ * p11_ + 0.5f * dt2 * p12_;

        p11_ += dt_ * (p21_ + p12_) + q1_;

        p12_ += dt_ * p22_;

        p20_ += dt_ * p21_;
        p21_ += dt_ * p22_;
        p22_ += q2_;
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
        const float k2 = p20_ / s;

        x0_ += k0 * y;
        x1_ += k1 * y;
        x2_ += k2 * y;

        const float p00 = p00_;
        const float p01 = p01_;
        const float p02 = p02_;

        p00_ -= k0 * p00;
        p01_ -= k0 * p01;
        p02_ -= k0 * p02;

        p10_ -= k1 * p00;
        p11_ -= k1 * p01;
        p12_ -= k1 * p02;

        p20_ -= k2 * p00;
        p21_ -= k2 * p01;
        p22_ -= k2 * p02;
    }

    /**
     * @brief Resets the state estimate and error covariance.
     *
     * @param state0 Position value to reset to.
     * @param state1 Velocity value to reset to.
     * @param state2 Acceleration value to reset to.
     */
    void reset(float state0 = 0.0f, float state1 = 0.0f, float state2 = 0.0f)
    {
        x0_ = state0;
        x1_ = state1;
        x2_ = state2;

        p00_ = 1.0f;
        p11_ = 1.0f;
        p22_ = 1.0f;

        p01_ = p02_ = 0.0f;
        p10_ = p12_ = 0.0f;
        p20_ = p21_ = 0.0f;
    }

    /**
     * @brief Returns the current position estimate.
     */
    float state0() const { return x0_; }

    /**
     * @brief Returns the current velocity estimate.
     */
    float state1() const { return x1_; }

    /**
     * @brief Returns the current acceleration estimate.
     */
    float state2() const { return x2_; }

private:
    float dt_;

    float q0_;
    float q1_;
    float q2_;
    float r_;

    float x0_ = 0.0f;
    float x1_ = 0.0f;
    float x2_ = 0.0f;

    float p00_ = 1.0f;
    float p01_ = 0.0f;
    float p02_ = 0.0f;

    float p10_ = 0.0f;
    float p11_ = 1.0f;
    float p12_ = 0.0f;

    float p20_ = 0.0f;
    float p21_ = 0.0f;
    float p22_ = 1.0f;
};

#endif // KALMAN_3D_FILTER_H
