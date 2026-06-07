#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <cstdint>
#include <cmath>

class KalmanFilter1D
{
public:
    KalmanFilter1D(float processNoise, float measurementNoise)
        : q_(processNoise),
          r_(measurementNoise),
          p_(1.0f), x_(0.0f) {}

    float update(float measurement)
    {
        p_ = p_ + q_;

        float k = p_ / (p_ + r_);
        x_ = x_ + k * (measurement - x_);
        p_ = (1.0f - k) * p_;

        return x_;
    }

    void reset(float initial = 0.0f)
    {
        x_ = initial;
        p_ = 1.0f;
    }

private:
    float q_;
    float r_;
    float p_;
    float x_;
};

#endif // KALMAN_FILTER_H