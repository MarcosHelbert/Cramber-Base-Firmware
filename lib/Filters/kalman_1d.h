#ifndef KALMAN_1D_FILTER_H
#define KALMAN_1D_FILTER_H

#include <cstdint>
#include <cmath>

class KalmanFilter1D
{
public:
    KalmanFilter1D(float processNoise, float measurementNoise)
        : q_(processNoise),
          r_(measurementNoise)
    {
    }

    float update(float measurement)
    {
        p_ += q_;

        const float k = p_ / (p_ + r_);
        x_ += k * (measurement - x_);
        p_ *= (1.0f - k);

        return x_;
    }

    void reset(float initial = 0.0f)
    {
        x_ = initial;
        p_ = 1.0f;
    }

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