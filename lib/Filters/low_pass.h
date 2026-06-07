#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#include <cstdint>
#include <cmath>

class LowPassFilter
{
public:
    LowPassFilter(float cutoffFreq, float sampleRate)
        : cutoffFreq_(cutoffFreq),
          sampleRate_(sampleRate)
    {
        updateCoefficients();
    }

    float update(float input)
    {
        output_ = alpha_ * input + (1.0f - alpha_) * output_;
        return output_;
    }

    void updateCutoffFrequency(float cutoffFreq)
    {
        cutoffFreq_ = cutoffFreq;
        updateCoefficients();
    }

    void reset(float initial = 0.0f)
    {
        output_ = initial;
    }

private:
    float cutoffFreq_, sampleRate_;
    float alpha_ = 0.0f;
    float output_ = 0.0f;

    void updateCoefficients()
    {
        float rc = 1.0f / (2.0f * 3.14159265f * cutoffFreq_);
        float dt = 1.0f / sampleRate_;
        alpha_ = dt / (rc + dt);
    }
};

#endif // LOW_PASS_FILTER_H