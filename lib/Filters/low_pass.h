#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#include <cstdint>
#include <cmath>

class LowPassFilter
{
public:
    /**
     * @brief Constructs a first-order (RC) low-pass filter.
     *
     * @param cutoffFreq Cutoff frequency, in Hz.
     * @param sampleRate Sampling rate of the input signal, in Hz.
     */
    LowPassFilter(float cutoffFreq, float sampleRate)
        : cutoffFreq_(cutoffFreq),
          sampleRate_(sampleRate)
    {
        updateCoefficients();
    }

    /**
     * @brief Filters one input sample through the low-pass filter.
     *
     * @param input Raw input sample.
     * @return Filtered output sample.
     */
    float update(float input)
    {
        output_ = alpha_ * input + (1.0f - alpha_) * output_;
        return output_;
    }

    /**
     * @brief Retunes the filter to a new cutoff frequency, keeping the
     * sample rate unchanged.
     *
     * @param cutoffFreq New cutoff frequency, in Hz.
     */
    void updateCutoffFrequency(float cutoffFreq)
    {
        cutoffFreq_ = cutoffFreq;
        updateCoefficients();
    }

    /**
     * @brief Resets the filter's output to a known value.
     *
     * @param initial Output value to reset to.
     */
    void reset(float initial = 0.0f)
    {
        output_ = initial;
    }

private:
    /**
     * @brief Recomputes the smoothing coefficient (alpha_) from
     * cutoffFreq_/sampleRate_. Called on construction and whenever the
     * cutoff frequency changes.
     */
    void updateCoefficients()
    {
        float rc = 1.0f / (2.0f * 3.14159265f * cutoffFreq_);
        float dt = 1.0f / sampleRate_;
        alpha_ = dt / (rc + dt);
    }

private:
    float cutoffFreq_, sampleRate_;
    float alpha_ = 0.0f;
    float output_ = 0.0f;
};

#endif // LOW_PASS_FILTER_H
