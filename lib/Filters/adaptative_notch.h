#ifndef ADAPTIVE_NOTCH_FILTER_H
#define ADAPTIVE_NOTCH_FILTER_H

#include <cstdint>
#include <cmath>

class AdaptiveNotchFilter
{
public:
    /**
     * @brief Constructs a biquad notch filter tuned to a center frequency.
     *
     * @param centerFreq Frequency to attenuate, in Hz.
     * @param bandwidth Width of the rejection band around centerFreq, in Hz.
     * @param sampleRate Sampling rate of the input signal, in Hz.
     */
    AdaptiveNotchFilter(float centerFreq, float bandwidth, float sampleRate)
        : centerFreq_(centerFreq),
          bandwidth_(bandwidth),
          sampleRate_(sampleRate)
    {
        updateCoefficients();
    }

    /**
     * @brief Filters one input sample through the notch filter.
     *
     * @param input Raw input sample.
     * @return Filtered output sample.
     */
    float update(float input)
    {
        float output = b0_ * input + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;

        x2_ = x1_;
        x1_ = input;
        y2_ = y1_;
        y1_ = output;

        return output;
    }

    /**
     * @brief Retunes the filter to a new center frequency, keeping the
     * bandwidth and sample rate unchanged.
     *
     * @param centerFreq New frequency to attenuate, in Hz.
     */
    void updateCenterFrequency(float centerFreq)
    {
        centerFreq_ = centerFreq;
        updateCoefficients();
    }

    /**
     * @brief Clears the filter's internal history (input/output delay lines).
     */
    void reset()
    {
        x1_ = x2_ = y1_ = y2_ = 0.0f;
    }

private:
    /**
     * @brief Recomputes the biquad coefficients from centerFreq_/bandwidth_/
     * sampleRate_. Called on construction and whenever the center frequency
     * changes.
     */
    void updateCoefficients()
    {
        float w0 = 2.0f * 3.14159265f * centerFreq_ / sampleRate_;
        float alpha = std::sin(w0) / (2.0f * (bandwidth_ / sampleRate_));

        b0_ = 1.0f;
        b1_ = -2.0f * std::cos(w0);
        b2_ = 1.0f;

        float a0 = 1.0f + alpha;
        a1_ = -2.0f * std::cos(w0);
        a2_ = 1.0f - alpha;

        b0_ /= a0;
        b1_ /= a0;
        b2_ /= a0;
        a1_ /= a0;
        a2_ /= a0;
    }

private:
    float centerFreq_, bandwidth_, sampleRate_;
    float b0_, b1_, b2_, a1_, a2_;
    float x1_ = 0.0f, x2_ = 0.0f, y1_ = 0.0f, y2_ = 0.0f;
};

#endif // ADAPTIVE_NOTCH_FILTER_H
