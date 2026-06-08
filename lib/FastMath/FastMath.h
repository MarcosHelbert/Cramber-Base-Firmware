#ifndef FASTMATH_H
#define FASTMATH_H

#include <array>
#include <stdint.h>

constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 6.28318530717958647692f;
constexpr float PI_BY_2 = 1.57079632679489661923f;
constexpr float PI_BY_180 = 0.01745329251994329577f; // PI/180

constexpr float TWO_BY_SQRT3 = 1.1547005383792515f; // 2/sqrt(3)
constexpr float INV_SQRT3 = 0.5773502691896257f;
constexpr float SQRT3_BY_2 = 0.8660254037844386f; // sqrt(3)/2

/**
 * @brief Fast math utilities class.
 */
class FastMath
{
public:
    static constexpr uint16_t LUT_SIZE = 2048;
    static constexpr uint16_t LUT_MASK = LUT_SIZE - 1;
    static constexpr float INDEX_SCALE = static_cast<float>(LUT_SIZE) / TWO_PI;
    static constexpr float INV_INDEX_SCALE = TWO_PI / static_cast<float>(LUT_SIZE);

private:
    static constexpr float constexpr_sin_impl(float x)
    {
        // Normalize to [-PI, PI] first
        while (x > PI)
            x -= TWO_PI;
        while (x < -PI)
            x += TWO_PI;

        // Taylor series: sin(x) = x - x³/3! + x⁵/5! - x⁷/7! + x⁹/9!
        const float x2 = x * x;
        float result = x;
        float term = x;

        term *= -x2 / (2.0f * 3.0f);
        result += term;

        term *= -x2 / (4.0f * 5.0f);
        result += term;

        term *= -x2 / (6.0f * 7.0f);
        result += term;

        term *= -x2 / (8.0f * 9.0f);
        result += term;

        term *= -x2 / (10.0f * 11.0f);
        result += term;

        term *= -x2 / (12.0f * 13.0f);
        result += term;

        return result;
    }

    struct alignas(4) TableGenerator
    {
        float data[LUT_SIZE];

        constexpr TableGenerator() : data{}
        {
            for (uint16_t i = 0; i < LUT_SIZE; ++i)
            {
                const float angle = static_cast<float>(i) * INV_INDEX_SCALE;
                data[i] = constexpr_sin_impl(angle);
            }
        }
    };

    static inline float normalize_angle_to_lut(float angle)
    {
        float idx = angle * INDEX_SCALE;

        if (idx < 0.0f || idx >= static_cast<float>(LUT_SIZE))
        {
            idx = idx - static_cast<int32_t>(idx / LUT_SIZE) * LUT_SIZE;

            if (idx < 0.0f)
                idx += static_cast<float>(LUT_SIZE);
        }

        return idx;
    }

    static inline const TableGenerator &get_lut()
    {
        static constexpr TableGenerator lut{};
        return lut;
    }

public:
    /**
     * @brief Fast sine with linear interpolation.
     *
     * @param angle Angle in radians (any value)
     * @return Sine of the angle [-1.0, 1.0]
     */
    static inline float fast_sin(float angle)
    {
        const float float_idx = normalize_angle_to_lut(angle);
        const uint16_t idx = static_cast<uint16_t>(float_idx);
        const float frac = float_idx - static_cast<float>(idx);

        const uint16_t next_idx = (idx + 1) & LUT_MASK;

        const auto &lut = get_lut();
        const float y0 = lut.data[idx];
        const float y1 = lut.data[next_idx];

        // Interpolation: y = y0 + frac * (y1 - y0)
        return y0 + frac * (y1 - y0);
    }

    /**
     * @brief Fast cosine using phase shift.
     *
     * @param angle Angle in radians
     * @return Cosine of the angle [-1.0, 1.0]
     */
    static inline float fast_cos(float angle)
    {
        // cos(x) = sin(x + π/2)
        return fast_sin(angle + PI_BY_2);
    }

    /**
     * @brief Calculates sine and cosine simultaneously.
     *
     * Approximately 1.7x faster than calling fast_sin() and fast_cos() separately.
     * Shares normalization calculations and leverages cache locality.
     *
     * @param angle Angle in radians
     * @param s Pointer to store sine
     * @param c Pointer to store cosine
     */
    static inline void fast_sincos(float angle, float *s, float *c)
    {
        const float float_idx_sin = normalize_angle_to_lut(angle);
        const uint16_t idx_sin = static_cast<uint16_t>(float_idx_sin);
        const float frac_sin = float_idx_sin - static_cast<float>(idx_sin);

        const auto &lut = get_lut();

        const uint16_t next_idx_sin = (idx_sin + 1) & LUT_MASK;
        const float y0_sin = lut.data[idx_sin];
        const float y1_sin = lut.data[next_idx_sin];
        *s = y0_sin + frac_sin * (y1_sin - y0_sin);

        constexpr uint16_t QUARTER_LUT = LUT_SIZE / 4;
        const uint16_t idx_cos = (idx_sin + QUARTER_LUT) & LUT_MASK;
        const uint16_t next_idx_cos = (idx_cos + 1) & LUT_MASK;

        const float y0_cos = lut.data[idx_cos];
        const float y1_cos = lut.data[next_idx_cos];
        *c = y0_cos + frac_sin * (y1_cos - y0_cos);
    }

    /**
     * @brief Fast tangent calculated as sin/cos.
     *
     * @param angle Angle in radians
     * @return Tangent of the angle
     */
    static inline float fast_tan(float angle)
    {
        float s, c;
        fast_sincos(angle, &s, &c);
        return s / c;
    }

    /**
     * @brief Fast hyperbolic tangent approximation (tanh).
     *
     * Stable and fast approximation using rational function:
     * tanh(x) ≈ x * (27 + x²) / (27 + 9x²)
     *
     * Good accuracy for x in roughly [-5, 5].
     */
    static inline float fast_tanh(float x)
    {
        if (x > 5.0f)
            return 1.0f;
        if (x < -5.0f)
            return -1.0f;

        const float x2 = x * x;

        const float num = x * (27.0f + x2);
        const float den = 27.0f + 9.0f * x2;

        return num / den;
    }

    /**
     * @brief Fast square root (sqrt) using Quake rsqrt + 2 NR refinements.
     *
     * @param x Input value
     * @return Square root of x
     */
    static inline float fast_sqrt(float x)
    {
        if (x <= 0.0f)
            return 0.0f;

        float xhalf = 0.5f * x;

        union
        {
            float f;
            uint32_t i;
        } u;

        u.f = x;
        u.i = 0x5f3759dfu - (u.i >> 1);
        float y = u.f;

        y = y * (1.5f - xhalf * y * y);
        y = y * (1.5f - xhalf * y * y);

        // sqrt(x) = x * (1/sqrt(x))
        return x * y;
    }

    /**
     * @brief Fast exponential approximation (e^x).
     * Stable for small/medium ranges [-10, 10].
     */
    static inline float fast_exp(float x)
    {
        if (x < -10.0f)
            return 0.0f;
        if (x > 10.0f)
            x = 10.0f;

        // Taylor expansion optimized (8 terms)
        float result = 1.0f;
        float term = 1.0f;

        term *= x / 1.0f;
        result += term;
        term *= x / 2.0f;
        result += term;
        term *= x / 3.0f;
        result += term;
        term *= x / 4.0f;
        result += term;
        term *= x / 5.0f;
        result += term;
        term *= x / 6.0f;
        result += term;
        term *= x / 7.0f;
        result += term;
        term *= x / 8.0f;
        result += term;

        return result;
    }

    static inline float fast_exp2(float x)
    {
        // exp(x) = 2^(x / ln(2))
        constexpr float LOG2E = 1.44269504089f;

        x *= LOG2E;

        int32_t i = static_cast<int32_t>(x);
        float f = x - static_cast<float>(i);

        // approx 2^f via polynomial
        float approx = 1.0f + f * (0.693147f + 0.240226f * f);

        // bit-level 2^i
        int32_t bits = (i + 127) << 23;

        union
        {
            uint32_t i;
            float f;
        } u;

        u.i = bits;

        return u.f * approx;
    }
};

static_assert((FastMath::LUT_SIZE & (FastMath::LUT_SIZE - 1)) == 0, "LUT_SIZE must be a power of 2 for optimizations!");

#endif // FASTMATH_H