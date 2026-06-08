#ifndef UTILS_H
#define UTILS_H

#include <algorithm>
#include <cmath>
#include <FastMath.h>

/**
 * @brief Sign function.
 */
template <typename T>
constexpr int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

/**
 * @brief Deadband filter.
 */
template <typename T>
constexpr T deadband(T val, T thresh)
{
    return (abs(val) > thresh) ? val : T(0);
}

/**
 * @brief Map a value from one range to another.
 *
 * @param val The value to map
 * @param in_min Input range minimum
 * @param in_max Input range maximum
 * @param out_min Output range minimum
 * @param out_max Output range maximum
 * @return Mapped value
 */
template <typename T>
constexpr T map(T val, T in_min, T in_max, T out_min, T out_max)
{
    if (in_max == in_min)
        return out_min;

    return out_min + (val - in_min) * (out_max - out_min) / (in_max - in_min);
}

/**
 * @brief Normalize angle to [-PI, PI] range.
 * Optimized using fast modulo for periodic wrapping.
 *
 * @param angle Angle in radians
 * @return Normalized angle in [-PI, PI]
 */
inline float normalizeAngle(float angle)
{
    angle = fmodf(angle + PI, TWO_PI);
    return (angle >= 0.0f ? angle : angle + TWO_PI) - PI;
}

/**
 * @brief Normalize angle to [0, 2*PI] range.
 *
 * @param angle Angle in radians
 * @return Normalized angle in [0, 2*PI]
 */
inline float normalizeAngle2Pi(float angle)
{
    angle = fmodf(angle, TWO_PI);
    if (angle < 0.0f)
        angle += TWO_PI;
    return angle;
}

/**
 * @brief Square a value (optimized by avoiding function call overhead).
 *
 * @param x Input value
 * @return x²
 */
template <typename T>
constexpr T square(T x)
{
    return x * x;
}

/**
 * @brief Cube a value.
 *
 * @param x Input value
 * @return x³
 */
template <typename T>
constexpr T cube(T x)
{
    return x * x * x;
}

/**
 * @brief Check if two floats are approximately equal.
 *
 * @param a First value
 * @param b Second value
 * @param epsilon Tolerance (default: 1e-6)
 * @return true if |a - b| < epsilon
 */
inline bool approxEqual(float a, float b, float epsilon = 1e-6f)
{
    return std::abs(a - b) < epsilon;
}

/**
 * @brief Smooth step interpolation (cubic Hermite).
 * Provides smooth acceleration/deceleration between 0 and 1.
 *
 * @param t Input value [0, 1]
 * @return Smoothed value [0, 1]
 */
template <typename T>
constexpr T smoothstep(T t)
{
    t = std::clamp(t, T(0), T(1));
    return t * t * (T(3) - T(2) * t);
}

/**
 * @brief Smoother step interpolation (quintic).
 * Even smoother than smoothstep, with zero 1st and 2nd derivatives at boundaries.
 *
 * @param t Input value [0, 1]
 * @return Smoothed value [0, 1]
 */
template <typename T>
constexpr T smootherstep(T t)
{
    t = std::clamp(t, T(0), T(1));
    return t * t * t * (t * (t * T(6) - T(15)) + T(10));
}

/**
 * @brief Converts an angle in degrees to radians.
 *
 * @param angle_deg Angle in degrees
 * @return Angle in radians
 */
inline float angleToRadians(float angle_deg)
{
    return angle_deg * PI_BY_180;
}

/**
 * @brief Converts a float in [-1.0, 1.0] range to int16_t in [-32767, 32767] range.
 *
 * @param x Input value in [-1.0, 1.0]
 * @return int16_t Output value in [-32767, 32767]
 */
inline int16_t float_to_i16(float x)
{
    return static_cast<int16_t>(std::clamp(x, -1.0f, 1.0f) * 32767.0f);
}

/**
 * @brief Scales an 8-bit unsigned value by another 8-bit unsigned value, returning an 8-bit result.
 *
 * @param a The value to be scaled (0-255)
 * @param b The scaling factor (0-255)
 * @return uint8_t The scaled result (0-255)
 */
inline uint8_t scale_u8_u8(uint8_t a, uint8_t b)
{
    return (uint8_t)(((uint16_t)a * (uint16_t)b + 127u) / 255u);
}

/**
 * @brief Smooth deadband with quadratic ramp.
 * Prevents abrupt transitions around zero.
 *
 * @param x Input signal
 * @param dead Deadband threshold (>= 0)
 * @return Smoothed output
 */
template <typename T>
constexpr T smoothDeadband(T x, T dead)
{
    T ax = abs(x);
    if (ax < dead && dead > T(0))
    {
        T t = ax / dead;
        return std::copysign(t * t, x);
    }
    return x;
}

#endif // UTILS_H