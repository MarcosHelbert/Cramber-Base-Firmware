#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <cstdint>

class CurrentSensor
{
public:
    CurrentSensor() = default;

    void init();
    void calibrateOffsets(uint16_t samples = 5000);

    void waitForSample();
    void update();
    void reset();

    float getIa() const { return i_a_; }
    float getIb() const { return i_b_; }
    float getIc() const { return i_c_; }

private:
    float i_a_ = 0.0f;
    float i_b_ = 0.0f;
    float i_c_ = 0.0f;

    float offset_ib_ = 2048.0f;
    float offset_ic_ = 2048.0f;
    float current_scale_ = 0.0f;

    void updateScale();
    float adcToCurrent(uint16_t raw, float offset) const;

    static constexpr float FILTER_ALPHA = 1.0f;
};

#endif // CURRENT_SENSOR_H