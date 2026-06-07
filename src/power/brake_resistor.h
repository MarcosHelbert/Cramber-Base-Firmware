#ifndef BRAKING_RESISTOR_H
#define BRAKING_RESISTOR_H

#include <cstdint>
#include <config.h>

class BrakingResistor
{
public:
    BrakingResistor(float resistance_ohm, float power_rating_w, float max_regen_current, uint32_t pwm_hz = 1000);

    void init();
    void update(float vbus, float i_bus);
    void updateOvervoltage(float vbus);

    void arm();
    void disarm();

    bool isSaturated() const { return saturated_; }
    bool isArmed() const { return armed_; }

private:
    float R_;
    float Pmax_;
    uint32_t pwm_hz_;
    float max_regen_current_;

    uint32_t arr_;

    bool armed_ = false;
    bool saturated_ = false;

    void setDuty_(float duty);
};

#endif // BRAKING_RESISTOR_H