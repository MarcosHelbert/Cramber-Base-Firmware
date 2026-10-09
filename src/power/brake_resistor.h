#ifndef BRAKE_RESISTOR_H
#define BRAKE_RESISTOR_H

#include <persistence/config_store.h>

class BrakeResistor
{
public:
    BrakeResistor(const PersistedConfig &config) : config_(config) {}

    void init();
    void update(float vbus, float i_bus);

    void arm();
    void disarm();

private:
    void setDuty_(float duty);

private:
    const PersistedConfig &config_;

    uint32_t arr_;

    bool armed_ = false;
    bool saturated_ = false;
};

#endif // BRAKE_RESISTOR_H
