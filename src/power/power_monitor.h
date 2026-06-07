
#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <cstdint>

class PowerMonitor
{
public:
    PowerMonitor() = default;

    void init();
    void update();

    float getVbus() const { return vbus_filtered_; }
    bool isPowerOk() const;

private:
    float vbus_filtered_ = 0.0f;

    float adcToVoltage(uint16_t raw) const;

    static constexpr float ADC_VREF = 3.3f;
    static constexpr float ADC_MAX = 4095.0f;

    static constexpr float VBUS_FILTER_ALPHA = 0.05f;
};

#endif // POWER_MONITOR_H