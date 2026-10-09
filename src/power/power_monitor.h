#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <cstdint>

#include <persistence/config_store.h>

class PowerMonitor
{
public:
    explicit PowerMonitor(const PersistedConfig &config) : config_(config) {}

    void init();
    void update();

    float getVbus() const { return vbus_filtered_; }
    bool isPowerOk() const;

private:
    float adcToVoltage(uint16_t raw) const;

private:
    const PersistedConfig &config_;

    float vbus_filtered_ = 0.0f;

    static constexpr float ADC_VREF = 3.3f;
    static constexpr float ADC_MAX = 4095.0f;

    static constexpr float VBUS_FILTER_ALPHA = 0.05f;
};

#endif // POWER_MONITOR_H
