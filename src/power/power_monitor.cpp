#include "power_monitor.h"

#include <hardware/adc.h>

void PowerMonitor::init()
{
    const int N = 8;
    uint32_t sum = 0;
    for (int i = 0; i < N; ++i)
    {
        sum += ADC::get(ADC::Channel::VBUS);
    }
    uint16_t avg = static_cast<uint16_t>(sum / N);
    float vbus = adcToVoltage(avg) * config_.vbus_divider_ratio;
    vbus_filtered_ = vbus;
}

void PowerMonitor::update()
{
    uint16_t raw = ADC::get(ADC::Channel::VBUS);

    float vbus = adcToVoltage(raw) * config_.vbus_divider_ratio;

    vbus_filtered_ += VBUS_FILTER_ALPHA * (vbus - vbus_filtered_);
}

bool PowerMonitor::isPowerOk() const
{
    bool a = vbus_filtered_ > config_.vbus_min_voltage;
    bool b = vbus_filtered_ < config_.vbus_max_voltage;
    return a && b;
}

float PowerMonitor::adcToVoltage(uint16_t raw) const
{
    return (raw / ADC_MAX) * ADC_VREF;
}