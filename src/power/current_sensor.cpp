#include "current_sensor.h"

#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>

#include <hardware/adc.h>
#include <system/timer.h>

void CurrentSensor::init()
{
    updateScale();
}

bool CurrentSensor::waitForSample()
{
    const uint32_t timeout_us = 10000;

    ADC_SR(ADC1) &= ~(ADC_SR_JEOC | ADC_SR_OVR);

    uint32_t start = micros();
    while (!(ADC_SR(ADC1) & ADC_SR_JEOC))
    {
        if ((micros() - start) > timeout_us)
            return false;
    }

    ADC_SR(ADC1) &= ~ADC_SR_JEOC;
    return true;
}

void CurrentSensor::calibrateOffsets(uint16_t samples)
{
    if (samples == 0)
        return;

    float sum_b = 0.0f;
    float sum_c = 0.0f;
    uint16_t valid = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        iwdg_reset();

        if (!waitForSample())
            continue;

        sum_b += ADC::getInjected(ADC::InjectedChannel::IB);
        sum_c += ADC::getInjected(ADC::InjectedChannel::IC);
        valid++;
    }

    if (valid == 0)
        return;

    offset_ib_ = sum_b / valid;
    offset_ic_ = sum_c / valid;
}

void CurrentSensor::update()
{
    uint16_t raw_ib = ADC::getInjected(ADC::InjectedChannel::IB);
    uint16_t raw_ic = ADC::getInjected(ADC::InjectedChannel::IC);

    float ib = adcToCurrent(raw_ib, offset_ib_);
    float ic = adcToCurrent(raw_ic, offset_ic_);

    i_b_ += FILTER_ALPHA * (ib - i_b_);
    i_c_ += FILTER_ALPHA * (ic - i_c_);

    i_a_ = -i_b_ - i_c_;
}

void CurrentSensor::reset()
{
    i_a_ = i_b_ = i_c_ = 0.0f;
    offset_ib_ = 2048.0f;
    offset_ic_ = 2048.0f;
}

float CurrentSensor::adcToCurrent(uint16_t raw, float offset) const
{
    return (static_cast<float>(raw) - offset) * current_scale_;
}

void CurrentSensor::updateScale()
{
    const float v_ref = 3.3f;
    const float adc_max = 4096.0f;

    current_scale_ = (v_ref / adc_max) / (config_.current_amp_gain * config_.shunt_resistance);
}