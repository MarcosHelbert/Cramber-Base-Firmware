#include "current_sensor.h"

#include <libopencm3/stm32/timer.h>

#include <hardware/adc.h>
#include <config.h>

void CurrentSensor::init()
{
    updateScale();
}

void CurrentSensor::updateScale()
{
    const float v_ref = 3.3f;
    const float adc_max = 4096.0f;

    current_scale_ = (v_ref / adc_max) / (CURRENT_AMP_GAIN * SHUNT_RESISTANCE);
}

void CurrentSensor::waitForSample()
{
    timer_clear_flag(TIM1, TIM_SR_UIF);
    while (!(TIM_SR(TIM1) & TIM_SR_UIF))
        ;
}

void CurrentSensor::calibrateOffsets(uint16_t samples)
{
    float sum_b = 0.0f;
    float sum_c = 0.0f;

    if (samples == 0)
        return;

    for (uint16_t i = 0; i < samples; i++)
    {
        waitForSample();
        sum_b += ADC::getInjected(ADC::InjectedChannel::IB);
        sum_c += ADC::getInjected(ADC::InjectedChannel::IC);
    }

    offset_ib_ = sum_b / samples;
    offset_ic_ = sum_c / samples;
}

float CurrentSensor::adcToCurrent(uint16_t raw, float offset) const
{
    return (static_cast<float>(raw) - offset) * current_scale_;
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
}