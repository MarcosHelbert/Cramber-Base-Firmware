#include "brake_resistor.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include <algorithm>

#include <debug.h>

void BrakeResistor::init()
{
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);
    gpio_set_af(GPIOB, GPIO_AF1, GPIO11);

    rcc_periph_reset_pulse(RST_TIM2);

    uint32_t tim_clk = rcc_apb1_frequency;
    uint32_t ppre1 = (RCC_CFGR & RCC_CFGR_PPRE1_MASK) >> RCC_CFGR_PPRE1_SHIFT;
    if (ppre1 != 0)
        tim_clk *= 2;

    arr_ = (tim_clk / config_.pwm_freq_hz) - 1;

    timer_set_prescaler(TIM2, 0);
    timer_set_period(TIM2, arr_);
    timer_set_counter(TIM2, 0);

    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1);
    timer_disable_oc_preload(TIM2, TIM_OC4);
    timer_set_oc_polarity_high(TIM2, TIM_OC4);
    timer_set_oc_value(TIM2, TIM_OC4, 0);

    timer_disable_preload(TIM2);
    timer_enable_counter(TIM2);

    setDuty_(0.0f);
}

void BrakeResistor::update(float vbus, float ibus)
{
    const float Pmax = config_.brake_resistor_power_rating_w;
    const float R = config_.brake_resistor_resistance_ohm;
    if (!armed_ || R <= 0.0f || Pmax <= 0.0f || vbus <= 1.0f)
    {
        setDuty_(0.0f);
        return;
    }

    float brake_duty = 0.0f;
    float brake_current = 0.0f;

    static float ibus_filt = 0.0f;
    ibus_filt += (ibus - ibus_filt) * 0.5f;

    constexpr float kRampV = 2.0f;
    const float over_v = vbus - config_.brake_resistor_trigger_voltage;
    if (over_v > 0.0f)
    {
        brake_current = -ibus_filt - config_.brake_resistor_max_regen_current;
        if (brake_current > 0.0f)
        {
            brake_duty = (brake_current * R) / vbus;
        }
        brake_duty = std::max(brake_duty, over_v / kRampV);
    }

    if (brake_duty >= 0.95f)
    {
        saturated_ = true;
    }
    else
    {
        saturated_ = false;
    }

    float max_duty = Pmax / ((vbus * vbus) / R);
    if (brake_duty > max_duty)
    {
        brake_duty = max_duty;
    }

    float duty_ = std::clamp(brake_duty, 0.0f, 0.95f);
    setDuty_(duty_);
}

void BrakeResistor::setDuty_(float duty)
{
    duty = std::clamp(duty, 0.0f, 0.95f);

    uint32_t period = arr_ + 1;
    uint32_t high_on = (uint32_t)(period * duty);

    timer_set_oc_value(TIM2, TIM_OC4, high_on);
}

void BrakeResistor::arm()
{
    armed_ = true;
    cdcLog("[BRK] arm\r\n");

    timer_enable_oc_output(TIM2, TIM_OC4);
}

void BrakeResistor::disarm()
{
    armed_ = false;

    setDuty_(0.0f);
    timer_disable_oc_output(TIM2, TIM_OC4);
    cdcLog("[BRK] disarm\r\n");
}