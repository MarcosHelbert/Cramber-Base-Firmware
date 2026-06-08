#include "brake_resistor.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include <algorithm>
#include <debug.h>

BrakingResistor::BrakingResistor(float resistance_ohm, float power_rating_w, float max_regen_current, uint32_t pwm_hz)
    : R_(resistance_ohm),
      Pmax_(power_rating_w),
      pwm_hz_(pwm_hz),
      max_regen_current_(max_regen_current),
      arr_(0)
{
}

void BrakingResistor::init()
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

    arr_ = (tim_clk / pwm_hz_) - 1;

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

void BrakingResistor::update(float vbus, float ibus)
{
    if (!armed_ || R_ <= 0.0f || Pmax_ <= 0.0f || vbus <= 1.0f)
    {
        setDuty_(0.0f);
        return;
    }

    float brake_duty = 0.0f;
    float brake_current = 0.0f;

    static float ibus_filt = 0.0f;
    ibus_filt += (ibus - ibus_filt) * 0.5f;

    brake_current = -ibus_filt - max_regen_current_;
    if (brake_current > 0.0f)
    {
        brake_duty = (brake_current * R_) / vbus;
    }

    if (brake_duty >= 0.95f)
    {
        saturated_ = true;
    }
    else
    {
        saturated_ = false;
    }

    float max_duty = Pmax_ / ((vbus * vbus) / R_);
    if (brake_duty > max_duty)
    {
        brake_duty = max_duty;
    }

    float duty_ = std::clamp(brake_duty, 0.0f, 0.95f);
    setDuty_(duty_);
}

void BrakingResistor::setDuty_(float duty)
{
    duty = std::clamp(duty, 0.0f, 0.95f);

    uint32_t period = arr_ + 1;
    uint32_t high_on = (uint32_t)(period * duty);

    timer_set_oc_value(TIM2, TIM_OC4, high_on);
}

void BrakingResistor::arm()
{
    armed_ = true;
    cdcLog("[BRK] arm\r\n");

    timer_enable_oc_output(TIM2, TIM_OC4);
}

void BrakingResistor::disarm()
{
    armed_ = false;

    setDuty_(0.0f);
    timer_disable_oc_output(TIM2, TIM_OC4);
    cdcLog("[BRK] disarm\r\n");
}