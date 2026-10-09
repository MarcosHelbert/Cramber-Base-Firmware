#include "encoder.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

#include <utils.h>

namespace kf
{
    static constexpr float q0 = 1e-6f;
    static constexpr float q1 = 1e-3f;
    static constexpr float q2 = 5e-2f;
    static constexpr float r = 1e-5f;
}

Encoder::Encoder(const PersistedConfig &config)
    : config_(config),
      two_pi_over_cpr_(TWO_PI / static_cast<float>(config_.encoder_cpr)),
      kalman_(1.0f / static_cast<float>(config_.pwm_freq_hz), kf::q0, kf::q1, kf::q2, kf::r)
{
}

void Encoder::init()
{
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    // AB
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4 | GPIO5);
    gpio_set_af(GPIOB, GPIO_AF2, GPIO4 | GPIO5);

    // Z (index)
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_af(GPIOC, GPIO_AF2, GPIO9);

    timer_set_period(TIM3, config_.encoder_cpr - 1);
    timer_slave_set_mode(TIM3, TIM_SMCR_SMS_EM3);

    timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(TIM3, TIM_IC2, TIM_IC_IN_TI2);

    timer_ic_set_filter(TIM3, TIM_IC1, TIM_IC_CK_INT_N_4);
    timer_ic_set_filter(TIM3, TIM_IC2, TIM_IC_CK_INT_N_4);

    // index
    timer_ic_set_input(TIM3, TIM_IC4, TIM_IC_IN_TI4);
    timer_ic_set_filter(TIM3, TIM_IC4, TIM_IC_CK_INT_N_8);
    timer_ic_set_polarity(TIM3, TIM_IC4, TIM_IC_RISING);
    timer_ic_disable(TIM3, TIM_IC4);

    timer_enable_counter(TIM3);
    reset();
}

void Encoder::setIdxSubscribe(bool enable)
{
    if (enable)
    {
        timer_clear_flag(TIM3, TIM_SR_CC4IF);
        timer_ic_enable(TIM3, TIM_IC4);
    }
    else
    {
        timer_ic_disable(TIM3, TIM_IC4);
    }
}

void Encoder::update()
{
    // ========= INDEX =========
    if (timer_get_flag(TIM3, TIM_SR_CC4IF))
    {
        timer_clear_flag(TIM3, TIM_SR_CC4IF);
        reset();
        indexed_ = true;
        return;
    }

    // ========= POSITION =========
    const uint32_t resolution = config_.encoder_cpr;
    const int32_t half_resolution = static_cast<int32_t>(resolution >> 1);

    const uint32_t raw = TIM_CNT(TIM3);
    const uint32_t current = config_.encoder_reverse
                                 ? static_cast<uint32_t>((resolution - 1u) - raw)
                                 : raw;

    int32_t diff = static_cast<int32_t>(current - last_count_);
    if (diff > half_resolution)
    {
        diff -= static_cast<int32_t>(resolution);
    }
    else if (diff < -half_resolution)
    {
        diff += static_cast<int32_t>(resolution);
    }

    absolute_position_ += diff;
    last_count_ = current;

    // ========= KALMAN (angle/velocity/acceleration estimate) =========
    const float position_rad = static_cast<float>(absolute_position_ - config_.mechanical_offset) * two_pi_over_cpr_;

    kalman_.predict();
    kalman_.update(position_rad);
}

void Encoder::reset()
{
    timer_set_counter(TIM3, 0);
    timer_clear_flag(TIM3, TIM_SR_CC4IF);

    last_count_ = 0;
    absolute_position_ = 0;
    indexed_ = false;

    kalman_.reset();
}