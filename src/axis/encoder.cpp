#include "encoder.h"

#include <utils.h>
#include <math.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

#include <hardware/dwt.h>

Encoder::Encoder(uint16_t cpr, bool reverse_dir,
                 uint32_t update_frequency_hz, uint32_t foc_frequency_hz)
    : resolution_(cpr),
      half_resolution_(static_cast<int16_t>(cpr >> 1)),
      update_frequency_hz_(update_frequency_hz),
      foc_frequency_hz_(foc_frequency_hz),
      two_pi_over_cpr_(TWO_PI / static_cast<float>(cpr)),
      reverse_direction_(reverse_dir),
      dt_foc_(1.0f / static_cast<float>(foc_frequency_hz))
{
    const float bw_mt = 80.0f;
    alpha_mt_ = 1.0f - expf(-TWO_PI * bw_mt / static_cast<float>(update_frequency_hz));

    const float bw_acc = 20.0f;
    alpha_acc_ = 1.0f - expf(-TWO_PI * bw_acc / static_cast<float>(update_frequency_hz));

    const float bw_pll = 50.0f;
    pll_kp_ = 2.0f * TWO_PI * bw_pll;
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_);
}

void Encoder::init()
{
    // encoder hardware
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    // AB
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4 | GPIO5);
    gpio_set_af(GPIOB, GPIO_AF2, GPIO4 | GPIO5);

    // Z (index)
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_af(GPIOC, GPIO_AF2, GPIO9);

    timer_set_period(TIM3, resolution_ - 1);
    timer_slave_set_mode(TIM3, TIM_SMCR_SMS_EM3);

    timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(TIM3, TIM_IC2, TIM_IC_IN_TI2);
    timer_ic_set_filter(TIM3, TIM_IC1, TIM_IC_CK_INT_N_8);
    timer_ic_set_filter(TIM3, TIM_IC2, TIM_IC_CK_INT_N_8);

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
        timer_set_counter(TIM3, 0);

        overflow_count_ = 0;
        last_count_enc_ = 0;
        last_count_pll_ = 0;
        absolute_position_ = 0;

        mt_count_accum_ = 0;
        mt_time_accum_ = 0;
        velocity_mt_ = 0.0f;

        pll_pos_ = 0.0f;
        pll_vel_ = 0.0f;
        velocity_pll_ = 0.0f;
        last_velocity_pll_ = 0.0f;
        acceleration_pll_ = 0.0f;

        indexed_ = true;
    }

    // ========= POSITION =========
    const uint32_t raw = TIM_CNT(TIM3);
    const uint32_t current = reverse_direction_ ? static_cast<uint32_t>((resolution_ - 1u) - raw) : raw;

    int32_t diff = static_cast<int32_t>(current - last_count_enc_);
    if (diff > half_resolution_)
    {
        diff -= resolution_;
        overflow_count_--;
    }
    else if (diff < -half_resolution_)
    {
        diff += resolution_;
        overflow_count_++;
    }

    absolute_position_ += diff;
    mechanical_angle_ = static_cast<float>(absolute_position_) * two_pi_over_cpr_;
    last_count_enc_ = current;

    // ===== M/T ESTIMATOR =====
    const uint32_t now = dwt_get_cycles();
    if (diff != 0)
    {
        const uint32_t dt_cycles = now - mt_last_cyccnt_;
        mt_last_cyccnt_ = now;
        mt_silence_start_ = now;

        mt_count_accum_ += diff;
        mt_time_accum_ += dt_cycles;

        if (mt_count_accum_ >= MT_WINDOW_COUNTS ||
            mt_count_accum_ <= -MT_WINDOW_COUNTS)
        {
            const float delta_rad = static_cast<float>(mt_count_accum_) * two_pi_over_cpr_;
            const float delta_sec = static_cast<float>(mt_time_accum_) * CYCLES_TO_SEC;
            const float omega = delta_rad / delta_sec;

            velocity_mt_ += alpha_mt_ * (omega - velocity_mt_);

            mt_count_accum_ = 0;
            mt_time_accum_ = 0;
        }
    }
    else
    {
        const uint32_t silence = now - mt_silence_start_;
        if (silence > MT_TIMEOUT_CYCLES)
        {
            velocity_mt_ += alpha_mt_ * (0.0f - velocity_mt_);
            mt_count_accum_ = 0;
            mt_time_accum_ = 0;
        }
    }

    float raw_acc = (velocity_mt_ - last_velocity_mt_) * static_cast<float>(update_frequency_hz_);
    acceleration_filtered_ += alpha_acc_ * (raw_acc - acceleration_filtered_);
    last_velocity_mt_ = velocity_mt_;
}

void Encoder::updateFOC()
{
    const uint32_t raw = TIM_CNT(TIM3);
    const uint32_t current = reverse_direction_ ? static_cast<uint32_t>((resolution_ - 1u) - raw) : raw;

    int32_t diff = static_cast<int32_t>(current - last_count_pll_);
    if (diff > half_resolution_)
        diff -= resolution_;
    else if (diff < -half_resolution_)
        diff += resolution_;

    last_count_pll_ = current;

    pll_pos_ += dt_foc_ * pll_vel_;

    const float delta = static_cast<float>(absolute_position_) + static_cast<float>(diff) - pll_pos_;

    pll_pos_ += dt_foc_ * pll_kp_ * delta;
    pll_vel_ += dt_foc_ * pll_ki_ * delta;

    if (pll_vel_ > -0.5f && pll_vel_ < 0.5f && diff == 0)
    {
        pll_vel_ = 0.0f;
    }

    velocity_pll_ = pll_vel_ * two_pi_over_cpr_;

    acceleration_pll_ = (velocity_pll_ - last_velocity_pll_) * static_cast<float>(foc_frequency_hz_);
    last_velocity_pll_ = velocity_pll_;
}

void Encoder::reset()
{
    timer_set_counter(TIM3, 0);
    timer_clear_flag(TIM3, TIM_SR_CC4IF);

    absolute_position_ = 0;
    overflow_count_ = 0;
    last_count_enc_ = 0;
    last_count_pll_ = 0;

    const uint32_t now = dwt_get_cycles();
    mt_count_accum_ = 0;
    mt_time_accum_ = 0;
    mt_last_cyccnt_ = now;
    mt_silence_start_ = now;

    velocity_mt_ = 0.0f;

    pll_pos_ = 0.0f;
    pll_vel_ = 0.0f;
    velocity_pll_ = 0.0f;
    last_velocity_pll_ = 0.0f;
    acceleration_pll_ = 0.0f;

    last_velocity_mt_ = 0.0f;
    acceleration_filtered_ = 0.0f;

    mechanical_angle_ = 0.0f;
    indexed_ = false;
}