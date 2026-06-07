#include "motor.h"

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include <system_state.h>
#include <config.h>
#include <svm.h>
#include <debug.h>

#include <hardware/timer.h>

Motor::Motor(
    PowerMonitor &power,
    CurrentSensor &current,
    DRV8301 &driver,
    Encoder &encoder) : power_(power),
                        current_(current),
                        driver_(driver),
                        encoder_(encoder) {}

uint8_t Motor::encodeDeadtime(uint32_t cycles)
{
    if (cycles <= 127)
        return cycles;

    if (cycles <= 255)
        return 0x80 | ((cycles - 64) / 2);

    if (cycles <= 511)
        return 0xC0 | ((cycles - 32) / 8);

    if (cycles <= 1023)
        return 0xE0 | ((cycles - 32) / 16);

    return 0xFF;
}

void Motor::init()
{
    state_ = MotorState::DISABLED;

    uint32_t tim_clk = rcc_apb2_frequency * 2;

    period_ = (tim_clk / (2 * MOTOR_PWM_FREQ_HZ)) - 1;
    pwm_period_s_ = 1.0f / MOTOR_PWM_FREQ_HZ;
    uint32_t deadtime_cycles = (uint64_t)tim_clk * MOTOR_DEADTIME_NS / 1'000'000'000ULL;

    timer_set_repetition_counter(TIM1, 1);

    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO8 | GPIO9 | GPIO10);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO8 | GPIO9 | GPIO10);

    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO13 | GPIO14 | GPIO15);
    gpio_set_af(GPIOB, GPIO_AF1, GPIO13 | GPIO14 | GPIO15);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO13 | GPIO14 | GPIO15);

    timer_disable_counter(TIM1);
    timer_set_master_mode(TIM1, TIM_CR2_MMS_UPDATE);

    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
    timer_set_period(TIM1, period_);
    timer_enable_preload(TIM1);

    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);

    timer_set_oc_mode(TIM1, TIM_OC1N, TIM_OCM_PWM2);
    timer_set_oc_mode(TIM1, TIM_OC2N, TIM_OCM_PWM2);
    timer_set_oc_mode(TIM1, TIM_OC3N, TIM_OCM_PWM2);

    timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM1);
    timer_set_oc_value(TIM1, TIM_OC4, period_ - 2);

    timer_enable_oc_preload(TIM1, TIM_OC1);
    timer_enable_oc_preload(TIM1, TIM_OC2);
    timer_enable_oc_preload(TIM1, TIM_OC3);
    timer_enable_oc_preload(TIM1, TIM_OC4);

    timer_set_oc_polarity_high(TIM1, TIM_OC1);
    timer_set_oc_polarity_high(TIM1, TIM_OC1N);
    timer_set_oc_polarity_high(TIM1, TIM_OC2);
    timer_set_oc_polarity_high(TIM1, TIM_OC2N);
    timer_set_oc_polarity_high(TIM1, TIM_OC3);
    timer_set_oc_polarity_high(TIM1, TIM_OC3N);

    timer_set_deadtime(TIM1, encodeDeadtime(deadtime_cycles));

    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_oc_output(TIM1, TIM_OC1N);
    timer_enable_oc_output(TIM1, TIM_OC2);
    timer_enable_oc_output(TIM1, TIM_OC2N);
    timer_enable_oc_output(TIM1, TIM_OC3);
    timer_enable_oc_output(TIM1, TIM_OC3N);

    timer_enable_oc_output(TIM1, TIM_OC4);

    setDuty(0.5f, 0.5f, 0.5f);

    timer_enable_irq(TIM1, TIM_DIER_CC4IE);
    nvic_set_priority(NVIC_TIM1_CC_IRQ, 5);
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);

    timer_enable_break_main_output(TIM1);
    timer_enable_counter(TIM1);
}

void Motor::setOutput(float va, float vb, float vc)
{
    float vbus = power_.getVbus();

    float tA, tB, tC;
    svm(va, vb, vc, vbus, tA, tB, tC);

    setDuty(tA, tB, tC);
}

bool Motor::arm()
{
    if (state_ == MotorState::ERROR)
        return false;

    if (!power_.isPowerOk())
    {
        state_ = MotorState::ERROR;
        cdcLog("[MOTOR] arm blocked: power out of range vbus=%.2f\r\n", power_.getVbus());
        return false;
    }

    cdcLog("[MOTOR] arm: enabling driver\r\n");
    driver_.enable();
    delay_ms(100);

    current_.reset();
    current_.calibrateOffsets();
    state_ = MotorState::IDLE;
    cdcLog("[MOTOR] arm complete: state=IDLE vbus=%.2f\r\n", power_.getVbus());
    return true;
}

void Motor::disarm()
{
    state_ = MotorState::DISABLED;
    setDuty(0.5f, 0.5f, 0.5f);
    driver_.disable();
    current_.reset();
    cdcLog("[MOTOR] disarm: driver disabled\r\n");
}

void Motor::start()
{
    if (state_ != MotorState::IDLE)
        return;

    state_ = MotorState::RUNNING;
}

void Motor::stop()
{
    if (state_ != MotorState::RUNNING)
        return;

    state_ = MotorState::IDLE;
}

bool Motor::measurePhaseResistance()
{
    if (!arm())
        return false;

    float sum_I = 0.0f;
    const int R_cycles = 100;

    for (int i = 0; i < R_cycles; i++)
    {
        setTestVoltage();
        delay_us(2000);
        current_.waitForSample();
        current_.update();
        float ib = current_.getIb();
        float ic = current_.getIc();
        sum_I += (abs(ib) + abs(ic)) * 0.5f;
    }

    setDuty(0.5f, 0.5f, 0.5f);

    float avg_I = sum_I / R_cycles;

    float V = CALIBRATION_MAX_VOLTAGE;
    float R_measured = V / (avg_I + 1e-6f);
    float R_phase = R_measured * 0.5f;

    if (!(R_phase > 0.001f && R_phase < 10.0f))
    {
        disarm();
        return false;
    }

    phase_resistance_ = R_phase;
    return true;
}

bool Motor::measurePhaseInductance()
{
    if (!arm())
        return false;

    const int cycles = 500;
    float sum_dI = 0.0f;

    for (int i = 0; i < cycles; i++)
    {
        setTestVoltage();
        delay_us(5);

        current_.waitForSample();
        current_.update();

        float Ib = current_.getIb();
        float Ic = current_.getIc();
        float I1 = (Ib + Ic) * 0.5f;

        delay_us(10);

        current_.waitForSample();
        current_.update();

        Ib = current_.getIb();
        Ic = current_.getIc();
        float I2 = (Ib + Ic) * 0.5f;

        float dI = abs(I2 - I1);
        sum_dI += dI;
    }

    setDuty(0.5f, 0.5f, 0.5f);

    float avg_dI = sum_dI / cycles;

    float dt = 10e-6f;
    float V = CALIBRATION_MAX_VOLTAGE;

    float L_measured = (V * dt) / (avg_dI + 1e-6f);
    float L_phase = L_measured * 0.5f;

    if (!(L_phase > 1e-6f && L_phase < 0.1f))
    {
        disarm();
        return false;
    }

    phase_inductance_ = L_phase;
    return true;
}

float Motor::getElectricalAngle() const
{
    float count = encoder_.getPosition();
    float mech_angle = (count / ENCODER_CPR) * TWO_PI;
    float ele_angle = (mech_angle * MOTOR_POLE_PAIRS) - g_state.encoder_offset_rad;
    return normalizeAngle2Pi(ele_angle);
}

void Motor::setDuty(float a, float b, float c)
{
    a = clamp(a, 0.05f, 0.95f);
    b = clamp(b, 0.05f, 0.95f);
    c = clamp(c, 0.05f, 0.95f);

    timer_set_oc_value(TIM1, TIM_OC1, (uint32_t)(a * period_));
    timer_set_oc_value(TIM1, TIM_OC2, (uint32_t)(b * period_));
    timer_set_oc_value(TIM1, TIM_OC3, (uint32_t)(c * period_));
}

void Motor::setTestVoltage()
{
    float vbus = power_.getVbus();

    if (vbus < 1.0f)
        vbus = 1.0f;

    float duty = CALIBRATION_MAX_VOLTAGE / vbus;

    duty = clamp(duty, -0.3f, 0.3f);

    float da = 0.5f;
    float db = 0.5f + duty;
    float dc = 0.5f - duty;

    setDuty(da, db, dc);
}