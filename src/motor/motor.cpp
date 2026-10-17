#include "motor.h"

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/nvic.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <svm.h>
#include <debug.h>
#include <system/timer.h>

Motor::Motor(
    const PersistedConfig &config,
    const PowerMonitor &power,
    CurrentSensor &current,
    DRV8301 &driver)
    : config_(config),
      power_(power),
      current_(current),
      driver_(driver) {}

void Motor::init()
{
    state_ = MotorState::DISARMED;

    uint32_t tim_clk = rcc_apb2_frequency * 2;

    period_ = (tim_clk / (2 * config_.pwm_freq_hz)) - 1;
    uint32_t deadtime_cycles = (uint64_t)tim_clk * config_.pwm_deadtime_ns / 1'000'000'000ULL;

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

    // Force a DRV8301 EN_GATE reset pulse on every boot (including after
    // CMD_REBOOT), not just whenever something happens to call arm() next.
    // drv8301.init() only sets up GPIO/SPI and leaves EN_GATE low; a reboot
    // resets the MCU's own state but not the DRV8301 itself (PVDD1 power
    // stays up across an MCU-only reset), so any fault it had latched
    // internally would otherwise persist silently until something armed the
    // motor and happened to notice.
    clearFault();
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
        last_arm_fail_reason_ = ArmFailReason::POWER_OUT_OF_RANGE;
        cdcLog("[MOTOR] arm blocked: power out of range vbus=%.2f\r\n", power_.getVbus());
        return false;
    }

    setDuty(0.5f, 0.5f, 0.5f);

    cdcLog("[MOTOR] arm: enabling driver\r\n");
    if (!driver_.enable())
    {
        state_ = MotorState::ERROR;
        last_arm_fail_reason_ = ArmFailReason::DRIVER_FAULT;
        cdcLog("[MOTOR] arm blocked: gate driver did not come up\r\n");
        return false;
    }

    current_.reset();
    current_.calibrateOffsets();
    state_ = MotorState::ARMED;
    last_arm_fail_reason_ = ArmFailReason::NONE;
    cdcLog("[MOTOR] arm complete: state=ARMED vbus=%.2f\r\n", power_.getVbus());
    return true;
}

void Motor::disarm()
{
    state_ = MotorState::DISARMED;
    setDuty(0.5f, 0.5f, 0.5f);
    driver_.disable();
    current_.reset();
    cdcLog("[MOTOR] disarm: driver disabled\r\n");
}

void Motor::clearFault()
{
    setDuty(0.5f, 0.5f, 0.5f);

    // disable() alone just holds EN_GATE low and leaves it there -- it
    // never toggles back up, so the DRV8301 never actually re-powers its
    // gate driver/charge pump and latched faults (GVDD_OV in particular)
    // never clear. enable() does the full low->high reset pulse the
    // datasheet calls for (100us low, then 20ms for the device to come back
    // up) and reads back CONTROL_1/CONTROL_2 plus hasFault() to confirm the
    // reset actually took.
    bool recovered = driver_.enable();

    // Immediately disable again -- clearFault() is meant to reset the
    // fault latch and return to DISARMED, not leave the motor energized.
    driver_.disable();
    current_.reset();

    state_ = MotorState::DISARMED;
    last_arm_fail_reason_ = ArmFailReason::NONE;
    cdcLog("[MOTOR] clearFault: EN_GATE reset pulse sent, recovered=%d, state=DISARMED\r\n", recovered);
}

bool Motor::calibrate()
{
    float resistance = measurePhaseResistance();
    if (isNanBits(resistance))
    {
        is_calibrated_ = false;
        return false;
    }

    float inductance = measurePhaseInductance();
    if (isNanBits(inductance))
    {
        is_calibrated_ = false;
        return false;
    }

    g_config.phase_resistance = resistance;
    g_config.phase_inductance = inductance;
    g_config.current_kp = inductance * g_config.current_bandwidth_rad_s;
    g_config.current_ki = resistance * g_config.current_bandwidth_rad_s;
    g_config.motor_calibrated = true;

    is_calibrated_ = true;
    last_calibration_fail_reason_ = CalibrationFailReason::NONE;
    return true;
}

void Motor::setTestVoltage(float test_voltage)
{
    float vbus = power_.getVbus();
    if (vbus < 1.0f)
        vbus = 1.0f;

    float duty = test_voltage / vbus;
    duty = std::clamp(duty, -0.3f, 0.3f);

    setDuty(0.5f, 0.5f + duty, 0.5f - duty);
}

float Motor::measurePhaseResistance()
{
    constexpr uint32_t kSteps = 50;
    constexpr uint32_t kSettleDelayUs = 2000;
    constexpr float kMinResistance = 0.001f;
    constexpr float kMaxResistance = 10.0f;
    const float kNan = std::numeric_limits<float>::quiet_NaN();

    const float max_voltage = config_.calibration_max_voltage;
    const float max_current = config_.calibration_test_current;
    const float vbus = power_.getVbus();

    if (vbus < 1.0f || max_voltage <= 0.0f || max_current <= 0.0f)
        return kNan;

    float sum_vi = 0.0f;
    float sum_ii = 0.0f;

    for (uint32_t i = 1; i <= kSteps; i++)
    {
        iwdg_reset();
        float test_voltage = max_voltage * (static_cast<float>(i) / static_cast<float>(kSteps));
        setTestVoltage(test_voltage);
        delay_us(kSettleDelayUs);

        if (!current_.waitForSample())
        {
            setDuty(0.5f, 0.5f, 0.5f);
            last_calibration_fail_reason_ = CalibrationFailReason::RESISTANCE_SAMPLE_TIMEOUT;
            return kNan;
        }

        current_.update();
        float measured_current = (std::abs(current_.getIb()) + std::abs(current_.getIc())) * 0.5f;

        if (measured_current > max_current)
            break;

        sum_vi += test_voltage * measured_current;
        sum_ii += measured_current * measured_current;
    }

    setDuty(0.5f, 0.5f, 0.5f);

    if (sum_ii <= 0.0f)
    {
        last_calibration_fail_reason_ = CalibrationFailReason::RESISTANCE_OUT_OF_RANGE;
        return kNan;
    }

    float resistance_phase_to_phase = sum_vi / sum_ii;
    float resistance = resistance_phase_to_phase * 0.5f;

    if (isNanBits(resistance) || !(resistance > kMinResistance && resistance < kMaxResistance))
    {
        last_calibration_fail_reason_ = CalibrationFailReason::RESISTANCE_OUT_OF_RANGE;
        return kNan;
    }

    return resistance;
}

float Motor::measurePhaseInductance()
{
    constexpr uint32_t kCycles = 200;
    constexpr uint32_t kStepDelayUs = 5;
    constexpr uint32_t kRestDelayUs = 500;
    constexpr float kDt = static_cast<float>(kStepDelayUs) * 1e-6f;
    constexpr float kMinInductance = 1e-6f;
    constexpr float kMaxInductance = 0.1f;
    const float kNan = std::numeric_limits<float>::quiet_NaN();

    const float test_voltage = config_.calibration_max_voltage;
    const float vbus = power_.getVbus();

    if (vbus < 1.0f || test_voltage <= 0.0f)
        return kNan;

    float sum_delta_i = 0.0f;

    for (uint32_t i = 0; i < kCycles; i++)
    {
        iwdg_reset();
        setDuty(0.5f, 0.5f, 0.5f);
        delay_us(kRestDelayUs);

        setTestVoltage(test_voltage);

        if (!current_.waitForSample())
        {
            setDuty(0.5f, 0.5f, 0.5f);
            last_calibration_fail_reason_ = CalibrationFailReason::INDUCTANCE_SAMPLE_TIMEOUT;
            return kNan;
        }
        current_.update();
        float i_start = (current_.getIb() + current_.getIc()) * 0.5f;

        delay_us(kStepDelayUs);

        if (!current_.waitForSample())
        {
            setDuty(0.5f, 0.5f, 0.5f);
            last_calibration_fail_reason_ = CalibrationFailReason::INDUCTANCE_SAMPLE_TIMEOUT;
            return kNan;
        }
        current_.update();
        float i_end = (current_.getIb() + current_.getIc()) * 0.5f;

        setDuty(0.5f, 0.5f, 0.5f);

        sum_delta_i += std::abs(i_end - i_start);
    }

    float avg_delta_i = sum_delta_i / kCycles;
    float inductance_phase_to_phase = (test_voltage * kDt) / (avg_delta_i + 1e-6f);
    float inductance = inductance_phase_to_phase * 0.5f;

    if (isNanBits(inductance) || !(inductance >= kMinInductance && inductance <= kMaxInductance))
    {
        last_calibration_fail_reason_ = CalibrationFailReason::INDUCTANCE_OUT_OF_RANGE;
        return kNan;
    }

    return inductance;
}

void Motor::setDuty(float a, float b, float c)
{
    a = std::clamp(a, 0.05f, 0.95f);
    b = std::clamp(b, 0.05f, 0.95f);
    c = std::clamp(c, 0.05f, 0.95f);

    timer_set_oc_value(TIM1, TIM_OC1, (uint32_t)(a * period_));
    timer_set_oc_value(TIM1, TIM_OC2, (uint32_t)(b * period_));
    timer_set_oc_value(TIM1, TIM_OC3, (uint32_t)(c * period_));
}

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
