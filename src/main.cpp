#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/iwdg.h>

#include <FreeRTOS.h>
#include <task.h>

#include <axis/axis.h>
#include <axis/encoder.h>
#include <axis/motor.h>

#include <communication/can.h>

#include <driver/drv8301.h>

#include <ffb/bumpstop.h>
#include <ffb/ffb_manager.h>

#include <hardware/adc.h>
#include <hardware/dwt.h>
#include <hardware/timer.h>

#include <power/brake_resistor.h>
#include <power/current_sensor.h>
#include <power/power_monitor.h>

#include <communication/usb.h>

#include <system_state.h>
#include <config.h>
#include <error.h>
#include <debug.h>

CurrentSensor current_sensor;
PowerMonitor power_monitor;
BrakingResistor braking_resistor = BrakingResistor(BRAKE_RESISTOR_RESISTANCE, BRAKE_RESISTOR_POWER_RATING, MAX_REGENERATIVE_CURRENT, MOTOR_PWM_FREQ_HZ);

ADC adc;
USB usb;
CAN can;

DRV8301 drv8301;

BumpStop bumpstop;
FFBManager ffb_manager;

Anticogging anticogging;
Encoder encoder = Encoder(ENCODER_CPR, ENCODER_REVERSE, MOTOR_PWM_FREQ_HZ);
Motor motor = Motor(power_monitor, current_sensor, drv8301, encoder);
FOC foc = FOC(MOTOR_PWM_FREQ_HZ);
Axis axis = Axis(1000, foc, encoder, bumpstop, motor, anticogging, ffb_manager);

static void watchdog_init_()
{
    iwdg_set_period_ms(IWDG_TIMEOUT_MS);
    iwdg_start();
    iwdg_reset();
}

static void usb_task(void *arg)
{
    (void)arg;

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1); // 1kHz

    while (1)
    {
        iwdg_reset();
        usb.update();

        if (axis.getState() == Axis::State::READY)
            usb.updateAxis(encoder.position());

        vTaskDelayUntil(&lastWakeTime, period);
    }
};

static void axis_task(void *arg)
{
    (void)arg;

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1); // 1kHz

    while (1)
    {
        iwdg_reset();
        axis.update();

        vTaskDelayUntil(&lastWakeTime, period);
    }
};

static void fault_task(void *arg)
{
    (void)arg;

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5); // 200 Hz

    bool last_power_ok = true;
    Error last_drv_err = Error::ERROR_NONE;

    uint32_t last_recovery_time = 0;
    uint32_t recovery_count = 0;

    while (1)
    {
        iwdg_reset();

        power_monitor.update();
        bool power_ok = power_monitor.isPowerOk();
        if (power_ok != last_power_ok)
        {
            cdcLog("[FAULT] power_ok=%d vbus=%.2f\r\n", power_ok, power_monitor.getVbus());
            last_power_ok = power_ok;
        }

        if (!power_ok)
        {
            axis.setError(Error::ERROR_VBUS_OUT_OF_RANGE);
            motor.disarm();
        }

        Error drv_err = drv8301.updateFault();
        if (drv_err != last_drv_err)
        {
            int32_t vbus_mv = static_cast<int32_t>(power_monitor.getVbus() * 1000.0f);
            int32_t ia_ma = static_cast<int32_t>(current_sensor.getIa() * 1000.0f);
            int32_t ib_ma = static_cast<int32_t>(current_sensor.getIb() * 1000.0f);
            int32_t ic_ma = static_cast<int32_t>(current_sensor.getIc() * 1000.0f);

            cdcLog(
                "[FAULT] drv_err=%d vbus=%ldmV ia=%ldmA ib=%ldmA ic=%ldmA s1=0x%03X s2=0x%03X\r\n",
                (int)drv_err,
                (long)vbus_mv,
                (long)ia_ma,
                (long)ib_ma,
                (long)ic_ma,
                drv8301.getStatus1(),
                drv8301.getStatus2());
            last_drv_err = drv_err;
        }

        if (drv_err != Error::ERROR_NONE)
        {
            uint32_t now = xTaskGetTickCount();
            if (now - last_recovery_time > pdMS_TO_TICKS(1000))
            {
                recovery_count = 0;
            }

            if (recovery_count < 3)
            {
                cdcLog("[FAULT] Attempting auto-recovery (%lu/3)...\r\n", recovery_count + 1);
                axis.setError(drv_err);
                motor.disarm();

                vTaskDelay(pdMS_TO_TICKS(10));

                if (motor.arm())
                {
                    motor.start();
                    axis.clearError();
                    last_recovery_time = now;
                    recovery_count++;
                    cdcLog("[FAULT] Auto-recovery successful.\r\n");
                }
                else
                {
                    axis.setError(drv_err);
                }
            }
            else
            {
                axis.setError(drv_err);
                motor.disarm();
            }
        }

        vTaskDelayUntil(&lastWakeTime, period);
    }
};

int16_t fail_count_ = 0;

float encoder_vel = 0;

int main()
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    dwt_init();
    watchdog_init_();
    adc.init();
    current_sensor.init();
    power_monitor.init();
    encoder.init();
    braking_resistor.init();

    usb.init();
    can.init();
    drv8301.init();
    motor.init();
    bumpstop.init();
    axis.init();

    while (1)
    {
        iwdg_reset();

        if (motor.getPhaseResistance() == 0.0f)
        {
            if (!motor.measurePhaseResistance())
            {
                fail_count_++;
                if (fail_count_ >= 3)
                    return 0;
            }
        }
        else if (motor.getPhaseInductance() == 0.0f)
        {
            if (!motor.measurePhaseInductance())
            {
                fail_count_++;
                if (fail_count_ >= 3)
                    return 0;
            }
            else
            {
                float bw = CURRENT_BANDWIDTH_RAD_S;
                float Kp = motor.getPhaseInductance() * bw;
                float Ki = motor.getPhaseResistance() * bw;

                g_state.phase_inductance_ = motor.getPhaseInductance();
                g_state.phase_resistance_ = motor.getPhaseResistance();

                foc.setGains(Kp, Ki);
                foc.setTargets(0.0f, 0.0f);
                foc.reset();

                break;
            }
        }

        delay_ms(100);
    }

    if (motor.getPhaseResistance() != 0.0f && motor.getPhaseInductance() != 0.0f)
    {
        motor.start();
        axis.start();
        braking_resistor.arm();

        xTaskCreate(usb_task, "usb", 1024, NULL, 7, NULL);
        xTaskCreate(axis_task, "axis", 2048, NULL, 5, NULL);
        xTaskCreate(fault_task, "fault", 384, NULL, 1, NULL);

        vTaskStartScheduler();
    }

    return 0;
}

extern "C" void tim1_cc_isr(void)
{
    if (!timer_get_flag(TIM1, TIM_SR_CC4IF))
        return;

    const uint32_t start_cycles = dwt_get_cycles();

    timer_clear_flag(TIM1, TIM_SR_CC4IF);

    float vbus = power_monitor.getVbus();

    current_sensor.update();
    encoder.update();

    static float previous_ibus = 0.0f;

    braking_resistor.update(vbus, previous_ibus);

    if (motor.getState() != Motor::MotorState::RUNNING)
        return;

    float ia = current_sensor.getIa();
    float ib = current_sensor.getIb();
    float ic = current_sensor.getIc();

    float angle = axis.getElectricalAngle();
    float velocity = encoder.velocity();
    float omega_e = velocity * MOTOR_POLE_PAIRS;

    foc.update(
        ia,
        ib,
        ic,
        angle,
        omega_e,
        g_state.phase_inductance_,
        MOTOR_FLUX_LINKAGE,
        vbus);

    motor.setOutput(foc.getVa(), foc.getVb(), foc.getVc());

    previous_ibus = (foc.getModD(vbus) * foc.getId()) + (foc.getModQ(vbus) * foc.getIq());

    // const uint32_t isr_cycles = dwt_get_cycles() - start_cycles;
}