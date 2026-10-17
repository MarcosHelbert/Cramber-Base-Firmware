#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

#include <axis/axis.h>

#include <driver/drv8301.h>

#include <encoder/encoder.h>

#include <ffb/ffb_manager.h>

#include <hardware/adc.h>

#include <motor/foc.h>
#include <motor/motor.h>

#include <persistence/config_store.h>

#include <power/brake_resistor.h>
#include <power/current_sensor.h>
#include <power/power_monitor.h>

#include <protocol/config_protocol.h>

#include <system/timer.h>

#include <usb/usb.h>

ConfigStore config_store;

DRV8301 drv8301 = DRV8301(g_config);
Encoder encoder = Encoder(g_config);

FFBManager ffb_manager = FFBManager(g_config);

ADC adc;

BrakeResistor brake_resistor = BrakeResistor(g_config);
CurrentSensor current_sensor = CurrentSensor(g_config);
PowerMonitor power_monitor = PowerMonitor(g_config);

FOC foc = FOC(g_config, g_config.pwm_freq_hz);
Motor motor = Motor(g_config, power_monitor, current_sensor, drv8301);
Axis axis = Axis(motor, encoder, ffb_manager);

ConfigProtocol config_protocol = ConfigProtocol(motor, foc, drv8301, axis, config_store, ffb_manager);

USB usb;

static void watchdog_init_()
{
    iwdg_set_period_ms(150);
    iwdg_start();
    iwdg_reset();
}

static void usb_task(void *arg)
{
    (void)arg;

    usb.start();

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1); // 1 kHz

    while (1)
    {
        iwdg_reset();
        if (axis.getState() == Axis::State::RUNNING)
        {
            usb.setAxis(encoder.angle(), g_config.steering_range_deg);
        }
        usb.update();
        config_protocol.poll();

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

static void axis_task(void *arg)
{
    (void)arg;
    if (g_config.home_on_boot)
    {
        if (axis.startHoming())
        {
            TickType_t lastWakeTime = xTaskGetTickCount();
            const TickType_t period = pdMS_TO_TICKS(1); // 1 kHz
            uint32_t waited_ms = 0;
            constexpr uint32_t kHomingTimeoutMs = 30000;

            while (axis.getState() == Axis::State::HOMING && waited_ms < kHomingTimeoutMs)
            {
                iwdg_reset();
                axis.update(1.0f / 1000.0f);
                vTaskDelayUntil(&lastWakeTime, period);
                waited_ms += 1;
            }

            axis.stopHoming();
        }
    }

    if (g_config.ffb_enabled_on_boot && g_config.motor_calibrated && g_config.encoder_calibrated)
    {
        if (motor.arm())
        {
            axis.setRunning(true);
            ffb_manager.startFFB();
        }
    }

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1); // 1 kHz

    while (1)
    {
        iwdg_reset();
        const float dt = 1.0f / 1000.0f; // 1 kHz
        axis.update(dt);

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

static void fault_task(void *arg)
{
    (void)arg;

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5); // 200 Hz

    while (1)
    {
        iwdg_reset();

        power_monitor.update();

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

int main()
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    dwt_init();
    watchdog_init_();
    config_store.load();
    axis.initBumpstop();
    adc.init();
    current_sensor.init();
    power_monitor.init();
    encoder.init();
    brake_resistor.init();
    brake_resistor.arm();

    usb.init();
    drv8301.init();
    motor.init();

    xTaskCreate(usb_task, "usb", 1024, NULL, 7, NULL);
    xTaskCreate(axis_task, "axis", 2048, NULL, 5, NULL);
    xTaskCreate(fault_task, "fault", 384, NULL, 1, NULL);

    vTaskStartScheduler();

    return 0;
}

extern "C" void tim1_cc_isr(void)
{
    if (!timer_get_flag(TIM1, TIM_SR_CC4IF))
        return;

    timer_clear_flag(TIM1, TIM_SR_CC4IF);

    float vbus = power_monitor.getVbus();

    current_sensor.update();
    encoder.update();

    static float previous_ibus = 0.0f;
    if (g_config.brake_resistor_enabled)
    {
        brake_resistor.update(vbus, previous_ibus);
    }

    Motor::MotorState motor_state = motor.getState();
    bool driving_open_loop = axis.getState() == Axis::State::CALIBRATING_ENCODER;
    bool calibrated = g_config.motor_calibrated && g_config.encoder_calibrated;

    if (motor_state != Motor::MotorState::ARMED || (!driving_open_loop && !calibrated))
    {
        return;
    }

    float ia = current_sensor.getIa();
    float ib = current_sensor.getIb();
    float ic = current_sensor.getIc();

    foc.setTargets(axis.getIdTarget(), axis.getIqTarget());

    float angle = axis.getElectricalAngle();
    float velocity = encoder.velocity();
    float omega_e = driving_open_loop ? 0.0f : velocity * g_config.motor_pole_pairs;

    foc.update(ia, ib, ic, angle, omega_e, g_config.phase_inductance, g_config.motor_flux_linkage, vbus);

    motor.setOutput(foc.getVa(), foc.getVb(), foc.getVc());

    previous_ibus = (foc.getModD(vbus) * foc.getId()) + (foc.getModQ(vbus) * foc.getIq());
}