#include <libopencm3/cm3/scb.h>
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
#include <motor/startup_sound.h>

#include <persistence/config_store.h>
#include <persistence/user_prefs_store.h>

#include <power/brake_resistor.h>
#include <power/current_sensor.h>
#include <power/power_monitor.h>

#include <protocol/config_protocol.h>

#include <telemetry/telemetry.h>

#include <system/timer.h>

#include <usb/usb.h>

#include <boot_layout.h>

ConfigStore config_store;
UserPrefsStore user_prefs;

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
StartupSound startup_sound = StartupSound(g_config);
Telemetry telemetry;

ConfigProtocol config_protocol = ConfigProtocol(motor, foc, drv8301, axis, config_store, ffb_manager, startup_sound, user_prefs, telemetry);

USB usb;

static void watchdog_init_()
{
    iwdg_set_period_ms(150);
    iwdg_start();
    iwdg_reset();
}

static TaskHandle_t usb_task_handle = nullptr;

extern "C" void tud_event_hook_cb(uint8_t rhport, uint32_t eventid, bool in_isr)
{
    (void)rhport;
    (void)eventid;
    if (usb_task_handle == nullptr)
        return;
    if (in_isr)
    {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(usb_task_handle, &woken);
        portYIELD_FROM_ISR(woken);
    }
    else
    {
        xTaskNotifyGive(usb_task_handle);
    }
}

static void usb_task(void *arg)
{
    (void)arg;

    usb_task_handle = xTaskGetCurrentTaskHandle();
    usb.start();

    const TickType_t period = pdMS_TO_TICKS(1); // 1 kHz
    TickType_t next = xTaskGetTickCount();

    while (1)
    {
        usb.handleEvents();

        const TickType_t now = xTaskGetTickCount();
        if (static_cast<int32_t>(now - next) >= 0)
        {
            next = now + period;

            iwdg_reset();
            if (axis.getState() == Axis::State::RUNNING)
            {
                usb.setAxis(encoder.angle(), g_config.steering_range_deg);
            }
            else
            {
                usb.setAxis(0.0f, g_config.steering_range_deg);
            }
            usb.sendReports();
            ffb_manager.tickStats(millis());
            config_protocol.poll();
            telemetry.poll(millis());
        }

        const int32_t wait = static_cast<int32_t>(next - xTaskGetTickCount());
        ulTaskNotifyTake(pdTRUE, wait > 0 ? static_cast<TickType_t>(wait) : 0);
    }
}

static void play_startup_sound_()
{
    if (!g_config.startup_sound_enabled || !g_config.motor_calibrated)
        return;

    vTaskDelay(pdMS_TO_TICKS(300));
    iwdg_reset();

    if (!motor.arm())
        return;

    if (startup_sound.start())
    {
        constexpr uint32_t kTimeoutMs = 3000;
        uint32_t waited_ms = 0;
        while (startup_sound.isPlaying() && waited_ms < kTimeoutMs)
        {
            iwdg_reset();
            vTaskDelay(pdMS_TO_TICKS(10));
            waited_ms += 10;
        }
        startup_sound.stop();
    }

    motor.disarm();
}

static void axis_task(void *arg)
{
    (void)arg;
    play_startup_sound_();

    if (g_config.home_on_boot || (g_config.ffb_enabled_on_boot && g_config.use_index_z))
    {
        if (axis.startHoming())
        {
            TickType_t lastWakeTime = xTaskGetTickCount();
            const TickType_t period = pdMS_TO_TICKS(1); // 1 kHz
            uint32_t waited_ms = 0;
            constexpr uint32_t kHomingTimeoutMs = 30000;

            while (!axis.isHomingDone() &&
                   axis.getLastHomingFailReason() == Axis::HomingFailReason::NONE &&
                   waited_ms < kHomingTimeoutMs)
            {
                iwdg_reset();
                axis.update(1.0f / 1000.0f);
                vTaskDelayUntil(&lastWakeTime, period);
                waited_ms += 1;
            }

            axis.stopHoming();
        }
    }

    if (g_config.ffb_enabled_on_boot && g_config.motor_calibrated && g_config.encoder_calibrated &&
        axis.isPositionReferenced())
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

__attribute__((section(".app_header"), used)) static const AppHeader kAppHeader = {
    APP_HEADER_MAGIC, 0, 0, kFirmwareVersion};

int main()
{
    SCB_VTOR = APP_VECTOR_ADDR;

    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    dwt_init();
    watchdog_init_();
    config_store.load();
    user_prefs.load(g_config);
    if (!g_config.use_index_z)
    {
        g_config.encoder_calibrated = false;
        g_config.encoder_lut_calibrated = false;
        g_config.anticogging_calibrated = false;
    }

    foc.updateGainsFromConfig();
    axis.initBumpstop();
    config_protocol.init();
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

    if (telemetry.due())
    {
        const float values[TLM_COUNT] = {
            current_sensor.getIa(),
            current_sensor.getIb(),
            current_sensor.getIc(),
            foc.getId(),
            foc.getIq(),
            axis.getIqTarget(),
            foc.getVd(),
            foc.getVq(),
            vbus,
            previous_ibus,
            axis.getElectricalAngle(),
            encoder.angle() * (180.0f / 3.14159265f),
            encoder.velocity(),
        };
        telemetry.push(values);
    }

    Motor::MotorState motor_state = motor.getState();
    Axis::State axis_state = axis.getState();
    bool driving_open_loop = axis_state == Axis::State::CALIBRATING_ENCODER || axis_state == Axis::State::HOMING;
    bool calibrated = g_config.motor_calibrated && g_config.encoder_calibrated;

    if (motor_state == Motor::MotorState::ARMED && startup_sound.isPlaying())
    {
        startup_sound.notePhaseCurrent(current_sensor.getIa());
        float v = startup_sound.update(vbus);
        motor.setOutput(v, -0.5f * v, -0.5f * v);
        previous_ibus = 0.0f;
        return;
    }

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