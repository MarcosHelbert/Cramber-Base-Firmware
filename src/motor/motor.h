#ifndef MOTOR_H
#define MOTOR_H

#include <cstdint>

#include <persistence/config_store.h>
#include <power/power_monitor.h>
#include <power/current_sensor.h>
#include <driver/drv8301.h>

class Motor
{
public:
    enum class MotorState
    {
        DISARMED = 0,
        ARMED,
        ERROR,
    };

    enum class ArmFailReason
    {
        NONE = 0,
        POWER_OUT_OF_RANGE,
        DRIVER_FAULT,
    };

    enum class CalibrationFailReason
    {
        NONE = 0,
        RESISTANCE_SAMPLE_TIMEOUT,
        RESISTANCE_OUT_OF_RANGE,
        INDUCTANCE_SAMPLE_TIMEOUT,
        INDUCTANCE_OUT_OF_RANGE,
    };

    Motor(const PersistedConfig &config, const PowerMonitor &power, CurrentSensor &current, DRV8301 &driver);

    void init();

    bool arm();
    void disarm();

    void clearFault();

    void setOutput(float va, float vb, float vc);

    bool calibrate();
    bool isCalibrated() const { return is_calibrated_; }

    MotorState getState() const { return state_; }
    ArmFailReason getLastArmFailReason() const { return last_arm_fail_reason_; }
    CalibrationFailReason getLastCalibrationFailReason() const { return last_calibration_fail_reason_; }

private:
    float measurePhaseResistance();
    float measurePhaseInductance();
    void setTestVoltage(float test_voltage);
    void setDuty(float a, float b, float c);
    uint8_t encodeDeadtime(uint32_t cycles);

private:
    const PersistedConfig &config_;
    const PowerMonitor &power_;
    CurrentSensor &current_;
    DRV8301 &driver_;

    bool is_calibrated_ = false;
    MotorState state_ = MotorState::DISARMED;
    ArmFailReason last_arm_fail_reason_ = ArmFailReason::NONE;
    CalibrationFailReason last_calibration_fail_reason_ = CalibrationFailReason::NONE;

    uint32_t period_ = 0;
};

#endif // MOTOR_H
