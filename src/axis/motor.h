#ifndef MOTOR_H
#define MOTOR_H

#include <cstdint>

#include "encoder.h"
#include <driver/drv8301.h>
#include <power/power_monitor.h>
#include <power/current_sensor.h>

class Motor
{
public:
    enum class MotorState
    {
        DISABLED = 0,
        IDLE,
        CALIBRATING,
        RUNNING,
        ERROR
    };

    Motor(PowerMonitor &power, CurrentSensor &current, DRV8301 &driver, Encoder &encoder);

    void init();

    bool arm();
    void disarm();

    void start();
    void stop();

    MotorState getState() const { return state_; };

    void setOutput(float va, float vb, float vc);

    bool measurePhaseResistance();
    bool measurePhaseInductance();

    float getElectricalAngle() const;

    float getPhaseInductance() const { return phase_inductance_; }
    float getPhaseResistance() const { return phase_resistance_; }

private:
    DRV8301 &driver_;
    PowerMonitor &power_;
    CurrentSensor &current_;
    Encoder &encoder_;

    MotorState state_ = MotorState::DISABLED;

    uint32_t period_ = 0;
    float pwm_period_s_ = 0.0f;

    float phase_inductance_ = 0.0f;
    float phase_resistance_ = 0.0f;

    void setTestVoltage();

    void setDuty(float a, float b, float c);
    uint8_t encodeDeadtime(uint32_t cycles);
};

#endif // MOTOR_H