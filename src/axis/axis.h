#ifndef AXIS_H
#define AXIS_H

#include <cstdint>

#include <encoder/encoder.h>
#include <motor/motor.h>
#include <persistence/config_store.h>

class Axis
{
public:
    enum class State
    {
        IDLE,
        CALIBRATING_ENCODER,
        ERROR,
    };

    enum class CalibrationPhase
    {
        SETTLE,
        SCAN_FWD,
        SCAN_REV,
        COMPUTE,
        DONE,
        FAILED,
    };

    enum class CalibrationFailReason
    {
        NONE = 0,
        ENCODER_DIRECTION_UNCLEAR,
        CPR_POLE_PAIRS_MISMATCH,
    };

public:
    Axis(Motor &motor, Encoder &encoder);

    bool startCalibration();
    void stopCalibration();

    void update(float dt);

    float getElectricalAngle() const;
    float getIdTarget() const { return id_target_; }
    float getIqTarget() const { return iq_target_; }

    CalibrationPhase getCalibrationPhase() const { return phase_; }
    CalibrationFailReason getLastCalibrationFailReason() const { return last_fail_reason_; }

    int32_t getEncoderPositionCounts() const { return encoder_.positionCounts(); }

    State getState() const { return state_; }

private:
    float smoothstepRamp_(float elapsed_s, float duration_s) const;
    float lutCorrection_(int32_t position_counts) const;

private:
    Motor &motor_;
    Encoder &encoder_;

    State state_ = State::IDLE;

    CalibrationPhase phase_ = CalibrationPhase::SETTLE;
    CalibrationFailReason last_fail_reason_ = CalibrationFailReason::NONE;
    float electrical_angle_ = 0.0f;
    float id_target_ = 0.0f;
    float iq_target_ = 0.0f;
    float settle_time_s_ = 0.0f;

    static constexpr uint32_t kLutSize = PersistedConfig::kEncoderLutSize;
    static constexpr uint32_t kMicroStepsPerSample = 40;

    int32_t scan_start_counts_ = 0;
    uint32_t lut_sample_index_ = 0;
    uint32_t lut_microstep_index_ = 0;

    float lut_error_fwd_[kLutSize] = {};
    float lut_error_bwd_[kLutSize] = {};

    float lut_start_mech_angle_ = 0.0f;
    float lut_end_mech_angle_ = 0.0f;
};

#endif // AXIS_H
