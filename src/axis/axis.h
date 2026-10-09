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
        ALIGNING,
        LUT_CALIBRATING,
        ERROR,
    };

    enum class AlignmentPhase
    {
        SETTLE,
        SCAN_FWD,
        SCAN_REV,
        DONE,
        FAILED,
    };

    enum class AlignmentFailReason
    {
        NONE = 0,
        ENCODER_DIRECTION_UNCLEAR,
        CPR_POLE_PAIRS_MISMATCH,
    };

    enum class LutCalibrationPhase
    {
        SETTLE,
        SCAN_FWD,
        SCAN_REV,
        COMPUTE,
        DONE,
        FAILED,
    };

    enum class LutCalibrationFailReason
    {
        NONE = 0,
        NOT_ALIGNED,
        ARM_FAILED,
    };

public:
    Axis(Motor &motor, Encoder &encoder);

    bool startAlignment();

    void update(float dt);

    float getElectricalAngle() const;
    float getIdTarget() const { return id_target_; }
    float getIqTarget() const { return iq_target_; }

    AlignmentPhase getAlignmentPhase() const { return alignment_phase_; }
    AlignmentFailReason getLastAlignmentFailReason() const { return last_alignment_fail_reason_; }

    int32_t getEncoderPositionCounts() const { return encoder_.positionCounts(); }

    void stopAlignment();

    bool startLutCalibration();
    void stopLutCalibration();
    void updateLutCalibration(float dt);
    LutCalibrationPhase getLutCalibrationPhase() const { return lut_phase_; }
    LutCalibrationFailReason getLastLutCalibrationFailReason() const { return last_lut_fail_reason_; }

    State getState() const { return state_; }

private:
    float smoothstepRamp_(float elapsed_s, float duration_s) const;
    float lutCorrection_(int32_t position_counts) const;

private:
    Motor &motor_;
    Encoder &encoder_;

    State state_ = State::IDLE;

    AlignmentPhase alignment_phase_ = AlignmentPhase::SETTLE;
    AlignmentFailReason last_alignment_fail_reason_ = AlignmentFailReason::NONE;
    float alignment_electrical_angle_ = 0.0f;
    float id_target_ = 0.0f;
    float iq_target_ = 0.0f;
    float alignment_settle_time_s_ = 0.0f;
    float alignment_scan_distance_ = 0.0f;
    int32_t alignment_start_counts_ = 0;
    int64_t alignment_count_sum_ = 0;
    uint32_t alignment_step_count_ = 0;

    static constexpr uint32_t kLutSize = PersistedConfig::kEncoderLutSize;
    static constexpr uint32_t kMicroStepsPerSample = 40;

    LutCalibrationPhase lut_phase_ = LutCalibrationPhase::SETTLE;
    LutCalibrationFailReason last_lut_fail_reason_ = LutCalibrationFailReason::NONE;

    float lut_settle_time_s_ = 0.0f;
    float lut_electrical_angle_ = 0.0f;
    uint32_t lut_sample_index_ = 0;
    uint32_t lut_microstep_index_ = 0;

    float lut_error_fwd_[kLutSize] = {};
    float lut_error_bwd_[kLutSize] = {};

    float lut_start_mech_angle_ = 0.0f;
    float lut_end_mech_angle_ = 0.0f;
};

#endif // AXIS_H