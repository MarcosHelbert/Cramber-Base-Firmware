#ifndef AXIS_H
#define AXIS_H

#include <cstdint>

#include <encoder/encoder.h>
#include <ffb/bumpstop.h>
#include <ffb/ffb_manager.h>
#include <motor/motor.h>
#include <persistence/config_store.h>

class Axis
{
public:
    enum class State
    {
        IDLE,
        CALIBRATING_ENCODER,
        HOMING,
        CALIBRATING_ANTICOGGING,
        RUNNING,
        ERROR,
    };

    enum class HomingFailReason
    {
        NONE = 0,
        NOT_READY,
        ARM_FAILED,
        INDEX_NOT_FOUND,
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
        ENCODER_DIRECTION_REVERSED,
        NOT_HOMED,
    };

    enum class AnticoggingPhase
    {
        SETTLE,
        SCAN,
        COMPUTE,
        DONE,
        FAILED,
    };

    enum class AnticoggingFailReason
    {
        NONE = 0,
        NOT_READY,
        ARM_FAILED,
        VELOCITY_NOT_REACHED,
        NOT_HOMED,
    };

public:
    Axis(Motor &motor, Encoder &encoder, FFBManager &ffb_manager);

    void initBumpstop() { bumpstop_.updateLimits(); }

    bool startCalibration();
    void stopCalibration();

    bool startHoming();
    void stopHoming();

    HomingFailReason getLastHomingFailReason() const { return last_homing_fail_reason_; }
    bool isHomingDone() const { return homing_done_; }

    bool startAnticoggingCalibration();
    void stopAnticoggingCalibration();

    AnticoggingPhase getAnticoggingPhase() const { return anticogging_phase_; }
    AnticoggingFailReason getLastAnticoggingFailReason() const { return last_anticogging_fail_reason_; }

    void setRunning(bool running)
    {
        if (running)
            iq_prev_ = 0.0f;
        state_ = running ? State::RUNNING : State::IDLE;
    }

    void update(float dt);

    float getElectricalAngle() const;
    float getIdTarget() const { return id_target_; }
    float getIqTarget() const { return iq_target_; }

    CalibrationPhase getCalibrationPhase() const { return phase_; }
    CalibrationFailReason getLastCalibrationFailReason() const { return last_fail_reason_; }

    int32_t getEncoderPositionCounts() const { return encoder_.positionCounts(); }
    // Counts the encoder moved during the last forward scan (expected ~= cpr).
    int32_t getLastScanDeltaCounts() const { return last_scan_delta_counts_; }

    State getState() const { return state_; }

    bool isPositionReferenced() const { return !g_config.use_index_z || encoder_.isIndexed(); }

private:
    float smoothstepRamp_(float elapsed_s, float duration_s) const;
    float lutCorrection_(int32_t position_counts) const;
    float anticoggingCorrection_(int32_t position_counts) const;
    uint32_t lutBucket_(int32_t position_counts) const;

private:
    Motor &motor_;
    Encoder &encoder_;
    FFBManager &ffb_manager_;
    BumpStop bumpstop_;

    State state_ = State::IDLE;
    float iq_prev_ = 0.0f;

    CalibrationPhase phase_ = CalibrationPhase::SETTLE;
    CalibrationFailReason last_fail_reason_ = CalibrationFailReason::NONE;
    float electrical_angle_ = 0.0f;
    float id_target_ = 0.0f;
    float iq_target_ = 0.0f;
    float settle_time_s_ = 0.0f;
    float settle_still_s_ = 0.0f;

    static constexpr uint32_t kLutSize = PersistedConfig::kEncoderLutSize;
    static constexpr uint32_t kMicroStepsPerSample = 40;

    int32_t scan_start_counts_ = 0;
    int32_t last_scan_delta_counts_ = 0;
    uint32_t lut_sample_index_ = 0;
    uint32_t lut_microstep_index_ = 0;

    float lut_error_fwd_[kLutSize] = {};
    float lut_error_bwd_[kLutSize] = {};
    uint32_t lut_count_fwd_[kLutSize] = {};
    uint32_t lut_count_bwd_[kLutSize] = {};

    float lut_start_mech_angle_ = 0.0f;
    float lut_offset_estimate_ = 0.0f;

    HomingFailReason last_homing_fail_reason_ = HomingFailReason::NONE;
    bool homing_done_ = false;
    int32_t homing_start_counts_ = 0;
    float homing_settle_time_s_ = 0.0f;
    float homing_electrical_angle_ = 0.0f;

    AnticoggingPhase anticogging_phase_ = AnticoggingPhase::SETTLE;
    AnticoggingFailReason last_anticogging_fail_reason_ = AnticoggingFailReason::NONE;
    float anticogging_settle_time_s_ = 0.0f;
    float anticogging_velocity_integral_ = 0.0f;
    int32_t anticogging_start_counts_ = 0;
    uint32_t anticogging_sample_index_ = 0;
    float anticogging_iq_sum_[kLutSize] = {};
    uint32_t anticogging_iq_count_[kLutSize] = {};
};

#endif // AXIS_H
