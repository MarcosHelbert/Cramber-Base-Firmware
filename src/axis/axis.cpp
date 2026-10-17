#include "axis.h"

#include <algorithm>

#include <utils.h>
#include <FastMath.h>
#include <system/timer.h>

Axis::Axis(Motor &motor, Encoder &encoder, FFBManager &ffb_manager)
    : motor_(motor), encoder_(encoder), ffb_manager_(ffb_manager) {}

float Axis::smoothstepRamp_(float elapsed_s, float duration_s) const
{
    float x = elapsed_s / duration_s;
    if (x > 1.0f)
        x = 1.0f;
    return (3.0f * x * x) - (2.0f * x * x * x);
}

bool Axis::startCalibration()
{
    if (!motor_.arm())
        return false;

    phase_ = CalibrationPhase::SETTLE;
    last_fail_reason_ = CalibrationFailReason::NONE;
    electrical_angle_ = 0.0f;
    id_target_ = 0.0f;
    settle_time_s_ = 0.0f;
    scan_start_counts_ = 0;
    lut_sample_index_ = 0;
    lut_microstep_index_ = 0;
    lut_start_mech_angle_ = 0.0f;
    lut_end_mech_angle_ = 0.0f;

    for (uint32_t i = 0; i < kLutSize; i++)
    {
        lut_error_fwd_[i] = 0.0f;
        lut_error_bwd_[i] = 0.0f;
    }

    state_ = State::CALIBRATING_ENCODER;
    return true;
}

void Axis::stopCalibration()
{
    if (phase_ == CalibrationPhase::DONE)
    {
        g_config.encoder_calibrated = true;
        g_config.encoder_lut_calibrated = true;
    }

    motor_.disarm();
    state_ = State::IDLE;
}

bool Axis::startHoming()
{
    if (!g_config.motor_calibrated || !g_config.encoder_calibrated)
    {
        last_homing_fail_reason_ = HomingFailReason::NOT_READY;
        return false;
    }

    if (!g_config.use_index_z)
    {
        g_config.mechanical_offset = encoder_.positionCounts();
        last_homing_fail_reason_ = HomingFailReason::NONE;
        homing_done_ = true;
        state_ = State::HOMING;
        return true;
    }

    if (!motor_.arm())
    {
        last_homing_fail_reason_ = HomingFailReason::ARM_FAILED;
        return false;
    }

    last_homing_fail_reason_ = HomingFailReason::NONE;
    homing_done_ = false;
    id_target_ = 0.0f;
    iq_target_ = 0.0f;
    homing_start_counts_ = encoder_.positionCounts();
    homing_settle_time_s_ = 0.0f;
    homing_electrical_angle_ = 0.0f;

    encoder_.setIdxSubscribe(true);

    state_ = State::HOMING;
    return true;
}

void Axis::stopHoming()
{
    encoder_.setIdxSubscribe(false);
    motor_.disarm();
    iq_target_ = 0.0f;
    state_ = State::IDLE;
}

void Axis::update(float dt)
{
    if (state_ == State::RUNNING)
    {
        float max_step = g_config.ffb_slew_rate_a_per_s * dt;

        float position_rad = encoder_.angle();
        float velocity = encoder_.velocity();
        float acceleration = encoder_.acceleration();

        float torque = ffb_manager_.getTorque(position_rad, velocity, acceleration, millis());
        torque += bumpstop_.calculate(position_rad, velocity);

        torque = std::clamp(torque, -g_config.max_current, g_config.max_current);

        float delta = torque - iq_prev_;
        delta = std::clamp(delta, -max_step, max_step);
        iq_prev_ += delta;

        id_target_ = 0.0f;
        iq_target_ = iq_prev_;
        return;
    }

    if (state_ == State::HOMING)
    {
        constexpr float kSettleTimeS = 0.05f;
        constexpr float kScanOmegaRadPerS = 4.0f * (TWO_PI / 2.0f);
        constexpr int32_t kDirectionDeadband = 8;

        if (encoder_.isIndexed())
        {
            iq_target_ = 0.0f;
            encoder_.setIdxSubscribe(false);
            homing_done_ = true;
            return;
        }

        homing_settle_time_s_ += dt;
        if (homing_settle_time_s_ < kSettleTimeS)
        {
            float smooth = smoothstepRamp_(homing_settle_time_s_, kSettleTimeS);
            homing_electrical_angle_ = 0.0f;
            id_target_ = 0.0f;
            iq_target_ = g_config.calibration_test_current * smooth;
            return;
        }

        homing_electrical_angle_ += kScanOmegaRadPerS * dt;
        id_target_ = 0.0f;
        iq_target_ = g_config.calibration_test_current;

        int32_t delta = encoder_.positionCounts() - homing_start_counts_;
        if (delta > static_cast<int32_t>(g_config.encoder_cpr) + kDirectionDeadband ||
            delta < -(static_cast<int32_t>(g_config.encoder_cpr) + kDirectionDeadband))
        {
            iq_target_ = 0.0f;
            encoder_.setIdxSubscribe(false);
            last_homing_fail_reason_ = HomingFailReason::INDEX_NOT_FOUND;
            return;
        }

        return;
    }

    constexpr float kSettleTimeS = 0.05f;
    const float pole_pairs = static_cast<float>(g_config.motor_pole_pairs);
    constexpr float kCalibrationRange = 0.05f;
    constexpr int32_t kDirectionDeadband = 8;
    const float kTotalSweepRad = TWO_PI * pole_pairs;
    const float kStepRad = kTotalSweepRad / static_cast<float>(kLutSize * kMicroStepsPerSample);

    int32_t position_counts = encoder_.positionCounts();

    switch (phase_)
    {
    case CalibrationPhase::SETTLE:
    {
        settle_time_s_ += dt;
        float smooth = smoothstepRamp_(settle_time_s_, kSettleTimeS);
        electrical_angle_ = 0.0f;
        id_target_ = g_config.calibration_test_current * smooth;

        if (settle_time_s_ >= kSettleTimeS)
        {
            scan_start_counts_ = position_counts;
            lut_start_mech_angle_ = encoder_.angle();
            lut_sample_index_ = 0;
            lut_microstep_index_ = 0;
            phase_ = CalibrationPhase::SCAN_FWD;
        }
        break;
    }

    case CalibrationPhase::SCAN_FWD:
    {
        electrical_angle_ += kStepRad;
        id_target_ = g_config.calibration_test_current;
        lut_microstep_index_++;

        if (lut_microstep_index_ >= kMicroStepsPerSample)
        {
            lut_microstep_index_ = 0;
            float theta_actual = encoder_.angle() * pole_pairs;
            float error = normalizeAngle(electrical_angle_ - theta_actual);
            lut_error_fwd_[lut_sample_index_] = error;
            lut_sample_index_++;

            if (lut_sample_index_ >= kLutSize)
            {
                int32_t delta = position_counts - scan_start_counts_;

                if (delta > kDirectionDeadband || delta < -kDirectionDeadband)
                {
                    float elec_rad_per_count = (pole_pairs * TWO_PI) / static_cast<float>(g_config.encoder_cpr);
                    float expected_count_delta = kTotalSweepRad / elec_rad_per_count;
                    float actual = std::abs(static_cast<float>(delta));

                    if (std::abs(actual - expected_count_delta) / expected_count_delta > kCalibrationRange)
                    {
                        phase_ = CalibrationPhase::FAILED;
                        last_fail_reason_ = CalibrationFailReason::CPR_POLE_PAIRS_MISMATCH;
                        break;
                    }
                }
                else
                {
                    phase_ = CalibrationPhase::FAILED;
                    last_fail_reason_ = CalibrationFailReason::ENCODER_DIRECTION_UNCLEAR;
                    break;
                }

                lut_sample_index_ = 0;
                phase_ = CalibrationPhase::SCAN_REV;
            }
        }
        break;
    }

    case CalibrationPhase::SCAN_REV:
    {
        electrical_angle_ -= kStepRad;
        id_target_ = g_config.calibration_test_current;
        lut_microstep_index_++;

        if (lut_microstep_index_ >= kMicroStepsPerSample)
        {
            lut_microstep_index_ = 0;
            float theta_actual = encoder_.angle() * pole_pairs;
            float error = normalizeAngle(electrical_angle_ - theta_actual);
            lut_error_bwd_[lut_sample_index_] = error;
            lut_sample_index_++;

            if (lut_sample_index_ >= kLutSize)
            {
                lut_end_mech_angle_ = encoder_.angle();
                id_target_ = 0.0f;
                phase_ = CalibrationPhase::COMPUTE;
            }
        }
        break;
    }

    case CalibrationPhase::COMPUTE:
    {
        float error_avg[kLutSize];
        for (uint32_t i = 0; i < kLutSize; i++)
        {
            error_avg[i] = (lut_error_fwd_[i] + lut_error_bwd_[kLutSize - 1 - i]) * 0.5f;
        }

        constexpr uint32_t kFilterHalfWindow = 2;
        float error_filtered[kLutSize];
        for (uint32_t i = 0; i < kLutSize; i++)
        {
            float sum = 0.0f;
            for (int32_t k = -static_cast<int32_t>(kFilterHalfWindow); k <= static_cast<int32_t>(kFilterHalfWindow); k++)
            {
                int32_t idx = (static_cast<int32_t>(i) + k + static_cast<int32_t>(kLutSize)) % static_cast<int32_t>(kLutSize);
                sum += error_avg[idx];
            }
            error_filtered[i] = sum / static_cast<float>(2 * kFilterHalfWindow + 1);
        }

        float mean_error = 0.0f;
        for (uint32_t i = 0; i < kLutSize; i++)
        {
            mean_error += error_filtered[i];
        }
        mean_error /= static_cast<float>(kLutSize);

        for (uint32_t i = 0; i < kLutSize; i++)
        {
            g_config.encoder_lut[i] = error_filtered[i] - mean_error;
        }

        float raw_offset_mech = (lut_start_mech_angle_ + lut_end_mech_angle_) * 0.5f;
        g_config.encoder_offset_rad = normalizeAngle2Pi((raw_offset_mech * pole_pairs) + mean_error);

        id_target_ = 0.0f;
        phase_ = CalibrationPhase::DONE;
        break;
    }

    case CalibrationPhase::DONE:
    case CalibrationPhase::FAILED:
        id_target_ = 0.0f;
        break;
    }
}

float Axis::getElectricalAngle() const
{
    if (state_ == State::CALIBRATING_ENCODER)
    {
        return electrical_angle_;
    }

    if (state_ == State::HOMING)
    {
        return normalizeAngle2Pi(homing_electrical_angle_);
    }

    float mechanical_angle = encoder_.angle();
    float electrical_angle = (mechanical_angle * static_cast<float>(g_config.motor_pole_pairs)) - g_config.encoder_offset_rad;

    if (g_config.encoder_lut_calibrated)
    {
        electrical_angle += lutCorrection_(encoder_.positionCounts());
    }

    return normalizeAngle2Pi(electrical_angle);
}

float Axis::lutCorrection_(int32_t position_counts) const
{
    const uint32_t cpr = g_config.encoder_cpr;
    uint32_t wrapped = static_cast<uint32_t>(((position_counts % static_cast<int32_t>(cpr)) + static_cast<int32_t>(cpr)) % static_cast<int32_t>(cpr));

    float index_f = (static_cast<float>(wrapped) / static_cast<float>(cpr)) * static_cast<float>(kLutSize);
    uint32_t i0 = static_cast<uint32_t>(index_f) % kLutSize;
    uint32_t i1 = (i0 + 1) % kLutSize;
    float frac = index_f - static_cast<float>(static_cast<uint32_t>(index_f));

    return g_config.encoder_lut[i0] + (g_config.encoder_lut[i1] - g_config.encoder_lut[i0]) * frac;
}
