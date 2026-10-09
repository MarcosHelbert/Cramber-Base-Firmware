#include "axis.h"

#include <utils.h>
#include <FastMath.h>

Axis::Axis(Motor &motor, Encoder &encoder)
    : motor_(motor), encoder_(encoder) {}

float Axis::smoothstepRamp_(float elapsed_s, float duration_s) const
{
    float x = elapsed_s / duration_s;
    if (x > 1.0f)
        x = 1.0f;
    return (3.0f * x * x) - (2.0f * x * x * x);
}

bool Axis::startAlignment()
{
    if (!motor_.arm())
        return false;

    alignment_phase_ = AlignmentPhase::SETTLE;
    last_alignment_fail_reason_ = AlignmentFailReason::NONE;
    alignment_electrical_angle_ = 0.0f;
    id_target_ = 0.0f;
    alignment_settle_time_s_ = 0.0f;
    alignment_scan_distance_ = 0.0f;
    alignment_start_counts_ = 0;
    alignment_count_sum_ = 0;
    alignment_step_count_ = 0;

    state_ = State::ALIGNING;
    return true;
}

void Axis::stopAlignment()
{
    if (alignment_phase_ == AlignmentPhase::DONE)
    {
        g_config.encoder_offset_rad = alignment_electrical_angle_;
        g_config.encoder_calibrated = true;
    }

    motor_.disarm();
    state_ = State::IDLE;
}

void Axis::update(float dt)
{
    if (state_ == State::LUT_CALIBRATING)
    {
        updateLutCalibration(dt);
        return;
    }

    constexpr float kSettleTimeS = 0.05f;
    constexpr float kPi = TWO_PI / 2.0f;
    constexpr float kScanOmegaRadPerS = 4.0f * kPi;
    const float kScanDistanceRad = TWO_PI * static_cast<float>(g_config.motor_pole_pairs);
    constexpr float kCalibrationRange = 0.05f;
    constexpr int32_t kDirectionDeadband = 8;

    int32_t position_counts = encoder_.positionCounts();

    switch (alignment_phase_)
    {
    case AlignmentPhase::SETTLE:
    {
        alignment_settle_time_s_ += dt;
        float smooth = smoothstepRamp_(alignment_settle_time_s_, kSettleTimeS);
        alignment_electrical_angle_ = 0.0f;
        id_target_ = g_config.calibration_test_current * smooth;

        if (alignment_settle_time_s_ >= kSettleTimeS)
        {
            alignment_start_counts_ = position_counts;
            alignment_count_sum_ = 0;
            alignment_step_count_ = 0;
            alignment_scan_distance_ = 0.0f;
            alignment_phase_ = AlignmentPhase::SCAN_FWD;
        }
        break;
    }

    case AlignmentPhase::SCAN_FWD:
    {
        alignment_scan_distance_ += kScanOmegaRadPerS * dt;
        alignment_electrical_angle_ = normalizeAngle2Pi(alignment_scan_distance_);
        id_target_ = g_config.calibration_test_current;

        alignment_count_sum_ += position_counts;
        alignment_step_count_++;

        if (alignment_scan_distance_ >= kScanDistanceRad)
        {
            int32_t delta = position_counts - alignment_start_counts_;

            if (delta > kDirectionDeadband || delta < -kDirectionDeadband)
            {
                float elec_rad_per_count = (static_cast<float>(g_config.motor_pole_pairs) * TWO_PI) / static_cast<float>(g_config.encoder_cpr);
                float expected_count_delta = kScanDistanceRad / elec_rad_per_count;
                float actual = std::abs(static_cast<float>(delta));

                if (std::abs(actual - expected_count_delta) / expected_count_delta > kCalibrationRange)
                {
                    alignment_phase_ = AlignmentPhase::FAILED;
                    last_alignment_fail_reason_ = AlignmentFailReason::CPR_POLE_PAIRS_MISMATCH;
                    break;
                }
            }
            else
            {
                alignment_phase_ = AlignmentPhase::FAILED;
                last_alignment_fail_reason_ = AlignmentFailReason::ENCODER_DIRECTION_UNCLEAR;
                break;
            }

            alignment_phase_ = AlignmentPhase::SCAN_REV;
        }
        break;
    }

    case AlignmentPhase::SCAN_REV:
    {
        alignment_scan_distance_ -= kScanOmegaRadPerS * dt;
        alignment_electrical_angle_ = normalizeAngle2Pi(alignment_scan_distance_);
        id_target_ = g_config.calibration_test_current;

        alignment_count_sum_ += position_counts;
        alignment_step_count_++;

        if (alignment_scan_distance_ <= 0.0f)
        {
            float average_counts = static_cast<float>(alignment_count_sum_) / static_cast<float>(alignment_step_count_);
            float offset = (average_counts / static_cast<float>(g_config.encoder_cpr)) * TWO_PI * static_cast<float>(g_config.motor_pole_pairs);

            alignment_electrical_angle_ = normalizeAngle2Pi(offset);
            id_target_ = 0.0f;
            alignment_phase_ = AlignmentPhase::DONE;
        }
        break;
    }

    case AlignmentPhase::DONE:
    case AlignmentPhase::FAILED:
        id_target_ = 0.0f;
        break;
    }
}

float Axis::getElectricalAngle() const
{
    if (state_ == State::ALIGNING || state_ == State::LUT_CALIBRATING)
    {
        return state_ == State::ALIGNING ? alignment_electrical_angle_ : lut_electrical_angle_;
    }

    float mechanical_angle = encoder_.angle();
    float electrical_angle = (mechanical_angle * static_cast<float>(g_config.motor_pole_pairs)) - g_config.encoder_offset_rad;

    if (g_config.encoder_lut_calibrated)
    {
        electrical_angle += lutCorrection_(encoder_.positionCounts());
    }

    return normalizeAngle2Pi(electrical_angle);
}

bool Axis::startLutCalibration()
{
    if (!g_config.encoder_calibrated)
    {
        last_lut_fail_reason_ = LutCalibrationFailReason::NOT_ALIGNED;
        return false;
    }

    if (!motor_.arm())
    {
        last_lut_fail_reason_ = LutCalibrationFailReason::ARM_FAILED;
        return false;
    }

    lut_phase_ = LutCalibrationPhase::SETTLE;
    last_lut_fail_reason_ = LutCalibrationFailReason::NONE;
    lut_settle_time_s_ = 0.0f;
    lut_electrical_angle_ = 0.0f;
    lut_sample_index_ = 0;
    lut_microstep_index_ = 0;
    id_target_ = 0.0f;
    lut_start_mech_angle_ = 0.0f;
    lut_end_mech_angle_ = 0.0f;

    for (uint32_t i = 0; i < kLutSize; i++)
    {
        lut_error_fwd_[i] = 0.0f;
        lut_error_bwd_[i] = 0.0f;
    }

    state_ = State::LUT_CALIBRATING;
    return true;
}

void Axis::stopLutCalibration()
{
    motor_.disarm();
    state_ = State::IDLE;
}

void Axis::updateLutCalibration(float dt)
{
    constexpr float kSettleTimeS = 0.05f;
    const float pole_pairs = static_cast<float>(g_config.motor_pole_pairs);
    const float kTotalSweepRad = TWO_PI * pole_pairs;
    const float kStepRad = kTotalSweepRad / static_cast<float>(kLutSize * kMicroStepsPerSample);

    switch (lut_phase_)
    {
    case LutCalibrationPhase::SETTLE:
    {
        lut_settle_time_s_ += dt;
        float smooth = smoothstepRamp_(lut_settle_time_s_, kSettleTimeS);
        lut_electrical_angle_ = 0.0f;
        id_target_ = g_config.calibration_test_current * smooth;

        if (lut_settle_time_s_ >= kSettleTimeS)
        {
            lut_start_mech_angle_ = encoder_.angle();
            lut_sample_index_ = 0;
            lut_microstep_index_ = 0;
            lut_electrical_angle_ = 0.0f;
            id_target_ = g_config.calibration_test_current;
            lut_phase_ = LutCalibrationPhase::SCAN_FWD;
        }
        break;
    }

    case LutCalibrationPhase::SCAN_FWD:
    {
        lut_electrical_angle_ += kStepRad;
        id_target_ = g_config.calibration_test_current;
        lut_microstep_index_++;

        if (lut_microstep_index_ >= kMicroStepsPerSample)
        {
            lut_microstep_index_ = 0;
            float theta_actual = encoder_.angle() * pole_pairs;
            float error = normalizeAngle(lut_electrical_angle_ - theta_actual);
            lut_error_fwd_[lut_sample_index_] = error;
            lut_sample_index_++;

            if (lut_sample_index_ >= kLutSize)
            {
                lut_sample_index_ = 0;
                lut_phase_ = LutCalibrationPhase::SCAN_REV;
            }
        }
        break;
    }

    case LutCalibrationPhase::SCAN_REV:
    {
        lut_electrical_angle_ -= kStepRad;
        id_target_ = g_config.calibration_test_current;
        lut_microstep_index_++;

        if (lut_microstep_index_ >= kMicroStepsPerSample)
        {
            lut_microstep_index_ = 0;
            float theta_actual = encoder_.angle() * pole_pairs;
            float error = normalizeAngle(lut_electrical_angle_ - theta_actual);
            lut_error_bwd_[lut_sample_index_] = error;
            lut_sample_index_++;

            if (lut_sample_index_ >= kLutSize)
            {
                lut_end_mech_angle_ = encoder_.angle();
                id_target_ = 0.0f;
                lut_phase_ = LutCalibrationPhase::COMPUTE;
            }
        }
        break;
    }

    case LutCalibrationPhase::COMPUTE:
    {
        float error_avg[kLutSize];
        for (uint32_t i = 0; i < kLutSize; i++)
        {
            error_avg[i] = (lut_error_fwd_[i] + lut_error_bwd_[kLutSize - 1 - i]) * 0.5f;
        }

        constexpr uint32_t kFilterHalfWindow = 2; // 5-point circular moving average
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
        float offset = normalizeAngle2Pi((raw_offset_mech * pole_pairs) + mean_error);

        g_config.encoder_offset_rad = offset;
        g_config.encoder_lut_calibrated = true;

        lut_phase_ = LutCalibrationPhase::DONE;
        break;
    }

    case LutCalibrationPhase::DONE:
    case LutCalibrationPhase::FAILED:
        id_target_ = 0.0f;
        break;
    }
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