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
    if (!isPositionReferenced())
    {
        last_fail_reason_ = CalibrationFailReason::NOT_HOMED;
        return false;
    }

    if (!motor_.arm())
        return false;

    phase_ = CalibrationPhase::SETTLE;
    last_fail_reason_ = CalibrationFailReason::NONE;
    electrical_angle_ = 0.0f;
    id_target_ = 0.0f;
    settle_time_s_ = 0.0f;
    settle_still_s_ = 0.0f;
    scan_start_counts_ = 0;
    lut_sample_index_ = 0;
    lut_microstep_index_ = 0;
    lut_start_mech_angle_ = 0.0f;
    lut_offset_estimate_ = 0.0f;

    for (uint32_t i = 0; i < kLutSize; i++)
    {
        lut_error_fwd_[i] = 0.0f;
        lut_error_bwd_[i] = 0.0f;
        lut_count_fwd_[i] = 0;
        lut_count_bwd_[i] = 0;
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
    if (!g_config.motor_calibrated)
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

bool Axis::startAnticoggingCalibration()
{
    if (!g_config.motor_calibrated || !g_config.encoder_calibrated)
    {
        last_anticogging_fail_reason_ = AnticoggingFailReason::NOT_READY;
        return false;
    }

    if (!isPositionReferenced())
    {
        last_anticogging_fail_reason_ = AnticoggingFailReason::NOT_HOMED;
        return false;
    }

    if (!motor_.arm())
    {
        last_anticogging_fail_reason_ = AnticoggingFailReason::ARM_FAILED;
        return false;
    }

    last_anticogging_fail_reason_ = AnticoggingFailReason::NONE;
    anticogging_phase_ = AnticoggingPhase::SETTLE;
    anticogging_settle_time_s_ = 0.0f;
    anticogging_velocity_integral_ = 0.0f;
    anticogging_sample_index_ = 0;
    id_target_ = 0.0f;
    iq_target_ = 0.0f;

    for (uint32_t i = 0; i < kLutSize; i++)
    {
        anticogging_iq_sum_[i] = 0.0f;
        anticogging_iq_count_[i] = 0;
    }

    state_ = State::CALIBRATING_ANTICOGGING;
    return true;
}

void Axis::stopAnticoggingCalibration()
{
    if (anticogging_phase_ == AnticoggingPhase::DONE)
    {
        g_config.anticogging_calibrated = true;
    }

    motor_.disarm();
    iq_target_ = 0.0f;
    state_ = State::IDLE;
}

void Axis::update(float dt)
{
    if (state_ == State::RUNNING)
    {
        if (motor_.getState() != Motor::MotorState::ARMED)
        {
            ffb_manager_.stopFFB();
            iq_prev_ = 0.0f;
            id_target_ = 0.0f;
            iq_target_ = 0.0f;
            state_ = State::IDLE;
            return;
        }

        float max_step = g_config.ffb_slew_rate_a_per_s * dt;

        float position_rad = encoder_.angle();
        float velocity = encoder_.velocity();
        float acceleration = encoder_.acceleration();

        float torque = ffb_manager_.getTorque(position_rad, velocity, acceleration, millis());
        torque += bumpstop_.calculate(position_rad, velocity);
        torque += idle_spring_.calculate(position_rad, velocity, ffb_manager_.isGameDriving(millis()), dt);

        if (g_config.anticogging_enabled && g_config.anticogging_calibrated)
        {
            torque += anticoggingCorrection_(encoder_.positionCounts());
        }

        torque = std::clamp(torque, -g_config.max_current, g_config.max_current);

        float delta = torque - iq_prev_;
        delta = std::clamp(delta, -max_step, max_step);
        iq_prev_ += delta;

        id_target_ = 0.0f;
        iq_target_ = iq_prev_;
        return;
    }

    if (state_ == State::CALIBRATING_ANTICOGGING)
    {
        constexpr float kVelKp = 0.3f;
        constexpr float kVelKi = 0.8f;
        constexpr float kSettleTimeoutS = 5.0f;
        constexpr float kVelocityToleranceRadS = 0.3f;

        const float target_velocity = g_config.anticogging_scan_velocity_rad_s;
        float velocity = encoder_.velocity();
        float vel_error = target_velocity - velocity;

        anticogging_velocity_integral_ += kVelKi * vel_error * dt;
        anticogging_velocity_integral_ = std::clamp(anticogging_velocity_integral_, -g_config.max_current, g_config.max_current);

        float iq = kVelKp * vel_error + anticogging_velocity_integral_;
        iq = std::clamp(iq, -g_config.max_current, g_config.max_current);

        id_target_ = 0.0f;
        iq_target_ = iq;

        if (anticogging_phase_ == AnticoggingPhase::SETTLE)
        {
            anticogging_settle_time_s_ += dt;

            if (std::abs(vel_error) <= kVelocityToleranceRadS)
            {
                anticogging_start_counts_ = encoder_.positionCounts();
                anticogging_sample_index_ = 0;
                anticogging_settle_time_s_ = 0.0f;
                anticogging_phase_ = AnticoggingPhase::SCAN;
                return;
            }

            if (anticogging_settle_time_s_ >= kSettleTimeoutS)
            {
                iq_target_ = 0.0f;
                anticogging_phase_ = AnticoggingPhase::FAILED;
                last_anticogging_fail_reason_ = AnticoggingFailReason::VELOCITY_NOT_REACHED;
                return;
            }
            return;
        }

        if (anticogging_phase_ == AnticoggingPhase::SCAN)
        {
            int32_t position_counts = encoder_.positionCounts();
            int32_t delta = position_counts - anticogging_start_counts_;

            uint32_t idx = lutBucket_(position_counts);
            anticogging_iq_sum_[idx] += iq;
            anticogging_iq_count_[idx]++;

            anticogging_settle_time_s_ += dt;
            const float scan_timeout_s = 4.0f * (TWO_PI / std::max(target_velocity, 0.1f)) + 2.0f;
            if (anticogging_settle_time_s_ > scan_timeout_s)
            {
                iq_target_ = 0.0f;
                anticogging_phase_ = AnticoggingPhase::FAILED;
                last_anticogging_fail_reason_ = AnticoggingFailReason::VELOCITY_NOT_REACHED;
                return;
            }

            if (delta >= static_cast<int32_t>(g_config.encoder_cpr))
            {
                iq_target_ = 0.0f;
                anticogging_phase_ = AnticoggingPhase::COMPUTE;
            }
            return;
        }

        if (anticogging_phase_ == AnticoggingPhase::COMPUTE)
        {
            iq_target_ = 0.0f;

            float averaged[kLutSize];
            for (uint32_t i = 0; i < kLutSize; i++)
            {
                averaged[i] = (anticogging_iq_count_[i] > 0)
                                  ? (anticogging_iq_sum_[i] / static_cast<float>(anticogging_iq_count_[i]))
                                  : 0.0f;
            }

            float mean_iq = 0.0f;
            for (uint32_t i = 0; i < kLutSize; i++)
            {
                mean_iq += averaged[i];
            }
            mean_iq /= static_cast<float>(kLutSize);

            for (uint32_t i = 0; i < kLutSize; i++)
            {
                g_config.anticogging_map[i] = averaged[i] - mean_iq;
            }

            anticogging_phase_ = AnticoggingPhase::DONE;
            return;
        }

        iq_target_ = 0.0f;
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
        constexpr float kRampS = 0.15f;
        constexpr float kHoldFirstS = 0.30f;
        constexpr float kSlewS = 0.15f;
        constexpr float kMinSettleS = 0.70f;
        constexpr float kMaxSettleS = 3.0f;
        constexpr float kStillVelocity = 0.2f;
        constexpr float kStillTimeS = 0.10f;
        constexpr float kHalfPi = TWO_PI * 0.25f;

        settle_time_s_ += dt;
        float smooth = smoothstepRamp_(settle_time_s_, kRampS);
        id_target_ = g_config.calibration_test_current * smooth;

        if (settle_time_s_ < kHoldFirstS)
            electrical_angle_ = kHalfPi;
        else
            electrical_angle_ = kHalfPi * (1.0f - std::min((settle_time_s_ - kHoldFirstS) / kSlewS, 1.0f));

        if (std::abs(encoder_.velocity()) < kStillVelocity)
            settle_still_s_ += dt;
        else
            settle_still_s_ = 0.0f;

        bool settled = settle_time_s_ >= kMinSettleS && settle_still_s_ >= kStillTimeS;
        if (settled || settle_time_s_ >= kMaxSettleS)
        {
            electrical_angle_ = 0.0f;
            scan_start_counts_ = position_counts;
            lut_start_mech_angle_ = encoder_.rawAngle();
            lut_offset_estimate_ = normalizeAngle2Pi(lut_start_mech_angle_ * pole_pairs);
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

        float theta_est = encoder_.rawAngle() * pole_pairs - lut_offset_estimate_;
        uint32_t bucket = lutBucket_(position_counts);
        lut_error_fwd_[bucket] += normalizeAngle(electrical_angle_ - theta_est);
        lut_count_fwd_[bucket]++;

        lut_microstep_index_++;
        if (lut_microstep_index_ >= kMicroStepsPerSample)
        {
            lut_microstep_index_ = 0;
            lut_sample_index_++;

            if (lut_sample_index_ >= kLutSize)
            {
                int32_t delta = position_counts - scan_start_counts_;
                last_scan_delta_counts_ = delta;

                if (delta < -kDirectionDeadband)
                {
                    g_config.encoder_reverse = !g_config.encoder_reverse;
                    encoder_.reset();
                    phase_ = CalibrationPhase::FAILED;
                    last_fail_reason_ = CalibrationFailReason::ENCODER_DIRECTION_REVERSED;
                    break;
                }

                if (delta > kDirectionDeadband)
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

        float theta_est = encoder_.rawAngle() * pole_pairs - lut_offset_estimate_;
        uint32_t bucket = lutBucket_(position_counts);
        lut_error_bwd_[bucket] += normalizeAngle(electrical_angle_ - theta_est);
        lut_count_bwd_[bucket]++;

        lut_microstep_index_++;
        if (lut_microstep_index_ >= kMicroStepsPerSample)
        {
            lut_microstep_index_ = 0;
            lut_sample_index_++;

            if (lut_sample_index_ >= kLutSize)
            {
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
            float fwd = lut_count_fwd_[i] ? lut_error_fwd_[i] / static_cast<float>(lut_count_fwd_[i]) : 0.0f;
            float bwd = lut_count_bwd_[i] ? lut_error_bwd_[i] / static_cast<float>(lut_count_bwd_[i]) : 0.0f;
            if (lut_count_fwd_[i] && lut_count_bwd_[i])
                error_avg[i] = (fwd + bwd) * 0.5f;
            else
                error_avg[i] = lut_count_fwd_[i] ? fwd : bwd;
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

        g_config.encoder_offset_rad = normalizeAngle2Pi(lut_offset_estimate_ - mean_error);

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

    float mechanical_angle = encoder_.rawAngle();
    float electrical_angle = (mechanical_angle * static_cast<float>(g_config.motor_pole_pairs)) - g_config.encoder_offset_rad;

    if (g_config.encoder_lut_calibrated)
    {
        electrical_angle += lutCorrection_(encoder_.positionCounts());
    }

    return normalizeAngle2Pi(electrical_angle);
}

static float interpolateTable(const float *table, uint32_t size, int32_t position_counts, uint32_t cpr)
{
    const int32_t icpr = static_cast<int32_t>(cpr);
    int32_t wrapped = ((position_counts % icpr) + icpr) % icpr;

    float index_f = (static_cast<float>(wrapped) / static_cast<float>(cpr)) * static_cast<float>(size) - 0.5f;
    if (index_f < 0.0f)
        index_f += static_cast<float>(size);

    uint32_t i0 = static_cast<uint32_t>(index_f) % size;
    uint32_t i1 = (i0 + 1) % size;
    float frac = index_f - static_cast<float>(static_cast<uint32_t>(index_f));

    return table[i0] + (table[i1] - table[i0]) * frac;
}

float Axis::lutCorrection_(int32_t position_counts) const
{
    return interpolateTable(g_config.encoder_lut, kLutSize, position_counts, g_config.encoder_cpr);
}

uint32_t Axis::lutBucket_(int32_t position_counts) const
{
    const int32_t cpr = static_cast<int32_t>(g_config.encoder_cpr);
    int32_t wrapped = ((position_counts % cpr) + cpr) % cpr;
    uint32_t bucket = static_cast<uint32_t>((static_cast<int64_t>(wrapped) * kLutSize) / cpr);
    return bucket < kLutSize ? bucket : kLutSize - 1;
}

float Axis::anticoggingCorrection_(int32_t position_counts) const
{
    return interpolateTable(g_config.anticogging_map, kLutSize, position_counts, g_config.encoder_cpr);
}
