#include "axis.h"

#include <utils.h>
#include <config.h>
#include <system_state.h>

#include <hardware/timer.h>

Axis::Axis(
    uint32_t frequency,
    FOC &foc,
    Encoder &encoder,
    BumpStop &bumpstop,
    Motor &motor,
    Anticogging &anticogging,
    FFBManager &ffb_manager)
    : frequency_(frequency),
      foc_(foc),
      encoder_(encoder),
      bumpstop_(bumpstop),
      motor_(motor),
      anticogging_(anticogging),
      ffb_manager_(ffb_manager)
{
    float dt = 1.0f / frequency_;
    max_step_ = MAX_DI_DT * dt;
}

void Axis::init()
{
    state_ = State::IDLE;
}

void Axis::update()
{
    switch (state_)
    {
    case State::SEARCH_INDEX:
    {
        const float dt = 1.0f / frequency_;
        const float ramp_time = 0.5f;
        time_ += dt;

        float x = time_ / ramp_time;
        if (x > 1.0f)
            x = 1.0f;

        float smooth = (3.0f * square(x)) - (2.0f * cube(x));
        float target_velocity = CALIBRATION_SCAN_OMEGA * smooth;

        open_loop_angle_ += target_velocity * dt;
        if (open_loop_angle_ > TWO_PI)
            open_loop_angle_ -= TWO_PI;

        float iq = CALIBRATION_CURRENT * smooth;
        foc_.setTargets(0.0f, iq);

        if (x >= 1.0f && !index_detected_)
        {
            encoder_.setIdxSubscribe(true);
            index_detected_ = true;
        }

        if (index_detected_ && encoder_.isIndexed())
        {
            foc_.setTargets(0.0f, 0.0f);

            open_loop_angle_ = 0.0f;
            time_ = 0.0f;
            align_start_ms_ = millis();

            state_ = State::ENCODER_ALIGNMENT_START;
            index_detected_ = false;
        }

        break;
    }

    case State::ENCODER_ALIGNMENT_START:
    {
        const float dt = 1.0f / frequency_;
        open_loop_angle_ = 0.0f;

        const float ramp_time = 0.05f;
        time_ += dt;

        float x = time_ / ramp_time;
        if (x > 1.0f)
            x = 1.0f;

        float smooth = (3.0f * square(x)) - (2.0f * cube(x));
        float id = CALIBRATION_CURRENT * smooth;

        foc_.setTargets(id, 0.0f);

        if (millis() - align_start_ms_ >= CALIBRATION_LOCK_DURATION_MS)
        {
            init_enc_val_ = encoder_.getPosition();
            encvaluesum_ = 0;
            num_steps_ = 0;
            open_loop_distance_ = 0.0f;

            state_ = State::ENCODER_ALIGNMENT_SCAN_FWD;
        }
        break;
    }

    case State::ENCODER_ALIGNMENT_SCAN_FWD:
    {
        const float dt = 1.0f / frequency_;
        open_loop_distance_ += CALIBRATION_SCAN_OMEGA * dt;

        open_loop_angle_ = fmod_fast(open_loop_distance_, TWO_PI);
        if (open_loop_angle_ < 0.0f)
            open_loop_angle_ += TWO_PI;

        foc_.setTargets(CALIBRATION_CURRENT, 0.0f);

        encvaluesum_ += encoder_.getPosition();
        num_steps_++;

        if (open_loop_distance_ >= CALIBRATION_SCAN_DISTANCE)
        {
            int32_t current_enc = encoder_.getPosition();

            // Determine direction
            if (current_enc > init_enc_val_ + 8)
            {
                encoder_direction_ = 1;
            }
            else if (current_enc < init_enc_val_ - 8)
            {
                encoder_direction_ = -1;
            }
            else
            {
                setError(Error::ERROR_ENCODER_FAILED);
                return;
            }

            // Check CPR vs Pole Pairs mismatch
            float elec_rad_per_enc = (MOTOR_POLE_PAIRS * TWO_PI) / (float)ENCODER_CPR;
            float expected_encoder_delta = CALIBRATION_SCAN_DISTANCE / elec_rad_per_enc;
            float actual_response = abs((float)(current_enc - init_enc_val_));

            if (abs(actual_response - expected_encoder_delta) / expected_encoder_delta > CALIBRATION_RANGE)
            {
                setError(Error::ERROR_CPR_POLEPAIRS_MISMATCH);
                return;
            }

            state_ = State::ENCODER_ALIGNMENT_SCAN_REV;
        }
        break;
    }

    case State::ENCODER_ALIGNMENT_SCAN_REV:
    {
        const float dt = 1.0f / frequency_;
        open_loop_distance_ -= CALIBRATION_SCAN_OMEGA * dt;

        open_loop_angle_ = fmod_fast(open_loop_distance_, TWO_PI);
        if (open_loop_angle_ < 0.0f)
            open_loop_angle_ += TWO_PI;

        foc_.setTargets(CALIBRATION_CURRENT, 0.0f);

        encvaluesum_ += encoder_.getPosition();
        num_steps_++;

        if (open_loop_distance_ <= 0.0f)
        {
            float average_enc_count = (float)encvaluesum_ / (float)num_steps_;
            float offset = (average_enc_count / (float)ENCODER_CPR) * TWO_PI * MOTOR_POLE_PAIRS;

            g_state.encoder_offset_rad = fmod_fast(offset, TWO_PI);
            if (g_state.encoder_offset_rad < 0.0f)
                g_state.encoder_offset_rad += TWO_PI;

            foc_.setTargets(0.0f, 0.0f);
            foc_.reset();

            encoder_.setIdxSubscribe(false);

            state_ = State::ANTICOGGING_CALIBRATION;
        }
        break;
    }

    case State::ANTICOGGING_CALIBRATION:
    {
        if (!COGGING_COMPENSATION_ENABLED)
        {
            state_ = State::READY;
            break;
        }

        const uint32_t samples = COGGING_COMPENSATION_SAMPLING_POINTS;

        float target_mech = (static_cast<float>(anticogging_index_) / samples) * TWO_PI;
        float current_mech = (encoder_.getPosition() - g_state.mechanical_offset) * (TWO_PI / ENCODER_CPR);
        float velocity = encoder_.getVelocity();

        float pos_err = target_mech - current_mech;
        pos_err = normalizeAngle(pos_err);

        float torque_p = COGGING_COMPENSATION_KP * pos_err;

        vel_integrator_ += COGGING_COMPENSATION_KI * pos_err * (1.0f / frequency_);
        vel_integrator_ = clamp(vel_integrator_, -MAX_CURRENT, MAX_CURRENT);

        float torque_d = -COGGING_COMPENSATION_KD * velocity;

        float torque = torque_p + torque_d + vel_integrator_;
        torque = clamp(torque, -MAX_CURRENT, MAX_CURRENT);
        foc_.setTargets(0.0f, torque);

        if (abs(pos_err) <= COGGING_COMPENSATION_POS_EPS && abs(velocity) <= COGGING_COMPENSATION_VEL_EPS)
        {
            anticogging_.setSample(anticogging_index_, vel_integrator_);

            anticogging_index_++;
            vel_integrator_ = 0.0f;

            if (anticogging_index_ >= samples)
            {
                foc_.setTargets(0.0f, 0.0f);
                foc_.reset();
                anticogging_index_ = 0;
                state_ = State::READY;
            }
        }

        break;
    }

    case State::READY:
    {
        float torque = 0.0f;

        const float encoder_to_rad = TWO_PI / ENCODER_CPR;

        const int32_t position_counts = encoder_.getPosition();
        int32_t centered_counts = position_counts - g_state.mechanical_offset;

        float positionRad = static_cast<float>(centered_counts) * encoder_to_rad;
        float velocity = encoder_.getVelocity();
        float acceleration = encoder_.getAcceleration();

        torque += ffb_manager_.getTorque(positionRad, velocity, acceleration, millis());
        torque += bumpstop_.calculate(centered_counts, velocity);

        if (COGGING_COMPENSATION_ENABLED)
        {
            float anticog = anticogging_.getCompensation(positionRad);
            anticog = clamp(anticog, -0.5f, 0.5f);
            torque += anticog;
        }

        torque = clamp(torque, -MAX_CURRENT, MAX_CURRENT);

        float delta = torque - iq_prev_;
        delta = clamp(delta, -max_step_, max_step_);

        iq_prev_ += delta;

        foc_.setTargets(0.0f, iq_prev_);

        break;
    }

    default:
        break;
    }
}

void Axis::setError(Error error)
{
    foc_.setTargets(0.0f, 0.0f);
    error_ = error;
    if (state_ != State::ERROR)
        previous_state_ = state_;
    state_ = State::ERROR;
}

void Axis::clearError()
{
    if (state_ == State::ERROR)
    {
        error_ = Error::ERROR_NONE;
        foc_.reset();
        iq_prev_ = 0.0f;
        state_ = previous_state_;
    }
}

float Axis::getElectricalAngle() const
{
    if (state_ == State::SEARCH_INDEX || state_ == State::ENCODER_ALIGNMENT_START || state_ == State::ENCODER_ALIGNMENT_SCAN_FWD || state_ == State::ENCODER_ALIGNMENT_SCAN_REV)
    {
        return open_loop_angle_;
    }

    return motor_.getElectricalAngle();
}

void Axis::start()
{
    if (state_ == State::IDLE)
    {
        state_ = State::SEARCH_INDEX;
    }
}
