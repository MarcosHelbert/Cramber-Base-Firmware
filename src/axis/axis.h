#ifndef AXIS_H
#define AXIS_H

#include <cstdint>

#include <error.h>

#include "anticogging.h"
#include "encoder.h"
#include "foc.h"
#include "motor.h"

#include <ffb/ffb_manager.h>
#include <ffb/bumpstop.h>

class Axis
{
public:
    enum class State
    {
        IDLE,
        SEARCH_INDEX,
        ENCODER_ALIGNMENT_START,
        ENCODER_ALIGNMENT_SCAN_FWD,
        ENCODER_ALIGNMENT_SCAN_REV,
        ANTICOGGING_CALIBRATION,
        READY,
        ERROR
    };

    Axis(uint32_t frequency, FOC &foc, Encoder &encoder, BumpStop &bumpstop, Motor &motor, Anticogging &anticogging, FFBManager &ffb_manager);

    void init();
    void update();

    void start();

    State getState() const { return state_; };
    void setError(Error error);
    void clearError();

    float getElectricalAngle() const;

private:
    State state_ = State::IDLE;
    State previous_state_ = State::IDLE;
    Error error_ = Error::ERROR_NONE;

    uint32_t frequency_;
    FOC &foc_;
    Encoder &encoder_;
    BumpStop &bumpstop_;
    Motor &motor_;
    Anticogging &anticogging_;
    FFBManager &ffb_manager_;

    float iq_prev_ = 0.0f;
    float max_step_;

    float open_loop_angle_ = 0.0f;
    float time_ = 0.0f;
    bool index_detected_ = false;
    uint32_t index_detect_time_ms_ = 0;
    uint32_t align_start_ms_ = 0;

    float open_loop_distance_ = 0.0f;
    int32_t init_enc_val_ = 0;
    int64_t encvaluesum_ = 0;
    uint32_t num_steps_ = 0;
    int8_t encoder_direction_ = 1;

    uint32_t anticogging_index_ = 0;
    float vel_integrator_ = 0.0f;
};

#endif // AXIS_H