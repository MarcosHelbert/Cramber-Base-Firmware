#ifndef ENCODER_H
#define ENCODER_H

#include <cstdint>
#include <kalman3d.h>

class Encoder
{
public:
    Encoder(uint16_t cpr, bool reverse_dir, uint32_t update_hz);

    void init();
    void update();
    void reset();
    void setIdxSubscribe(bool enable);

    int32_t position() const { return absolute_position_; }
    float mechanicalAngle() const { return mechanical_angle_; }
    bool isIndexed() const { return indexed_; }

    float velocity() const { return kalman_.state1(); }
    float acceleration() const { return kalman_.state2(); }

private:
    const uint16_t resolution_;
    const int16_t half_resolution_;
    const float two_pi_over_cpr_;
    const bool reverse_direction_;

    uint32_t last_count_ = 0;
    int32_t overflow_count_ = 0;
    int32_t absolute_position_ = 0;
    float mechanical_angle_ = 0.0f;
    bool indexed_ = false;

    KalmanFilter3D kalman_;
};

#endif // ENCODER_H