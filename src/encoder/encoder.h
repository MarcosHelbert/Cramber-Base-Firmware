#ifndef ENCODER_H
#define ENCODER_H

#include <cstdint>

#include <kalman3d.h>

#include <persistence/config_store.h>

class Encoder
{
public:
    explicit Encoder(const PersistedConfig &config);

    void init();
    void update();
    void reset();

    void setIdxSubscribe(bool enable);
    bool isIndexed() const { return indexed_; }

    float angle() const { return kalman_.state0(); }
    float velocity() const { return kalman_.state1(); }
    float acceleration() const { return kalman_.state2(); }

    int32_t positionCounts() const { return absolute_position_; }

private:
    const PersistedConfig &config_;

    const float two_pi_over_cpr_;

    uint32_t last_count_ = 0;
    int32_t absolute_position_ = 0;
    bool indexed_ = false;

    KalmanFilter3D kalman_;
};

#endif // ENCODER_H
