#ifndef ENCODER_H
#define ENCODER_H

#include <cstdint>

class Encoder
{
public:
    Encoder(uint16_t cpr, bool reverse_dir, uint32_t update_frequency_hz, uint32_t foc_frequency_hz);

    void init();
    void update();
    void updateFOC();
    void reset();

    inline int32_t getPosition() const { return absolute_position_; }

    inline float getMechanicalAngle() const { return mechanical_angle_; }
    inline bool isIndexed() const { return indexed_; }
    void setIdxSubscribe(bool enable);

    inline float getVelocity() const { return velocity_mt_; }

    inline float getVelocityFF() const { return velocity_pll_; }
    inline float getAcceleration() const { return acceleration_filtered_; }

private:
    uint32_t last_count_enc_ = 0;
    uint32_t last_count_pll_ = 0;
    int32_t overflow_count_ = 0;
    int32_t absolute_position_ = 0;
    float mechanical_angle_ = 0.0f;

    int32_t mt_count_accum_ = 0;
    uint32_t mt_time_accum_ = 0;
    uint32_t mt_last_cyccnt_ = 0;
    uint32_t mt_silence_start_ = 0;
    float velocity_mt_ = 0.0f;
    float alpha_mt_;

    float last_velocity_mt_ = 0.0f;
    float acceleration_filtered_ = 0.0f;
    float alpha_acc_;

    static constexpr int32_t MT_WINDOW_COUNTS = 4;
    static constexpr uint32_t MT_TIMEOUT_CYCLES = 3360000u;

    static constexpr float CYCLES_TO_SEC = 1.0f / 168000000.0f;

    float pll_pos_ = 0.0f;          // [counts float]
    float pll_vel_ = 0.0f;          // [counts/s]
    float velocity_pll_ = 0.0f;     // [rad/s]
    float acceleration_pll_ = 0.0f; // [rad/s²]
    float last_velocity_pll_ = 0.0f;

    float pll_kp_;
    float pll_ki_;
    float dt_foc_;

    uint16_t resolution_;
    int16_t half_resolution_;
    uint32_t update_frequency_hz_;
    uint32_t foc_frequency_hz_;
    float two_pi_over_cpr_;
    bool reverse_direction_;
    bool indexed_ = false;
};

#endif // ENCODER_H