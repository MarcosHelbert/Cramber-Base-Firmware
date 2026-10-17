#ifndef BUMPSTOP_H
#define BUMPSTOP_H

class BumpStop
{
public:
    void init();

    void updateLimits();
    float calculate(float position_rad, float velocity_rad_s) const;

private:
    bool enabled_ = false;

    float half_range_rad_ = 0.0f;
    float transition_rad_ = 0.0f;
    float inv_transition_rad_ = 0.0f;

    float stiffness_ = 0.0f;
    float damping_ = 0.0f;

    float wall_start_pos_ = 0.0f;
    float wall_start_neg_ = 0.0f;

    float torque_clamp_ = 0.0f;
};

#endif // BUMPSTOP_H
