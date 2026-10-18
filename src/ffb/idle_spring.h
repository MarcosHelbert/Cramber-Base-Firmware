#ifndef IDLE_SPRING_H
#define IDLE_SPRING_H

class IdleSpring
{
public:
    void reset() { weight_ = 0.0f; }

    float calculate(float position_rad, float velocity_rad_s, bool game_driving, float dt);

    bool isEngaged() const { return weight_ > 0.0f; }

private:
    float weight_ = 0.0f;
};

#endif // IDLE_SPRING_H
