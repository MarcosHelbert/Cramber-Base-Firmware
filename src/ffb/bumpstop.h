#ifndef BUMPSTOP_H
#define BUMPSTOP_H

class BumpStop
{
public:
    void init();
    void update();
    float calculate(float position_rad, float velocity_rad_s);

private:
};

#endif // BUMPSTOP_H
