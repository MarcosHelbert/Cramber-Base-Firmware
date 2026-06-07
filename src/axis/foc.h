#ifndef FOC_H
#define FOC_H

class FOC
{
public:
    FOC(float frequency = 1000) : dt_(1.0f / frequency) {}

    void reset();

    void update(float ia, float ib, float ic, float electrical_angle, float omega_e, float inductance, float flux_linkage, float vbus);
    void setGains(float kp_gain, float ki_gain);
    void setTargets(float id, float iq);

    float getVa() const { return va_; }
    float getVb() const { return vb_; }
    float getVc() const { return vc_; }

    float getId() const { return id_; }
    float getIq() const { return iq_; }
    float getModD(float vbus) const { return vd_ * (3.0f / (2.0f * vbus)); }
    float getModQ(float vbus) const { return vq_ * (3.0f / (2.0f * vbus)); }

private:
    const float dt_;

    float va_ = 0.5f;
    float vb_ = 0.5f;
    float vc_ = 0.5f;

    float id_ = 0.0f;
    float iq_ = 0.0f;
    float vd_ = 0.0f;
    float vq_ = 0.0f;
    float vmax_ = 1.0f;

    float kp_ = 0.0f;
    float ki_ = 0.0f;

    float id_target_ = 0.0f;
    float iq_target_ = 0.0f;

    float d_integral_ = 0.0f;
    float q_integral_ = 0.0f;

    void current_control(float vmax, float d, float q, float &vd, float &vq, float omega_e, float L, float flux);
};

#endif // FOC_H