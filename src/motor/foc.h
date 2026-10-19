#ifndef FOC_H
#define FOC_H

#include <FastMath.h>

#include <persistence/config_store.h>

class FOC
{
public:
    FOC(const PersistedConfig &config, float frequency = 1000)
        : config_(config), dt_(1.0f / frequency) {}

    void reset();
    void update(float ia, float ib, float ic, float electrical_angle, float omega_e, float inductance, float flux_linkage, float vbus);

    void updateGainsFromConfig();

    void setTargets(float id, float iq);

    float getVa() const { return va_; }
    float getVb() const { return vb_; }
    float getVc() const { return vc_; }

    float getId() const { return id_; }
    float getVd() const { return vd_; }
    float getVq() const { return vq_; }
    float getIq() const { return iq_; }

    float getModD(float vbus) const { return (vbus > 0.0f) ? (vd_ / (vbus * INV_SQRT3)) : 0.0f; }
    float getModQ(float vbus) const { return (vbus > 0.0f) ? (vq_ / (vbus * INV_SQRT3)) : 0.0f; }

private:
    void current_control(float vmax, float d, float q, float &vd, float &vq, float omega_e, float L, float flux);

private:
    const PersistedConfig &config_;

    float dt_;
    float kp_ = 0.0f;
    float ki_ = 0.0f;

    float id_target_ = 0.0f;
    float iq_target_ = 0.0f;
    float id_ = 0.0f;
    float iq_ = 0.0f;

    float vd_ = 0.0f;
    float vq_ = 0.0f;
    float d_integral_ = 0.0f;
    float q_integral_ = 0.0f;

    float va_ = 0.5f;
    float vb_ = 0.5f;
    float vc_ = 0.5f;
};

#endif // FOC_H
