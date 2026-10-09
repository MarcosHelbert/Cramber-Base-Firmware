#include "foc.h"

#include <algorithm>
#include <FastMath.h>

#include "svm.h"

void FOC::reset()
{
    d_integral_ = 0.0f;
    q_integral_ = 0.0f;
    va_ = vb_ = vc_ = 0.5f;
}

void FOC::update(float ia, float ib, float ic, float electrical_angle, float omega_e, float inductance, float flux_linkage, float vbus)
{
    float sin_val, cos_val;
    FastMath::fast_sincos(electrical_angle, &sin_val, &cos_val);

    float alpha, beta;
    clarke(ia, ib, ic, alpha, beta);

    float d, q;
    park(alpha, beta, sin_val, cos_val, d, q);

    id_ = d;
    iq_ = q;

    float vmax = vbus * INV_SQRT3;

    float vd, vq;
    current_control(vmax, d, q, vd, vq, omega_e, inductance, flux_linkage);

    invPark(vd, vq, sin_val, cos_val, alpha, beta);
    invClarke(alpha, beta, va_, vb_, vc_);
}

void FOC::updateGainsFromConfig()
{
    kp_ = config_.current_kp;
    ki_ = config_.current_ki;
}

void FOC::setTargets(float id, float iq)
{
    id_target_ = std::clamp(id, -config_.max_current, config_.max_current);
    iq_target_ = std::clamp(iq, -config_.max_current, config_.max_current);
}

void FOC::current_control(float vmax, float d, float q, float &vd, float &vq, float omega_e, float L, float flux)
{
    float err_d = id_target_ - d;
    float err_q = iq_target_ - q;

    // Feedforward: Coupling compensation + Back-EMF
    float vd_ff = -omega_e * L * q;
    float vq_ff = omega_e * L * d + omega_e * flux;

    float vd_p = kp_ * err_d;
    float vq_p = kp_ * err_q;
    vd = vd_p + d_integral_ + vd_ff;
    vq = vq_p + q_integral_ + vq_ff;

    float v_mag_sq = square(vd) + square(vq);
    float vmax_sq = square(vmax);

    float vd_sat = vd;
    float vq_sat = vq;

    if (v_mag_sq > vmax_sq)
    {
        float scale = vmax / FastMath::fast_sqrt(v_mag_sq);
        vd_sat = vd * scale;
        vq_sat = vq * scale;
    }

    float kaw = 1.0f;
    d_integral_ += (ki_ * err_d + kaw * (vd_sat - vd)) * dt_;
    q_integral_ += (ki_ * err_q + kaw * (vq_sat - vq)) * dt_;
    d_integral_ = std::clamp(d_integral_, -vmax, vmax);
    q_integral_ = std::clamp(q_integral_, -vmax, vmax);

    vd_ = vd_sat;
    vq_ = vq_sat;

    vd = vd_sat;
    vq = vq_sat;
}
