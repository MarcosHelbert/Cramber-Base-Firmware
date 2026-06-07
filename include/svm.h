#ifndef SVM_H
#define SVM_H

#include "utils.h"

inline void clarke(float ia, float ib, float ic, float &alpha, float &beta)
{
    alpha = ia;
    beta = INV_SQRT3 * (ib - ic);
}

inline void park(float alpha, float beta, float sin_val, float cos_val, float &d, float &q)
{
    d = alpha * cos_val + beta * sin_val;
    q = -alpha * sin_val + beta * cos_val;
}

inline void invPark(float vd, float vq, float sin_val, float cos_val, float &alpha, float &beta)
{
    alpha = vd * cos_val - vq * sin_val;
    beta = vd * sin_val + vq * cos_val;
}

inline void invClarke(float alpha, float beta, float &va, float &vb, float &vc)
{
    float half_alpha = -0.5f * alpha;
    float beta_ = SQRT3_BY_2 * beta;

    va = alpha;
    vb = half_alpha + beta_;
    vc = half_alpha - beta_;
}

inline void svm(float va, float vb, float vc, float vbus, float &duty_a, float &duty_b, float &duty_c)
{
    if (vbus < 1.0f)
        vbus = 1.0f;

    float max_v = max(max(va, vb), vc);
    float min_v = min(min(va, vb), vc);
    float offset = (max_v + min_v) * 0.5f;

    va -= offset;
    vb -= offset;
    vc -= offset;

    duty_a = (va / vbus) + 0.5f;
    duty_b = (vb / vbus) + 0.5f;
    duty_c = (vc / vbus) + 0.5f;
}

#endif // SVM_H