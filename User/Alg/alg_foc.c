/**
  ******************************************************************************
  * @file    alg_foc.c
  * @brief   FOC current loop (see alg_foc.h).
  *
  * Layer: Alg.
  *
  *   ia, ib, ic --Clarke--> i_alpha, i_beta --Park(theta)--> id, iq
  *      id_ref - id --> PI_d --> vd
  *      iq_ref - iq --> PI_q --> vq
  *   vd, vq --circle limit--> --invPark--> v_alpha, v_beta --> SVPWM --> duty
  *
  * The circle limit is applied in the dq frame before the inverse Park, so the
  * angle of the voltage vector is preserved (no torque direction error when the
  * loop saturates).
  ******************************************************************************
  */

#include "alg_foc.h"
#include <math.h>
#include "alg_svpwm.h"
#include "mid_trig.h"

#define FOC_INV_SQRT3   (0.57735027f)   /* 1/sqrt(3) */
#define FOC_SQRT3       (1.73205081f)   /* sqrt(3)   */

void alg_foc_init(alg_foc_t *f, float vdc, float m_max,
                  float kp, float ki, float v_lim)
{
    if (f == 0) {
        return;
    }

    alg_pi_init(&f->pi_d, kp, ki, -v_lim, v_lim);
    alg_pi_init(&f->pi_q, kp, ki, -v_lim, v_lim);

    f->vdc   = (vdc > 0.0f) ? vdc : 12.0f;
    f->m_max = (m_max > 0.0f) ? m_max : 1.0f;
    if (f->m_max > ALG_SVPWM_M_MAX) {
        f->m_max = ALG_SVPWM_M_MAX;   /* beyond this the duties leave 0..1 */
    }
    f->v_lim = (v_lim > 0.0f) ? v_lim : (0.5f * f->vdc * f->m_max);

    alg_foc_reset(f);
}

void alg_foc_reset(alg_foc_t *f)
{
    if (f == 0) {
        return;
    }
    alg_pi_reset(&f->pi_d);
    alg_pi_reset(&f->pi_q);
    f->id = 0.0f;
    f->iq = 0.0f;
    f->vd = 0.0f;
    f->vq = 0.0f;
    f->m  = 0.0f;
}

void alg_clarke(float ia, float ib, float *alpha, float *beta)
{
    if (alpha != 0) { *alpha = ia; }
    if (beta != 0)  { *beta  = (ia + (2.0f * ib)) * FOC_INV_SQRT3; }
}

void alg_park(float alpha, float beta, float sin_t, float cos_t,
              float *d, float *q)
{
    if (d != 0) { *d =  (alpha * cos_t) + (beta * sin_t); }
    if (q != 0) { *q = -(alpha * sin_t) + (beta * cos_t); }
}

void alg_inv_park(float d, float q, float sin_t, float cos_t,
                  float *alpha, float *beta)
{
    if (alpha != 0) { *alpha = (d * cos_t) - (q * sin_t); }
    if (beta != 0)  { *beta  = (d * sin_t) + (q * cos_t); }
}

float alg_foc_step(alg_foc_t *f,
                   float ia, float ib, float ic,
                   float theta,
                   float id_ref, float iq_ref,
                   float ts,
                   float duty[3])
{
    float c;
    float s;
    float alpha;
    float beta;
    float vd;
    float vq;
    float v_alpha;
    float v_beta;
    float mag;
    float k;
    float scale;

    (void)ic;   /* ia + ib + ic = 0, Clarke only needs two phases */

    if ((f == 0) || (duty == 0)) {
        return 0.0f;
    }

    mid_trig_cos_sin(theta, &c, &s);

    alg_clarke(ia, ib, &alpha, &beta);
    alg_park(alpha, beta, s, c, &f->id, &f->iq);

    vd = alg_pi_step(&f->pi_d, id_ref - f->id, ts);
    vq = alg_pi_step(&f->pi_q, iq_ref - f->iq, ts);

    /* Voltage circle: keep the vector direction, scale the magnitude. */
    mag = sqrtf((vd * vd) + (vq * vq));
    if (mag > f->v_lim) {
        k = f->v_lim / mag;
        vd *= k;
        vq *= k;
        mag = f->v_lim;               /* report the clamped magnitude */
        /* The PI integrators were wound up against a limit they cannot reach;
         * clamping them also keeps the recovery free of overshoot. */
        f->pi_d.integ = vd - (f->pi_d.kp * (id_ref - f->id));
        f->pi_q.integ = vq - (f->pi_q.kp * (iq_ref - f->iq));
    }
    f->vd = vd;
    f->vq = vq;

    alg_inv_park(vd, vq, s, c, &v_alpha, &v_beta);

    /* Volts -> modulation index (m = 1 at 0.5*Vdc phase amplitude). */
    scale = 2.0f / f->vdc;
    f->m  = mag * scale;
    alg_svpwm_ab(v_alpha * scale, v_beta * scale, duty);

    return f->m;
}
