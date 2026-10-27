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

    f->k_emf   = 0.0f;
    f->l_h     = 0.0f;
    f->ff_gain = 0.0f;
    f->ff_lo   = 100.0f;   /* 16 Hz electrical                   */
    f->ff_hi   = 300.0f;   /* 48 Hz electrical                   */

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

void alg_foc_set_ff(alg_foc_t *f, float k_emf, float l_h, float gain)
{
    if (f == 0) {
        return;
    }
    f->k_emf   = (k_emf > 0.0f) ? k_emf : 0.0f;
    f->l_h     = (l_h   > 0.0f) ? l_h   : 0.0f;
    f->ff_gain = (gain  > 0.0f) ? gain  : 0.0f;
    if (f->ff_gain > 1.0f) {
        f->ff_gain = 1.0f;
    }
}

void alg_foc_set_vdc(alg_foc_t *f, float vdc)
{
    if ((f == 0) || (vdc < 6.0f) || (vdc > 30.0f)) {
        return;                       /* implausible: keep the last value */
    }

    f->vdc   = vdc;
    f->v_lim = 0.5f * vdc * f->m_max;

    /* The PI limits follow the voltage limit, otherwise the integrators
     * would keep pushing towards a voltage the bridge cannot produce. */
    f->pi_d.out_max =  f->v_lim;
    f->pi_d.out_min = -f->v_lim;
    f->pi_q.out_max =  f->v_lim;
    f->pi_q.out_min = -f->v_lim;
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
                   float theta, float omega_e,
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

    /* S7.2 feedforward: tell the loop about the coupling and back-EMF terms
     * instead of letting the integrators discover them.
     *   vd = R*id + L*did/dt - w*L*iq
     *   vq = R*iq + L*diq/dt + w*L*id + w*psi
     * K_emf already includes psi, and omega_e is signed so a reversing motor
     * gets the right sign without a special case. */
    if ((f->ff_gain > 0.0f) && (omega_e != 0.0f)) {
        /* At low speed the encoder speed estimate (10 ms window) is noisy,
         * so the feedforward mostly injects that noise into vq: measured
         * ripple 14 -> 31 mA at 12 Hz electrical. Fade it in only once the
         * speed is high enough for the term to matter. */
        float g  = f->ff_gain;
        float av = (omega_e > 0.0f) ? omega_e : -omega_e;

        if (f->ff_hi > f->ff_lo) {
            float a = (av - f->ff_lo) / (f->ff_hi - f->ff_lo);

            if (a < 0.0f) { a = 0.0f; }
            if (a > 1.0f) { a = 1.0f; }
            g *= a;
        }

        if (g > 0.0f) {
            float wl = g * f->l_h * omega_e;

            vd -= wl * f->iq;
            vq += wl * f->id;
            vq += g * f->k_emf * omega_e;
        }
    }

    /* Voltage circle: keep the vector direction, scale the magnitude. */
    mag = sqrtf((vd * vd) + (vq * vq));
    if (mag > f->v_lim) {
        k = f->v_lim / mag;
        vd *= k;
        vq *= k;
        mag = f->v_lim;               /* report the clamped magnitude */
        /* Anti-windup for a VECTOR limit: scale both integrators by the same
         * factor k. Recomputing them per axis (the first attempt) let the two
         * axes fight each other, which swung the voltage vector direction by
         * tens of degrees while saturated (measured: vd jumping +/-3.5 V). */
        f->pi_d.integ *= k;
        f->pi_q.integ *= k;
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
