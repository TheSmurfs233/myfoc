/**
  ******************************************************************************
  * @file    alg_obs.c
  * @brief   Back-EMF observer for encoder-less FOC (S10.1).
  *
  * Layer: Alg.
  *
  * Shadow mode only: the caller decides whether the returned angle is used
  * for control. Nothing in this file touches the PWM path.
  ******************************************************************************
  */

#include "alg_obs.h"
#include "mid_trig.h"
#include <math.h>

#define OBS_TWO_PI      6.283185307179586f
#define OBS_DEG2RAD     0.0174532925199433f

void alg_obs_init(alg_obs_t *o, float r_ohm, float l_h,
                  float lpf_hz, float pll_wn_hz, float w_max_rad_s)
{
    if (o == 0) {
        return;
    }

    o->r_ohm = r_ohm;
    o->l_h   = l_h;
    o->w_max = (w_max_rad_s > 0.0f) ? w_max_rad_s : 2000.0f;

    /* First order low pass: k = 1 - exp(-2*pi*fc*ts). The caller passes the
     * cutoff in Hz and we fold it into a per-sample coefficient at step time,
     * so keep it as a time constant here. */
    if (lpf_hz > 0.0f) {
        o->lpf_k = 1.0f / (OBS_TWO_PI * lpf_hz);
    } else {
        o->lpf_k = 0.0f;        /* 0 = filtering disabled */
    }

    alg_pll_init(&o->pll, pll_wn_hz, 0.707f, o->w_max);
    alg_obs_reset(o, 0.0f);
}

void alg_obs_reset(alg_obs_t *o, float theta)
{
    if (o == 0) {
        return;
    }

    o->theta   = theta;
    o->omega_e = 0.0f;
    o->e_alpha = 0.0f;
    o->e_beta  = 0.0f;
    o->ed      = 0.0f;
    o->eq      = 0.0f;
    o->i_a_prev = 0.0f;
    o->i_b_prev = 0.0f;
    o->inited   = 0u;
    alg_pll_reset(&o->pll, theta);
}

void alg_obs_step(alg_obs_t *o, float v_alpha, float v_beta,
                  float i_alpha, float i_beta, float ts)
{
    float di_a;
    float di_b;
    float ea;
    float eb;
    float c;
    float s;
    float alpha;    /* filter coefficient for this step */
    float err;

    if ((o == 0) || (ts <= 0.0f)) {
        return;
    }

    if (o->inited == 0u) {
        o->i_a_prev = i_alpha;
        o->i_b_prev = i_beta;
        o->inited   = 1u;
        return;                      /* no di/dt on the first sample */
    }

    /* ---- back-EMF from the model: e = v - R*i - L*di/dt ---- */
    di_a = (i_alpha - o->i_a_prev) / ts;
    di_b = (i_beta  - o->i_b_prev) / ts;

    o->i_a_prev = i_alpha;
    o->i_b_prev = i_beta;

    ea = v_alpha - (o->r_ohm * i_alpha) - (o->l_h * di_a);
    eb = v_beta  - (o->r_ohm * i_beta)  - (o->l_h * di_b);

    /* ---- low pass (time constant form, recomputed per sample) ---- */
    if (o->lpf_k > 0.0f) {
        alpha = (o->lpf_k > ts) ? (ts / o->lpf_k) : 1.0f;
        if (alpha > 1.0f) { alpha = 1.0f; }

        o->e_alpha += alpha * (ea - o->e_alpha);
        o->e_beta  += alpha * (eb - o->e_beta);
    } else {
        o->e_alpha = ea;
        o->e_beta  = eb;
    }

    /* ---- rotate into the estimated frame ----
     * ed =  e_a*cos + e_b*sin = |e|*sin(theta_hat - theta)
     * eq = -e_a*sin + e_b*cos = |e|*cos(theta_hat - theta)
     * The PLL drives ed to zero; eq is the amplitude, useful as a health
     * indicator (should be ~omega_e * flux once locked). */
    mid_trig_cos_sin(o->theta, &c, &s);
    o->ed = ( o->e_alpha * c) + (o->e_beta * s);
    o->eq = (-o->e_alpha * s) + (o->e_beta * c);

    /* Sign: ed > 0 means theta_hat leads theta, so the loop needs -ed. */
    err = -o->ed;

    o->omega_e = alg_pll_step_err(&o->pll, err, ts);
    o->theta   = o->pll.theta;
}
