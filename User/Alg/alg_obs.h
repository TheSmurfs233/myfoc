/**
  ******************************************************************************
  * @file    alg_obs.h
  * @brief   Back-EMF observer for encoder-less FOC (S10.1 infrastructure).
  *
  * Layer: Alg.
  *
  * Principle (see docs/34):
  *
  *   e_ab = v_ab - R*i_ab - L*di_ab/dt          <- back-EMF estimate
  *   ed   =  e_alpha*cos(theta) + e_beta*sin(theta)   <- angle error
  *   eq   = -e_alpha*sin(theta) + e_beta*cos(theta)   <- |e| when locked
  *
  * ed is fed to a PLL (sign inverted) that drives it to zero; theta then
  * tracks the rotor. eq should settle near omega_e * flux.
  *
  * Shadow mode: this module only computes; nothing here touches the control
  * path until S10.5 selects the observer as the angle source.
  ******************************************************************************
  */

#ifndef __ALG_OBS_H
#define __ALG_OBS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "alg_pll.h"

typedef struct {
    /* configuration */
    float   r_ohm;
    float   l_h;
    float   lpf_hz;     /* back-EMF low pass cutoff, 0 = off            */
    float   lpf_k;      /* derived: time constant, s                    */
    float   w_max;      /* speed clamp for the PLL, rad/s              */
    uint8_t comp;       /* 1 = phase-lead compensation on               */
    float   delay_s;    /* extra delay folded into the compensation     */

    /* state */
    alg_pll_t pll;
    float   theta;      /* estimated electrical angle, rad 0..2pi     */
    float   omega_e;    /* estimated electrical speed, rad/s          */
    float   e_alpha;    /* filtered back-EMF, V                       */
    float   e_beta;
    float   ed;         /* back-EMF in the estimated frame, V         */
    float   eq;
    float   i_a_prev;   /* previous alpha/beta current for di/dt      */
    float   i_b_prev;
    uint8_t inited;
} alg_obs_t;

/* lpf_hz: first order filter on e_alpha/e_beta (0 disables).
 * pll_wn_hz / w_max: PLL bandwidth and speed clamp. */
void alg_obs_init(alg_obs_t *o, float r_ohm, float l_h,
                  float lpf_hz, float pll_wn_hz, float w_max_rad_s);

void alg_obs_reset(alg_obs_t *o, float theta);

/* One update at the control rate. v/i are in the stationary alpha-beta frame,
 * ts in seconds. */
void alg_obs_step(alg_obs_t *o, float v_alpha, float v_beta,
                  float i_alpha, float i_beta, float ts);

#ifdef __cplusplus
}
#endif

#endif /* __ALG_OBS_H */
