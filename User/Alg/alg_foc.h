/**
  ******************************************************************************
  * @file    alg_foc.h
  * @brief   FOC current loop: Clarke, Park, two PI, circle limit, inverse Park.
  *
  * Layer: Alg (pure math, host-testable).
  *
  * Conventions (must match alg_svpwm.c):
  *   - Clarke is amplitude invariant: i_alpha = ia, i_beta = (ia + 2*ib)/sqrt(3)
  *   - The modulation index m is 1.0 at the SVPWM linear limit, i.e. the phase
  *     voltage amplitude is 0.5*Vdc. So m = 2*|V|/Vdc.
  *   - theta is the electrical angle from the encoder (rad), 0..2pi.
  ******************************************************************************
  */

#ifndef __ALG_FOC_H
#define __ALG_FOC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "alg_pi.h"

typedef struct {
    alg_pi_t pi_d;
    alg_pi_t pi_q;

    float    vdc;        /* bus voltage used for the modulation (V)      */
    float    m_max;      /* modulation limit (1.0 = linear limit)        */
    float    v_lim;      /* derived: 0.5 * vdc * m_max (V)               */

    /* S7.2 feedforward. All three default to 0 (off) so that the Alg layer
     * stays independent of board_cfg; the App configures them at init. */
    float    k_emf;      /* V per electrical rad/s                     */
    float    l_h;        /* phase inductance, H                        */
    float    ff_gain;    /* 0 = off, 1 = full                          */
    float    ff_lo;      /* FF off below this |w_e|, rad/s             */
    float    ff_hi;      /* FF fully on above this |w_e|, rad/s        */

    /* outputs, kept for logging */
    float    id;
    float    iq;
    float    vd;
    float    vq;
    float    m;
} alg_foc_t;

void alg_foc_init(alg_foc_t *f, float vdc, float m_max,
                  float kp, float ki, float v_lim);

void alg_foc_reset(alg_foc_t *f);

/* Enable/scale the feedforward. gain 0 disables it completely. */
void alg_foc_set_ff(alg_foc_t *f, float k_emf, float l_h, float gain);

/* Update the bus voltage used for the modulation and the voltage limit.
 * Call once per millisecond with the measured value (S7.5). */
void alg_foc_set_vdc(alg_foc_t *f, float vdc);

/* One 20 kHz control step.
 *   ia/ib/ic : measured phase currents (A)
 *   theta    : electrical angle from the encoder (rad)
 *   omega_e  : electrical angular speed (rad/s), signed
 *   id_ref/iq_ref : current references (A)
 *   ts       : loop period (s)
 *   duty[3]  : resulting phase duties, 0..1
 * Returns the modulation index that was used. */
float alg_foc_step(alg_foc_t *f,
                   float ia, float ib, float ic,
                   float theta, float omega_e,
                   float id_ref, float iq_ref,
                   float ts,
                   float duty[3]);

/* Building blocks, exposed for the host tests. */
void alg_clarke(float ia, float ib, float *alpha, float *beta);
void alg_park(float alpha, float beta, float sin_t, float cos_t,
              float *d, float *q);
void alg_inv_park(float d, float q, float sin_t, float cos_t,
                  float *alpha, float *beta);

#ifdef __cplusplus
}
#endif

#endif /* __ALG_FOC_H */
