/**
  ******************************************************************************
  * @file    alg_ident.h
  * @brief   Motor parameter identification helpers (S10.7): R, L and KE.
  *
  * Layer: Alg (pure math, host-testable).
  ******************************************************************************
  */

#ifndef __ALG_IDENT_H
#define __ALG_IDENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Accumulator for the inductance fit.
 *
 * During a d-axis current step the machine obeys
 *      vd - R*id = L * didt
 * so with y = vd - R*id and d = didt the least-squares slope is
 *      L = sum(y*d) / sum(d*d)
 * Accumulated frame by frame at 20 kHz: no buffer, no division until the end.
 */
typedef struct {
    float    s_yd;   /* sum(y * d)   */
    float    s_dd;   /* sum(d * d)   */
    uint32_t n;      /* samples added */
} alg_ident_l_t;

void alg_ident_l_reset(alg_ident_l_t *s);

/* One 20 kHz sample.  didt is in A/s, vd/id from the FOC state. */
void alg_ident_l_add(alg_ident_l_t *s, float vd, float id, float didt, float r_ohm);

/* Returns 0 on success, 1 = not enough data, 2 = negative/zero inductance. */
uint8_t alg_ident_l_result(const alg_ident_l_t *s, float *l_h);

/* Two steady-state points: the line through them cancels the inverter drop
 * and the ADC offset, which a single point cannot.  Returns 0 if the two
 * currents are too close to give a usable slope. */
float alg_ident_r(float v1, float i1, float v2, float i2);

/* KE from a steady drag: |e| / |omega_e|, both electrical.  Returns 0 below
 * 1 rad/s. */
float alg_ident_ke(float e_abs, float omega_e);

#ifdef __cplusplus
}
#endif

#endif /* __ALG_IDENT_H */
