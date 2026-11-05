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

/* One 20 kHz sample.  didt is in A/s, vd/id from the FOC state.
 *
 * Frames where the loop was against its voltage limit, or where the current
 * barely moved, carry no usable slope and are skipped.  With a 0.5 A step the
 * term L*didt is about 5 V against a 5.7 V budget, so those frames used to
 * bias the fit low (0.29 mH against 0.5 mH declared).  Pass vlim <= 0 to
 * disable the saturation check. */
void alg_ident_l_add(alg_ident_l_t *s, float vd, float id, float didt,
                     float r_ohm, float vlim);

/* Returns 0 on success, 1 = not enough data, 2 = negative/zero inductance. */
uint8_t alg_ident_l_result(const alg_ident_l_t *s, float *l_h);

/* Two steady-state points: the line through them cancels the inverter drop
 * and the ADC offset, which a single point cannot.  Returns 0 if the two
 * currents are too close to give a usable slope. */
float alg_ident_r(float v1, float i1, float v2, float i2);

/* KE from a steady drag: |e| / |omega_e|, both electrical.  Returns 0 below
 * 1 rad/s. */
float alg_ident_ke(float e_abs, float omega_e);

/* Series impedance of one injection frequency.  The caller hands over the
 * sine and cosine coefficients of v and i at that frequency, i.e.
 *   x(t) = Xc*cos(wt) + Xs*sin(wt)
 * and gets  Z = R + jwL  solved out of Z = V/I:
 *   R = (Vc*Ic + Vs*Is) / |I|^2,  L = ((Vc*Is - Vs*Ic) / |I|^2) / w
 * Returns 0 on success, 1 when |I| is too small to divide by, 2 when omega
 * is not usable, 3 when the fit gives a negative inductance. */
uint8_t alg_ident_z(float v_c, float v_s, float i_c, float i_s, float omega,
                    float *l_h, float *r_ohm);

/* Below this much current an impedance has no meaning; the injection
 * amplitude is expected to be several times this. */
#define ALG_IDENT_I_MIN  0.01f


#ifdef __cplusplus
}
#endif

#endif /* __ALG_IDENT_H */
