/**
  ******************************************************************************
  * @file    alg_svpwm.h
  * @brief   Three-phase duty generation from a rotating voltage vector.
  *
  * Layer: Alg. Pure math, no HAL, host-testable.
  *
  * Method: sinusoidal modulation plus min-max (zero-sequence) injection.
  * That is mathematically the same as seven-segment SVPWM but needs no sector
  * table: the common-mode term -(max+min)/2 is removed from the three duties,
  * which leaves every line-to-line voltage untouched while centring the three
  * duties around 0.5. Because of that the linear range reaches
  * 2/sqrt(3) = 1.1547 (six-step) instead of 1.0.
  ******************************************************************************
  */

#ifndef __ALG_SVPWM_H
#define __ALG_SVPWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Linear limit of the modulation index (six-step equivalent). */
#define ALG_SVPWM_M_MAX     1.1547f

/* Build the three duties for electrical angle theta (rad, any value) and
 * modulation index m (0 .. ALG_SVPWM_M_MAX).
 *
 * duty[] comes out in [0,1] with max+min == 1, so the waveform stays symmetric
 * about 0.5 and never clamps inside the linear range. A duty of 0.5 for all
 * three phases is the zero vector. */
void alg_svpwm(float theta, float m, float duty[3]);

/* Stationary-frame form: v_alpha/v_beta carry the modulation index directly. */
void alg_svpwm_ab(float v_alpha, float v_beta, float duty[3]);

#ifdef __cplusplus
}
#endif

#endif /* __ALG_SVPWM_H */
