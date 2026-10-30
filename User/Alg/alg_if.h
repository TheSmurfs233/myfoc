/**
  ******************************************************************************
  * @file    alg_if.h
  * @brief   Open-loop I/f ramp for encoder-less startup (S10.4).
  *
  * Layer: Alg.
  *
  * The problem: at standstill there is no back-EMF, so the observer has no
  * angle to report and the FOC cannot close the loop. The standard answer is
  * to drive a current vector at a frequency we choose and let the rotor be
  * dragged along ("I/f"), then hand over to the observer once it has locked.
  *
  * This module only produces that angle and frequency ramp. The caller applies
  * the current vector in the forced frame and, crucially, ramps the amplitude
  * up instead of stepping it: the open-loop alignment has a 180 deg electrical
  * ambiguity (measured: two stable positions 27.4 mechanical degrees apart), so
  * a gentle start lets the rotor slide into sync instead of fighting.
  ******************************************************************************
  */

#ifndef __ALG_IF_H
#define __ALG_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    /* configuration */
    float   acc_hz_s;   /* frequency ramp, Hz per s                     */
    float   i_amp;      /* current amplitude for the forced vector, A   */
    float   f_min_hz;   /* below this the ramp holds, Hz                */
    float   align_s;    /* d-axis pre-align time, s (0 = none)          */

    /* state */
    float   theta;      /* forced electrical angle, rad [0, 2pi)        */
    float   f_hz;       /* current electrical frequency, Hz             */
    float   f_target;   /* target electrical frequency, Hz              */
    float   i_now;      /* current amplitude after its own ramp, A      */
    float   t_align;    /* time spent in the pre-align phase, s         */
    uint8_t phase;      /* 0 = d-axis align (no torque), 1 = drag       */
    uint8_t run;
} alg_if_t;

void alg_if_init(alg_if_t *v, float acc_hz_s, float i_amp, float f_min_hz);

/* Pre-align duration: the vector is held on the d axis (no torque) so the
 * rotor is pulled to a known angle before the drag starts. This is what turns
 * the 180 deg ambiguity into a deterministic start. 0 disables it. */
void alg_if_set_align_time(alg_if_t *v, float seconds);

/* Start from standstill: angle 0, frequency f_min_hz, amplitude 0 (so the
 * caller gets a soft ramp even if it forgets to do one itself). */
void alg_if_start(alg_if_t *v);

/* New target frequency (Hz, signed). The ramp moves toward it at acc_hz_s. */
void alg_if_set_target(alg_if_t *v, float f_hz);

void alg_if_stop(alg_if_t *v);

/* Advance the ramp by ts seconds. Called at the control rate. */
void alg_if_step(alg_if_t *v, float ts);

#ifdef __cplusplus
}
#endif

#endif /* __ALG_IF_H */
