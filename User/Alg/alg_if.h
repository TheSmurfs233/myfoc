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
  *
  * Two soft edges keep the rotor from kicking (S10.8, #166):
  *   - the amplitude ramps in over ramp_s (300 ms by default in the App) so
  *     the pre-align pulls the rotor instead of yanking it;
  *   - at the end of the pre-align the vector crossfades from d to q over
  *     xfade_s instead of hopping 90 deg in one 50 us frame.
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
    float   ramp_s;     /* amplitude ramp 0 -> i_amp takes this long, s */
    float   xfade_s;    /* d -> q crossfade time, s (0 = instant)       */

    /* state */
    float   theta;      /* forced electrical angle, rad [0, 2pi)        */
    float   f_hz;       /* current electrical frequency, Hz             */
    float   f_target;   /* target electrical frequency, Hz              */
    float   i_now;      /* current amplitude after its own ramp, A      */
    float   xq;         /* 0 = vector on d, 1 = on q (crossfade state)  */
    float   t_align;    /* time spent in the pre-align phase, s         */
    uint8_t phase;      /* 0 = d-axis align (no torque), 1 = drag       */
    uint8_t run;
} alg_if_t;

void alg_if_init(alg_if_t *v, float acc_hz_s, float i_amp, float f_min_hz);

/* Pre-align duration: the vector is held on the d axis (no torque) so the
 * rotor is pulled to a known angle before the drag starts. This is what turns
 * the 180 deg ambiguity into a deterministic start. 0 disables it. */
void alg_if_set_align_time(alg_if_t *v, float seconds);

/* Amplitude ramp time (0 -> i_amp). Longer = gentler pull on the rotor.
 * Default 0.1 s; the App uses 0.3 s. */
void alg_if_set_ramp_time(alg_if_t *v, float seconds);

/* d -> q crossfade time at the end of the pre-align. 0 = old behaviour
 * (the whole amplitude hops to the q axis in one control period). */
void alg_if_set_xfade(alg_if_t *v, float seconds);

/* Start from standstill: angle 0, frequency f_min_hz, amplitude 0 (so the
 * caller gets a soft ramp even if it forgets to do one itself). */
void alg_if_start(alg_if_t *v);

/* New target frequency (Hz, signed). The ramp moves toward it at acc_hz_s. */
void alg_if_set_target(alg_if_t *v, float f_hz);

void alg_if_stop(alg_if_t *v);

/* Advance the ramp by ts seconds. Called at the control rate. */
void alg_if_step(alg_if_t *v, float ts);

/* Split the forced vector into its d and q components (id + iq = i_now, and
 * iq grows from 0 to i_now over xfade_s). Pointers may be null. */
void alg_if_apply(const alg_if_t *v, float *id, float *iq);

#ifdef __cplusplus
}
#endif

#endif /* __ALG_IF_H */
