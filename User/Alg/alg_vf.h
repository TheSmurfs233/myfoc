/**
  ******************************************************************************
  * @file    alg_vf.h
  * @brief   Open-loop V/f ramp: electrical angle, frequency ramp, V/f curve.
  *
  * Layer: Alg (depends on Cfg only). Host-testable.
  *
  * The step is meant to be called once per ADC frame (20kHz) with a fixed
  * ts, so the angle advances by exactly 2*pi*f*ts per call and the ramp is
  * deterministic. Direction is a software phase-sequence reversal:
  * the returned angle is negated, which swaps phases B and C (equivalent to
  * swapping two motor wires) without touching the ADC channel mapping.
  ******************************************************************************
  */

#ifndef __ALG_VF_H
#define __ALG_VF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float   theta;       /* electrical angle, wrapped to [0, 2pi)          */
    float   f_hz;        /* present electrical frequency                   */
    float   f_target;    /* frequency the ramp is heading to               */
    float   acc_hz_s;    /* ramp rate, Hz per second (always > 0)          */
    float   m_scale;     /* current foldback factor, VF_M_SCALE_MIN .. 1   */
    uint8_t dir;         /* 0 = normal, 1 = reversed phase sequence        */
    uint8_t on;          /* 1 = modulating                                 */
} alg_vf_t;

void  alg_vf_init(alg_vf_t *vf);

/* Start modulating towards hz (clamped to VF_F_MAX). Resets the foldback. */
void  alg_vf_start(alg_vf_t *vf, float hz);

/* Stop modulating and drop the frequency to 0 (next start ramps from 0). */
void  alg_vf_stop(alg_vf_t *vf);

/* Advance one step of ts seconds; returns the angle to feed to alg_svpwm()
 * (already negated when dir == 1). */
float alg_vf_step(alg_vf_t *vf, float ts);

/* Modulation index for the present frequency: boost + k*f, clamped to
 * VF_M_MAX, then scaled by the current foldback. */
float alg_vf_mod_index(const alg_vf_t *vf);

#ifdef __cplusplus
}
#endif

#endif /* __ALG_VF_H */
