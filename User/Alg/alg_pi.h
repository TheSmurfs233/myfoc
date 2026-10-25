/**
  ******************************************************************************
  * @file    alg_pi.h
  * @brief   PI controller with clamping anti-windup.
  *
  * Layer: Alg (pure math, host-testable).
  *
  * Used by the d and q current loops. The output is clamped to
  * [out_min, out_max] and the integrator is recomputed on clamping so it can
  * never wind up while the voltage limit is active (the current loop spends a
  * lot of time saturated during a step, so plain clamping is not enough).
  ******************************************************************************
  */

#ifndef __ALG_PI_H
#define __ALG_PI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float kp;        /* proportional gain, out per error               */
    float ki;        /* integral gain, out per (error * second)        */
    float out_min;
    float out_max;
    float integ;     /* integrator state (already includes ki)         */
} alg_pi_t;

void  alg_pi_init(alg_pi_t *pi, float kp, float ki, float out_min, float out_max);
void  alg_pi_reset(alg_pi_t *pi);

/* err = reference - feedback; ts in seconds. Returns the clamped output. */
float alg_pi_step(alg_pi_t *pi, float err, float ts);

#ifdef __cplusplus
}
#endif

#endif /* __ALG_PI_H */
