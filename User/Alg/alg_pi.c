/**
  ******************************************************************************
  * @file    alg_pi.c
  * @brief   PI controller with clamping anti-windup (see alg_pi.h).
  *
  * Layer: Alg.
  ******************************************************************************
  */

#include "alg_pi.h"

void alg_pi_init(alg_pi_t *pi, float kp, float ki, float out_min, float out_max)
{
    if (pi == 0) {
        return;
    }
    pi->kp      = kp;
    pi->ki      = ki;
    pi->out_min = out_min;
    pi->out_max = out_max;
    pi->integ   = 0.0f;
}

void alg_pi_reset(alg_pi_t *pi)
{
    if (pi == 0) {
        return;
    }
    pi->integ = 0.0f;
}

float alg_pi_step(alg_pi_t *pi, float err, float ts)
{
    float out;

    if (pi == 0) {
        return 0.0f;
    }

    pi->integ += pi->ki * err * ts;
    out = (pi->kp * err) + pi->integ;

    /* Recompute the integrator when the output saturates: the classic
     * "clamping with back calculation" that keeps the loop linear as soon as
     * the voltage limit releases. */
    if (out > pi->out_max) {
        out = pi->out_max;
        pi->integ = out - (pi->kp * err);
    } else if (out < pi->out_min) {
        out = pi->out_min;
        pi->integ = out - (pi->kp * err);
    }

    return out;
}
