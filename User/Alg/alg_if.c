/**
  ******************************************************************************
  * @file    alg_if.c
  * @brief   Open-loop I/f ramp (S10.4).
  *
  * Layer: Alg. No hardware access, no floating point library beyond constants.
  ******************************************************************************
  */

#include "alg_if.h"

#define IF_TWO_PI   6.283185307179586f

void alg_if_init(alg_if_t *v, float acc_hz_s, float i_amp, float f_min_hz)
{
    if (v == 0) {
        return;
    }

    v->acc_hz_s = (acc_hz_s > 0.0f) ? acc_hz_s : 30.0f;
    v->i_amp    = (i_amp > 0.0f) ? i_amp : 0.4f;
    v->f_min_hz = (f_min_hz > 0.0f) ? f_min_hz : 2.0f;

    v->theta    = 0.0f;
    v->f_hz     = v->f_min_hz;
    v->f_target = v->f_min_hz;
    v->i_now    = 0.0f;
    v->run      = 0u;
}

void alg_if_start(alg_if_t *v)
{
    if (v == 0) {
        return;
    }

    v->theta    = 0.0f;
    v->f_hz     = v->f_min_hz;
    v->f_target = v->f_min_hz;
    v->i_now    = 0.0f;         /* the caller sees a soft start by design */
    v->run      = 1u;
}

void alg_if_set_target(alg_if_t *v, float f_hz)
{
    if (v == 0) {
        return;
    }

    v->f_target = f_hz;
    v->run      = 1u;
}

void alg_if_stop(alg_if_t *v)
{
    if (v == 0) {
        return;
    }

    v->run   = 0u;
    v->i_now = 0.0f;
    v->f_hz  = v->f_min_hz;
}

void alg_if_step(alg_if_t *v, float ts)
{
    float df;

    if ((v == 0) || (ts <= 0.0f) || (v->run == 0u)) {
        return;
    }

    /* ---- frequency ramp ---- */
    df = v->f_target - v->f_hz;
    if (df > 0.0f) {
        v->f_hz += v->acc_hz_s * ts;
        if (v->f_hz > v->f_target) {
            v->f_hz = v->f_target;
        }
    } else if (df < 0.0f) {
        v->f_hz -= v->acc_hz_s * ts;
        if (v->f_hz < v->f_target) {
            v->f_hz = v->f_target;
        }
    }

    /* ---- amplitude ramp: 20 ms worth per 100 ms, i.e. i_amp per 0.1 s ----
     * Kept here so a caller that applies i_now directly still gets a soft
     * start, which is what stops the 180 deg ambiguity from stalling it. */
    if (v->i_now < v->i_amp) {
        v->i_now += v->i_amp * ts * 10.0f;      /* 0 -> i_amp in 100 ms */
        if (v->i_now > v->i_amp) {
            v->i_now = v->i_amp;
        }
    } else if (v->i_now > v->i_amp) {
        v->i_now -= v->i_amp * ts * 10.0f;
        if (v->i_now < v->i_amp) {
            v->i_now = v->i_amp;
        }
    }

    /* ---- angle ---- */
    v->theta += IF_TWO_PI * v->f_hz * ts;
    while (v->theta >= IF_TWO_PI) { v->theta -= IF_TWO_PI; }
    while (v->theta <  0.0f)     { v->theta += IF_TWO_PI; }
}
