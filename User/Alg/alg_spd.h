/**
  ******************************************************************************
  * @file    alg_spd.h
  * @brief   Speed loop: PI + friction feedforward + start-up kick.
  *
  * Layer: Alg (pure math, host-testable).
  *
  * Measured on hardware (S7.0, two runs):
  *   - breakaway current 0.09 .. >0.10 A depending on the cogging detent the
  *     rotor happens to rest on, so a start-up kick of 0.15 A is used to make
  *     sure the rotor always breaks free;
  *   - the motor still turns down to 0.03 A and stops at 0.02 A;
  *   - friction fits iq_ff = 0.015 + 8e-5 * rpm within 0.01 A.
  *
  * The speed feedback is a 10 ms moving average (mid_enc), so the loop is
  * intended to run at 1 kHz and its bandwidth must stay well below 20 Hz.
  ******************************************************************************
  */

#ifndef __ALG_SPD_H
#define __ALG_SPD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "alg_pi.h"

typedef struct {
    alg_pi_t pi;          /* speed PI, output in amperes            */
    float    ff_base;     /* friction feedforward, A                */
    float    ff_slope;    /* A per rpm                              */
    float    kick;        /* breakaway current, A                    */
    float    kick_rpm;    /* only apply the kick below this speed    */
    float    out_max;     /* current limit, A                       */
    float    spd_ref;     /* rpm                                    */
    float    out;         /* iq reference produced last step, A     */
    float    ff_acc;      /* A per rad/s^2, acceleration feedforward */
    float    ref_prev;    /* previous reference, for the derivative  */
    uint8_t  on;
    uint8_t  kick_on;    /* set by the App for a short start-up window */
    float    gain_lo_rpm;   /* full gains below this speed            */
    float    gain_hi_rpm;   /* reduced gains from this speed up       */
    float    gain_hi_scale; /* gain multiplier at and above hi        */
    float    lpf_lo_rpm;    /* filter band start                       */
    float    lpf_hi_rpm;    /* filter band end                         */
    float    lpf_lo_k;      /* feedback filter coefficient below LO    */
    float    lpf_hi_k;      /* feedback filter coefficient above HI    */
    float    spd_f;         /* filtered feedback, rpm                  */
} alg_spd_t;

/* Speed dependent gain scaling (see board_cfg). */
void  alg_spd_set_sched(alg_spd_t *s, float lo_rpm, float hi_rpm, float hi_scale);
/* Speed dependent feedback filter (1.0 = off). */
void  alg_spd_set_lpf(alg_spd_t *s, float lo_rpm, float hi_rpm, float lo_k, float hi_k);

void  alg_spd_init(alg_spd_t *s, float kp, float ki, float out_max,
                   float ff_base, float ff_slope, float kick,
                   float kick_rpm);
void  alg_spd_reset(alg_spd_t *s);

/* spd_rpm = measured mechanical speed, ts = seconds since the last call.
 * Returns the iq reference (already limited). */
float alg_spd_step(alg_spd_t *s, float spd_rpm, float ts);

#ifdef __cplusplus
}
#endif

#endif /* __ALG_SPD_H */
