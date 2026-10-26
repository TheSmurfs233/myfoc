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
    uint8_t  on;
} alg_spd_t;

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
