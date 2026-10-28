/**
  ******************************************************************************
  * @file    alg_spd.c
  * @brief   Speed loop (see alg_spd.h).
  *
  * Layer: Alg.
  *
  *   iq_ref = PI(spd_ref - spd) + sign(spd_ref) * (ff_base + ff_slope*|spd_ref|)
  *
  * then clamped to +-out_max, and raised to at least "kick" in the direction of
  * the target while the target is not zero. The kick matters because of the
  * cogging detents: 0.03 A keeps the rotor turning but anything below ~0.10 A
  * may fail to start it from some positions (measured in S7.0).
  ******************************************************************************
  */

#include "alg_spd.h"

#define SPD_EPS   0.001f

void alg_spd_init(alg_spd_t *s, float kp, float ki, float out_max,
                  float ff_base, float ff_slope, float kick,
                  float kick_rpm)
{
    if (s == 0) {
        return;
    }

    alg_pi_init(&s->pi, kp, ki, -out_max, out_max);

    s->ff_base  = (ff_base  > 0.0f) ? ff_base  : 0.0f;
    s->ff_slope = (ff_slope > 0.0f) ? ff_slope : 0.0f;
    s->kick     = (kick     > 0.0f) ? kick     : 0.0f;
    s->kick_rpm = (kick_rpm > 0.0f) ? kick_rpm : 30.0f;
    s->out_max  = (out_max  > 0.0f) ? out_max  : 1.0f;

    alg_spd_reset(s);
}

void alg_spd_reset(alg_spd_t *s)
{
    if (s == 0) {
        return;
    }
    alg_pi_reset(&s->pi);
    s->spd_ref = 0.0f;
    s->out     = 0.0f;
    s->on      = 0u;
    s->kick_on = 0u;
    s->pi.out_max = s->out_max;
    s->pi.out_min = -s->out_max;
}

float alg_spd_step(alg_spd_t *s, float spd_rpm, float ts)
{
    float u;
    float ff;
    float ref;

    if (s == 0) {
        return 0.0f;
    }

    ref = s->spd_ref;

    if (s->on == 0u) {
        s->out = 0.0f;
        return 0.0f;
    }

    /* Target still zero: no PI, no feedforward, just stop asking for torque. */
    if ((ref < SPD_EPS) && (ref > -SPD_EPS)) {
        alg_pi_reset(&s->pi);
        s->out = 0.0f;
        return 0.0f;
    }

    u = alg_pi_step(&s->pi, ref - spd_rpm, ts);

    /* Friction feedforward: helps low speed a lot because the 10 ms speed
     * window makes the loop slow to react. */
    ff = s->ff_base + (s->ff_slope * ((ref > 0.0f) ? ref : -ref));
    u += (ref > 0.0f) ? ff : -ff;

    /* Start-up kick. It is deliberately NOT gated by the measured speed any
     * more: with a target near the old gate (50 rpm) the condition toggled
     * every loop, producing a rattle of kick-on / overshoot / kick-off /
     * undershoot. The App enables the kick for a short window after a target
     * change; a rotor that never moves is caught by the stall protection. */
    /* The kick must release as soon as the rotor moves: holding 0.20 A for
     * the whole window accelerates at ~690 rpm/s, which for a 30 rpm target
     * gave a 128 rpm peak (measured). So: kick window AND still not moving. */
    if ((s->kick_on != 0u) && (spd_rpm < 10.0f) && (spd_rpm > -10.0f)) {
        if (ref > 0.0f) {
            if (u <  s->kick) { u =  s->kick; }
        } else {
            if (u > -s->kick) { u = -s->kick; }
        }
    }

    if (u >  s->out_max) { u =  s->out_max; }
    if (u < -s->out_max) { u = -s->out_max; }

    s->out = u;
    return u;
}
