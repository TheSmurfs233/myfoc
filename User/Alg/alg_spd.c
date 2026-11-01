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
    s->gain_lo_rpm   = 300.0f;
    s->gain_hi_rpm   = 600.0f;
    s->gain_hi_scale = 1.0f;     /* no scheduling unless configured */
    s->lpf_lo_rpm    = 60.0f;
    s->lpf_hi_rpm    = 180.0f;
    s->lpf_lo_k      = 1.0f;
    s->lpf_hi_k      = 1.0f;
    s->spd_f         = 0.0f;
    s->out_max  = (out_max  > 0.0f) ? out_max  : 1.0f;
    s->ff_acc   = 0.0f;
    s->acc_max  = 0.0f;
    s->ref_prev = 0.0f;

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
    s->ref_prev = 0.0f;
    s->kick_on = 0u;
    s->pi.out_max = s->out_max;
    s->pi.out_min = -s->out_max;
}

void alg_spd_set_sched(alg_spd_t *s, float lo_rpm, float hi_rpm, float hi_scale)
{
    if (s == 0) {
        return;
    }
    s->gain_lo_rpm   = (lo_rpm > 0.0f) ? lo_rpm : 1.0f;
    s->gain_hi_rpm   = (hi_rpm > s->gain_lo_rpm) ? hi_rpm : (s->gain_lo_rpm + 1.0f);
    s->gain_hi_scale = (hi_scale >= 0.0f) ? hi_scale : 1.0f;
    if (s->gain_hi_scale > 1.0f) {
        s->gain_hi_scale = 1.0f;
    }
}

void alg_spd_set_lpf(alg_spd_t *s, float lo_rpm, float hi_rpm,
                     float lo_k, float hi_k)
{
    if (s == 0) {
        return;
    }
    s->lpf_lo_rpm = (lo_rpm > 0.0f) ? lo_rpm : 1.0f;
    s->lpf_hi_rpm = (hi_rpm > s->lpf_lo_rpm) ? hi_rpm : (s->lpf_lo_rpm + 1.0f);
    s->lpf_lo_k = (lo_k > 1.0f) ? 1.0f : ((lo_k < 0.01f) ? 0.01f : lo_k);
    s->lpf_hi_k = (hi_k > 1.0f) ? 1.0f : ((hi_k < 0.01f) ? 0.01f : hi_k);
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

    /* Gain scheduling + feedback filter, both speed dependent. The 10 ms
     * speed window gives a staircase of 7-10 rpm at 300 rpm; the loop would
     * chase it and turn cogging into current ripple, so at high speed the
     * feedback is low-pass filtered and the gains are reduced. */
    {
        float av = (spd_rpm < 0.0f) ? -spd_rpm : spd_rpm;
        float sc = 1.0f;
        float kf = s->lpf_lo_k;
        float t  = 0.0f;

        if (av > s->gain_lo_rpm) {
            t = (av - s->gain_lo_rpm) / (s->gain_hi_rpm - s->gain_lo_rpm);
            if (t > 1.0f) { t = 1.0f; }
        }

        if ((s->gain_hi_scale < 1.0f) && (t > 0.0f)) {
            sc = 1.0f + t * (s->gain_hi_scale - 1.0f);
        }
        if (s->lpf_hi_k < 1.0f) {
            float tf = 0.0f;

            if (av > s->lpf_lo_rpm) {
                tf = (av - s->lpf_lo_rpm) / (s->lpf_hi_rpm - s->lpf_lo_rpm);
                if (tf > 1.0f) { tf = 1.0f; }
            }
            kf = s->lpf_lo_k + tf * (s->lpf_hi_k - s->lpf_lo_k);
        }

        /* First order filter on the feedback (no-op when kf == 1). */
        if (kf < 1.0f) {
            s->spd_f += kf * (spd_rpm - s->spd_f);
        } else {
            s->spd_f = spd_rpm;
        }

        if (sc < 1.0f) {
            float kp = s->pi.kp;
            float ki = s->pi.ki;

            s->pi.kp = kp * sc;
            s->pi.ki = ki * sc;
            u = alg_pi_step(&s->pi, ref - s->spd_f, ts);
            s->pi.kp = kp;
            s->pi.ki = ki;
        } else {
            u = alg_pi_step(&s->pi, ref - s->spd_f, ts);
        }
    }
    /* Friction feedforward: helps low speed a lot because the 10 ms speed
     * window makes the loop slow to react. */
    ff = s->ff_base + (s->ff_slope * ((ref > 0.0f) ? ref : -ref));

    /* Acceleration feedforward: J/Kt * d(ref)/dt. Following the 600 rpm/s
     * ramp needs about 0.2 A; without this the integrator had to build it
     * and the loop overshot (measured: a 100 rpm step reached 196 rpm). */
    if (s->ff_acc > 0.0f) {
        float alpha = ((ref - s->ref_prev) / ts) * (6.2831853f / 60.0f);

        /* #167: the reference is ramped in the App's 1 ms task while this loop
         * has its own 1 kHz tick. The two are not phase locked, so now and then
         * one call catches two ramp steps and the derivative doubles: measured
         * +0.20 A for a single sample every 40 ms during a 600 rpm/s ramp.
         * Clamping to the configured ramp rate removes the spike without
         * touching the steady-state feedforward. */
        if ((s->acc_max > 0.0f) && (alpha > s->acc_max)) {
            alpha = s->acc_max;
        }
        if ((s->acc_max > 0.0f) && (alpha < -s->acc_max)) {
            alpha = -s->acc_max;
        }
        u += s->ff_acc * alpha;
    }
    s->ref_prev = ref;
    u += (ref > 0.0f) ? ff : -ff;

    /* Start-up kick. It is deliberately NOT gated by the measured speed any
     * more: with a target near the old gate (50 rpm) the condition toggled
     * every loop, producing a rattle of kick-on / overshoot / kick-off /
     * undershoot. The App enables the kick for a short window after a target
     * change; a rotor that never moves is caught by the stall protection. */
    /* The kick must release as soon as the rotor moves: holding 0.20 A for
     * the whole window accelerates at ~690 rpm/s, which for a 30 rpm target
     * gave a 128 rpm peak (measured). So: kick window AND still not moving.
     *
     * The gate also scales with the target: with a fixed 10 rpm gate a
     * 100 rpm step overshot by 20% because the kick kept accelerating for
     * the whole window. */
    {
        float gate = (0.10f * ((ref > 0.0f) ? ref : -ref)) + 5.0f;

        if (gate < 10.0f) {
            gate = 10.0f;
        }

        if ((s->kick_on != 0u) && (spd_rpm < gate) && (spd_rpm > -gate)) {
            if (ref > 0.0f) {
                if (u <  s->kick) { u =  s->kick; }
            } else {
                if (u > -s->kick) { u = -s->kick; }
            }
        }
    }

    if (u >  s->out_max) { u =  s->out_max; }
    if (u < -s->out_max) { u = -s->out_max; }

    s->out = u;
    return u;
}

void alg_spd_preload(alg_spd_t *s, float out_a)
{
    float ff;

    if (s == 0) {
        return;
    }

    ff = s->ff_base + (s->ff_slope * ((s->spd_ref > 0.0f) ? s->spd_ref : -s->spd_ref));

    /* The step function adds +-ff on top of the PI output, so only the rest
     * has to come from the integrator. The error is zero at the moment of the
     * transfer (reference and feedback both start at the current speed), hence
     * out = integ + ff. */
    s->pi.integ = out_a - ((s->spd_ref > 0.0f) ? ff : -ff);
    s->ref_prev = s->spd_ref;
    s->spd_f    = s->spd_ref;
    s->kick_on  = 0u;
    s->out      = out_a;
}
