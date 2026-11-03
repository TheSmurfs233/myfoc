/**
  ******************************************************************************
  * @file    alg_if.c
  * @brief   Open-loop I/f ramp (S10.4), with soft amplitude ramp and d -> q
  *          crossfade (S10.8, #166).
  *
  * Layer: Alg. No hardware access, no floating point library beyond constants.
  ******************************************************************************
  */

#include "alg_if.h"

#define IF_TWO_PI       6.283185307179586f
#define IF_RAMP_DEF_S   0.1f        /* default amplitude ramp: 100 ms */

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
    v->xq       = 1.0f;
    v->t_align  = 0.0f;
    v->phase    = 0u;
    v->align_s  = 0.0f;
    v->ramp_s   = IF_RAMP_DEF_S;
    v->xfade_s  = 0.0f;         /* instant: keeps the original behaviour */
    v->run      = 0u;
}

void alg_if_set_align_time(alg_if_t *v, float seconds)
{
    if (v == 0) {
        return;
    }
    v->align_s = (seconds > 0.0f) ? seconds : 0.0f;
}

void alg_if_set_ramp_time(alg_if_t *v, float seconds)
{
    if (v == 0) {
        return;
    }
    v->ramp_s = (seconds > 0.0f) ? seconds : IF_RAMP_DEF_S;
}

void alg_if_set_xfade(alg_if_t *v, float seconds)
{
    if (v == 0) {
        return;
    }
    v->xfade_s = (seconds > 0.0f) ? seconds : 0.0f;
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
    v->xq       = 0.0f;         /* then crossfades d -> q in the drag phase */
    v->t_align  = 0.0f;
    v->phase    = (v->align_s > 0.0f) ? 0u : 1u;
    v->run      = 1u;
}

void alg_if_set_target(alg_if_t *v, float f_hz)
{
    if (v == 0) {
        return;
    }

    /* Only move the target. Do NOT touch run: this used to set it to 1, so any
     * later call resurrected a ramp that the hand-over had already stopped
     * (measured 06:00: the I/f came back and hijacked the current references,
     * pinning the drive at 600 rpm with the drag current). */
    /* #170: alg_if_start seeds f_hz with the POSITIVE minimum (2 Hz). In
     * reverse the drag then had to ramp down through zero (67 ms of forward
     * rotation) before it moved the right way, which kicked the rotor. If the
     * frame is still inside 1.5 * f_min and the new target asks for the other
     * direction, snap to the signed minimum instead. */
    if ((v->run != 0u) && (v->phase == 1u) && (f_hz != 0.0f) &&
        ((v->f_hz > 0.0f) != (f_hz > 0.0f)) &&
        (v->f_hz <  (1.5f * v->f_min_hz)) &&
        (v->f_hz > -(1.5f * v->f_min_hz))) {
        v->f_hz = (f_hz > 0.0f) ? v->f_min_hz : -v->f_min_hz;
    }
    v->f_target = f_hz;
}

void alg_if_stop(alg_if_t *v)
{
    if (v == 0) {
        return;
    }

    v->run     = 0u;
    v->i_now   = 0.0f;
    v->xq      = 0.0f;
    v->f_hz    = v->f_min_hz;
    v->phase   = 0u;
    v->t_align = 0.0f;
}

/* Amplitude ramp: reach i_amp in ramp_s seconds (both directions). */
static void if_ramp_amp(alg_if_t *v, float ts)
{
    float step = v->i_amp * ts / v->ramp_s;

    if (v->i_now < v->i_amp) {
        v->i_now += step;
        if (v->i_now > v->i_amp) { v->i_now = v->i_amp; }
    } else if (v->i_now > v->i_amp) {
        v->i_now -= step;
        if (v->i_now < v->i_amp) { v->i_now = v->i_amp; }
    }
}

/* d -> q crossfade. A hard 90 deg hop put the whole 0.6 A on the q axis in one
 * 50 us frame = 17 mN.m step = ~1800 rpm/s on this rotor; that is the "kick"
 * seen at start-up (measured 2026-11-01, #166). */
static void if_ramp_xfade(alg_if_t *v, float ts)
{
    if (v->xq >= 1.0f) {
        return;
    }

    if (v->xfade_s <= 0.0f) {
        v->xq = 1.0f;
        return;
    }

    v->xq += ts / v->xfade_s;
    if (v->xq > 1.0f) {
        v->xq = 1.0f;
    }
}

void alg_if_step(alg_if_t *v, float ts)
{
    float df;

    if ((v == 0) || (ts <= 0.0f) || (v->run == 0u)) {
        return;
    }

    /* ---- d-axis pre-align: hold the angle, no frequency ramp yet ---- */
    if (v->phase == 0u) {
        v->t_align += ts;

        /* amplitude ramps up, angle stays at 0, whole vector still on d */
        if_ramp_amp(v, ts);
        v->xq = 0.0f;

        if (v->t_align >= v->align_s) {
            v->phase = 1u;
            v->t_align = 0.0f;
            v->f_hz = v->f_min_hz;
            /* the crossfade starts here, not at its end */
            v->xq = 0.0f;
        }
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

    /* ---- amplitude ramp (kept here so a caller that applies i_now directly
     * still gets a soft start, which is what stops the 180 deg ambiguity from
     * stalling it) ---- */
    if_ramp_amp(v, ts);

    /* ---- d -> q crossfade ---- */
    if_ramp_xfade(v, ts);

    /* ---- angle ---- */
    v->theta += IF_TWO_PI * v->f_hz * ts;
    while (v->theta >= IF_TWO_PI) { v->theta -= IF_TWO_PI; }
    while (v->theta <  0.0f)     { v->theta += IF_TWO_PI; }
}

void alg_if_apply(const alg_if_t *v, float *id, float *iq)
{
    float x;
    float i;

    if (v == 0) {
        if (id != 0) { *id = 0.0f; }
        if (iq != 0) { *iq = 0.0f; }
        return;
    }

    x = v->xq;
    if (x < 0.0f) { x = 0.0f; }
    if (x > 1.0f) { x = 1.0f; }
    i = v->i_now;

    if (id != 0) { *id = i * (1.0f - x); }
    if (iq != 0) { *iq = i * x; }
}
