/**
  ******************************************************************************
  * @file    alg_ident.c
  * @brief   Motor parameter identification helpers (S10.7).
  *
  * Layer: Alg (pure math, host-testable).
  *
  * Why it looks like this (measured on the reference motor, S10.7):
  *   - R from ONE point carries the inverter drop (~0.1 V) and the ADC offset
  *     straight into the result: 0.3 A reads 2.6 ohm, 0.6 A reads 2.45 ohm.
  *     The slope of the line through two points drops both terms.
  *   - L from the closed-loop rise time needs the loop bandwidth, which moves
  *     with kp.  Fitting vd - R*id = L*didt uses only the two things the drive
  *     already measures (vd, id) and is independent of the tuning.
  *   - KE has no instrument in this build: |e| of the running observer divided
  *     by |omega_e| is the same constant the V/f+scope method measures.
  ******************************************************************************
  */

#include "alg_ident.h"
#include <math.h>

/* One frame of didt can be huge right at the step (V/L ~ 3000 A/s) but a frame
 * where the ADC glitched would dominate the whole sum, so clamp it. */
#define IDENT_DIDT_MAX   20000.0f
#define IDENT_L_MIN      1e-9f
/* Below this the current is settling, not ramping: the frame adds noise to
 * sum(d*d) without adding information about the slope. */
#define IDENT_DIDT_MIN   100.0f
/* Within 5% of the voltage limit means the loop asked for more than it could
 * deliver, so vd no longer describes the machine. */
#define IDENT_V_SAT      0.95f

void alg_ident_l_reset(alg_ident_l_t *s)
{
    s->s_yd = 0.0f;
    s->s_dd = 0.0f;
    s->n    = 0u;
}

void alg_ident_l_add(alg_ident_l_t *s, float vd, float id, float didt,
                     float r_ohm, float vlim)
{
    float d = didt;
    float y = vd - (r_ohm * id);

    if (fabsf(d) < IDENT_DIDT_MIN) {
        return;
    }
    if ((vlim > 0.0f) && (fabsf(vd) >= (vlim * IDENT_V_SAT))) {
        return;
    }

    if (d >  IDENT_DIDT_MAX) { d =  IDENT_DIDT_MAX; }
    if (d < -IDENT_DIDT_MAX) { d = -IDENT_DIDT_MAX; }

    s->s_yd += y * d;
    s->s_dd += d * d;
    s->n++;
}

uint8_t alg_ident_l_result(const alg_ident_l_t *s, float *l_h)
{
    if ((s->n < 2u) || (s->s_dd <= 0.0f)) {
        return 1u;
    }
    *l_h = s->s_yd / s->s_dd;
    if (*l_h <= IDENT_L_MIN) {
        return 2u;
    }
    return 0u;
}

float alg_ident_r(float v1, float i1, float v2, float i2)
{
    float di = i2 - i1;

    if (fabsf(di) < 0.05f) {
        return 0.0f;
    }
    return (v2 - v1) / di;
}

float alg_ident_ke(float e_abs, float omega_e)
{
    float w = fabsf(omega_e);

    if (w < 1.0f) {
        return 0.0f;
    }
    return e_abs / w;
}

uint8_t alg_ident_z(float v_c, float v_s, float i_c, float i_s, float omega,
                    float *l_h, float *r_ohm)
{
    float den = (i_c * i_c) + (i_s * i_s);

    if (den < (ALG_IDENT_I_MIN * ALG_IDENT_I_MIN)) {
        return 1u;
    }
    if (omega < 1.0f) {
        return 2u;
    }
    *r_ohm = ((v_c * i_c) + (v_s * i_s)) / den;
    *l_h   = (((v_c * i_s) - (v_s * i_c)) / den) / omega;
    if (*l_h <= IDENT_L_MIN) {
        return 3u;
    }
    return 0u;
}
