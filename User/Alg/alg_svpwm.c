/**
  ******************************************************************************
  * @file    alg_svpwm.c
  * @brief   Min-max injection SVPWM (see alg_svpwm.h).
  *
  * Layer: Alg. The trigonometry comes from mid_trig, so switching between the
  * lookup table and the CORDIC co-processor needs no change here.
  *
  * cos(theta - 120) and cos(theta + 120) are derived from the single (c, s)
  * pair with the exact rotation identity, which costs two multiplies each and
  * keeps the three duties consistent with each other:
  *     cos(t -+ 120) = -0.5*cos(t) +- (sqrt3/2)*sin(t)
  ******************************************************************************
  */

#include "alg_svpwm.h"
#include "mid_trig.h"

#define ALG_COS120      (-0.5f)
#define ALG_SIN120      (0.8660254f)        /* sqrt(3)/2 */

void alg_svpwm(float theta, float m, float duty[3])
{
    float c;
    float s;
    float hi;
    float lo;
    float off;
    uint8_t i;

    if (duty == 0) {
        return;
    }

    mid_trig_cos_sin(theta, &c, &s);

    duty[0] = 0.5f + 0.5f * m * c;
    duty[1] = 0.5f + 0.5f * m * ((ALG_COS120 * c) + (ALG_SIN120 * s));
    duty[2] = 0.5f + 0.5f * m * ((ALG_COS120 * c) - (ALG_SIN120 * s));

    hi = duty[0];
    lo = duty[0];
    for (i = 1u; i < 3u; i++) {
        if (duty[i] > hi) { hi = duty[i]; }
        if (duty[i] < lo) { lo = duty[i]; }
    }

    /* Remove the zero-sequence component: this is the injection that makes it
     * SVPWM instead of plain sinusoidal modulation. */
    off = 0.5f * (hi + lo) - 0.5f;
    for (i = 0u; i < 3u; i++) {
        duty[i] -= off;
    }
}
