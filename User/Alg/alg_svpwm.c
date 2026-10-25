/**
  ******************************************************************************
  * @file    alg_svpwm.c
  * @brief   Min-max injection SVPWM (see alg_svpwm.h).
  *
  * Layer: Alg.
  ******************************************************************************
  */

#include "alg_svpwm.h"
#include <math.h>

#define ALG_TWO_PI      6.283185307179586f
#define ALG_PHASE_120   2.094395102393195f      /* 2*pi/3 */

void alg_svpwm(float theta, float m, float duty[3])
{
    float hi;
    float lo;
    float off;
    uint8_t i;

    if (duty == 0) {
        return;
    }

    /* Wrap in a loop rather than fmodf(): this is called at 20kHz and the
     * angle accumulator already keeps theta near [0, 2pi). */
    while (theta >= ALG_TWO_PI) { theta -= ALG_TWO_PI; }
    while (theta < 0.0f)        { theta += ALG_TWO_PI; }

    duty[0] = 0.5f + 0.5f * m * cosf(theta);
    duty[1] = 0.5f + 0.5f * m * cosf(theta - ALG_PHASE_120);
    duty[2] = 0.5f + 0.5f * m * cosf(theta + ALG_PHASE_120);

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
