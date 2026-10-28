/**
  ******************************************************************************
  * @file    alg_pll.h
  * @brief   Second order phase-locked loop for angle/speed estimation.
  *
  * Layer: Alg.
  *
  * Why: the windowed estimator (sum of angle deltas over ENC_SPD_WINDOW_MS)
  * produces a 10 ms staircase and about 15 ms of delay, which caps the speed
  * loop bandwidth at 1-3 Hz. A PLL tracks the angle continuously instead:
  * the speed comes out every sample with 1-2 ms of delay, and the noise
  * rejection is set by the loop bandwidth rather than by a block average.
  *
  * Structure (type 2 loop):
  *
  *   err  = wrap(theta_meas - theta_hat)
  *   w   += ki * err * dt
  *   th  += (w + kp * err) * dt
  *
  * with kp = 2*zeta*wn and ki = wn^2 so that wn (Hz) and zeta are the only
  * two things worth tuning.
  ******************************************************************************
  */

#ifndef __ALG_PLL_H
#define __ALG_PLL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float   theta;    /* estimated angle, rad, wrapped to [0, 2pi)   */
    float   omega;    /* estimated speed, rad/s (same unit as theta) */
    float   kp;       /* proportional gain, 1/s                      */
    float   ki;       /* integral gain, 1/s^2                        */
    float   w_max;    /* speed clamp, rad/s                          */
    float   err;      /* last wrapped phase error, rad               */
    uint8_t locked;   /* 1 when |err| is small                       */
} alg_pll_t;

/* wn_hz: natural frequency, zeta: damping (0.707 is a good default),
 * w_max_rad_s: clamp for the estimated speed. */
void  alg_pll_init(alg_pll_t *p, float wn_hz, float zeta, float w_max_rad_s);

/* Retune without touching the state. */
void  alg_pll_set_bw(alg_pll_t *p, float wn_hz, float zeta);

/* Jump the estimated angle (keeps the speed estimate). */
void  alg_pll_reset(alg_pll_t *p, float theta0);

/* One update. theta_meas in rad (any range), dt in seconds.
 * Returns the estimated speed in rad/s. */
float alg_pll_step(alg_pll_t *p, float theta_meas, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __ALG_PLL_H */
