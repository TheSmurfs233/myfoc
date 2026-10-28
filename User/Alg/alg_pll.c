/**
  ******************************************************************************
  * @file    alg_pll.c
  * @brief   Second order PLL (see alg_pll.h).
  *
  * Layer: Alg.
  ******************************************************************************
  */

#include "alg_pll.h"

#define PLL_TWO_PI   6.283185307179586f
#define PLL_PI       3.141592653589793f

void alg_pll_init(alg_pll_t *p, float wn_hz, float zeta, float w_max_rad_s)
{
    if (p == 0) {
        return;
    }
    p->theta  = 0.0f;
    p->omega  = 0.0f;
    p->err    = 0.0f;
    p->locked = 0u;
    p->w_max  = (w_max_rad_s > 0.0f) ? w_max_rad_s : 1000.0f;

    alg_pll_set_bw(p, wn_hz, zeta);
}

void alg_pll_set_bw(alg_pll_t *p, float wn_hz, float zeta)
{
    float wn;

    if (p == 0) {
        return;
    }
    if (wn_hz <= 0.0f) {
        wn_hz = 100.0f;
    }
    if ((zeta <= 0.0f) || (zeta > 2.0f)) {
        zeta = 0.707f;
    }

    wn     = PLL_TWO_PI * wn_hz;
    p->kp  = 2.0f * zeta * wn;
    p->ki  = wn * wn;
}

void alg_pll_reset(alg_pll_t *p, float theta0)
{
    if (p == 0) {
        return;
    }
    p->theta  = theta0;
    p->err    = 0.0f;
    p->locked = 0u;
}

float alg_pll_step(alg_pll_t *p, float theta_meas, float dt)
{
    float err;

    if ((p == 0) || (dt <= 0.0f)) {
        return (p != 0) ? p->omega : 0.0f;
    }

    /* Phase error, shortest way around the circle. */
    err = theta_meas - p->theta;
    while (err >  PLL_PI) { err -= PLL_TWO_PI; }
    while (err < -PLL_PI) { err += PLL_TWO_PI; }

    p->err = err;

    /* Type 2 loop: the integrator carries the speed, the proportional term
     * pulls the angle in. */
    p->omega += p->ki * err * dt;
    if (p->omega >  p->w_max) { p->omega =  p->w_max; }
    if (p->omega < -p->w_max) { p->omega = -p->w_max; }

    p->theta += (p->omega + p->kp * err) * dt;
    while (p->theta >= PLL_TWO_PI) { p->theta -= PLL_TWO_PI; }
    while (p->theta <  0.0f)       { p->theta += PLL_TWO_PI; }

    p->locked = (err < 0.15f) && (err > -0.15f) ? 1u : 0u;

    return p->omega;
}
