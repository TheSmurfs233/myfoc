/**
  ******************************************************************************
  * @file    bsp_cordic.h
  * @brief   CORDIC co-processor wrapper for sin/cos.
  *
  * Layer: Bsp.
  ******************************************************************************
  */

#ifndef __BSP_CORDIC_H
#define __BSP_CORDIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Enable the CORDIC clock and program it for cosine + 24 iterations. */
void bsp_cordic_init(void);

/* cos and sin of theta (radians, any value). Uses the hardware Q1.31 path:
 * the angle is wrapped into [-pi, pi] first, results come back as Q1.31. */
void bsp_cordic_cos_sin(float theta, float *c, float *s);

/* Same call, but also hands back the raw 32-bit results exactly as read from
 * RDATA. Used by the "cordic" debug command to work out the real Q format. */
void bsp_cordic_cos_sin_raw(float theta, float *c, float *s,
                            int32_t *c_raw, int32_t *s_raw);

/* Self-check against five known angles. Returns 0 when the results match,
 * non-zero when the Q format assumption is wrong (misconfiguration). */
uint8_t bsp_cordic_self_test(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CORDIC_H */
