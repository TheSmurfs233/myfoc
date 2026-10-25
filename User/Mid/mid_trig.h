/**
  ******************************************************************************
  * @file    mid_trig.h
  * @brief   sin/cos front end: one interface, two back ends.
  *
  * Layer: Mid. The back end is chosen at compile time by TRIG_MODE in
  * board_cfg.h:
  *   TRIG_MODE_LUT    - 256-entry sine table + linear interpolation (default)
  *   TRIG_MODE_CORDIC - hardware CORDIC co-processor
  *
  * Keeping the table here (instead of in Alg) means alg_svpwm only depends on
  * this interface and the layer order Alg -> Mid -> Bsp stays intact.
  ******************************************************************************
  */

#ifndef __MID_TRIG_H
#define __MID_TRIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Prepare the selected back end. Returns 0 on success; on the CORDIC path a
 * non-zero value means the hardware self-test failed. */
uint8_t mid_trig_init(void);

/* cos and sin of theta (radians, any value). */
void mid_trig_cos_sin(float theta, float *c, float *s);

/* Name of the active back end, for the "trig" command / boot banner. */
const char *mid_trig_name(void);

#ifdef __cplusplus
}
#endif

#endif /* __MID_TRIG_H */
