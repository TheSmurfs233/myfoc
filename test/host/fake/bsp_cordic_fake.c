/**
  ******************************************************************************
  * @file    bsp_cordic_fake.c
  * @brief   Host stand-in for the CORDIC wrapper.
  *
  * The host build always uses the LUT back end, but app_cmd.c references the
  * "cordic" debug command, so the symbols must exist.
  ******************************************************************************
  */

#include "bsp_cordic.h"

void bsp_cordic_init(void)
{
}

void bsp_cordic_cos_sin_raw(float theta, float *c, float *s,
                            int32_t *c_raw, int32_t *s_raw)
{
    (void)theta;

    if (c != 0) { *c = 1.0f; }
    if (s != 0) { *s = 0.0f; }
    if (c_raw != 0) { *c_raw = 0x7FFFFFFF; }
    if (s_raw != 0) { *s_raw = 0; }
}

void bsp_cordic_cos_sin(float theta, float *c, float *s)
{
    bsp_cordic_cos_sin_raw(theta, c, s, 0, 0);
}

uint8_t bsp_cordic_self_test(void)
{
    return 0u;
}
