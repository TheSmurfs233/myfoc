/**
  ******************************************************************************
  * @file    bsp_cordic.c
  * @brief   CORDIC co-processor wrapper (see bsp_cordic.h).
  *
  * Layer: Bsp.
  *
  * The G4 CORDIC computes cos and sin in one pass: 1 argument (the angle) and
  * 2 results, both in Q1.31. The angle itself is also Q1.31, with [-pi, pi]
  * mapped onto the full int32 range, so the conversion is theta * 2^31 / pi.
  * Out-of-range angles are not wrapped by the hardware, hence the software
  * wrap below.
  ******************************************************************************
  */

#include "bsp_cordic.h"
#include "stm32g4xx_hal.h"
#include <math.h>

#define CORDIC_PI            3.14159265358979f
#define CORDIC_TWO_PI        6.28318530717959f
#define CORDIC_Q31_PER_RAD   (2147483648.0f / CORDIC_PI)     /* 6.8357e8 */
#define CORDIC_Q31_TO_FLOAT  (1.0f / 2147483648.0f)

/* PRECISION field is 4 bits wide and holds (iterations - 1), so the maximum is
 * 15 -> 16 iterations. 16 iterations give ~1.5e-5 resolution, five times better
 * than the 256-entry table (7.5e-5). NOTE: 23 << 4 would overflow into the
 * SCALE field (bit 8) and silently scale every result by 0.5. */
#define CORDIC_PREC_16       ((uint32_t)(15u << CORDIC_CSR_PRECISION_Pos))

/* CSR.FUNC = 0 selects the cosine function (cos + sin pair); the HAL header
 * that names it is not part of this project, so use the raw value. */
#define CORDIC_FUNC_COSINE   0u

void bsp_cordic_init(void)
{
    __HAL_RCC_CORDIC_CLK_ENABLE();

    /* FUNC = cosine, 1 argument, 2 results, 16 iterations, 32-bit Q1.31,
     * scale 1.0 (all other fields stay 0). */
    CORDIC->CSR = (CORDIC_FUNC_COSINE | CORDIC_PREC_16 | CORDIC_CSR_NRES);
}

void bsp_cordic_cos_sin_raw(float theta, float *c, float *s,
                            int32_t *c_raw, int32_t *s_raw)
{
    int32_t angle;

    while (theta >= CORDIC_PI)  { theta -= CORDIC_TWO_PI; }
    while (theta < -CORDIC_PI)  { theta += CORDIC_TWO_PI; }

    angle = (int32_t)(theta * CORDIC_Q31_PER_RAD);

    /* Handshake: the G4 has no NRDY bit, "no result pending" is RRDY == 0.
     * Writing WDATA while a result is still queued is silently dropped, which
     * makes the next read return the PREVIOUS conversion (a one-call lag).
     * So: drain first, then wait for each of the two results separately. */
    while ((CORDIC->CSR & CORDIC_CSR_RRDY) != 0u) {
        (void)CORDIC->RDATA;
    }

    CORDIC->WDATA = (uint32_t)angle;

    while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0u) {
        /* spin: 16 iterations */
    }
    if (c_raw != 0) { *c_raw = (int32_t)CORDIC->RDATA; }

    while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0u) {
        /* spin: second result */
    }
    if (s_raw != 0) { *s_raw = (int32_t)CORDIC->RDATA; }

    if (c_raw != 0) { *c = (float)(*c_raw) * CORDIC_Q31_TO_FLOAT; }
    if (s_raw != 0) { *s = (float)(*s_raw) * CORDIC_Q31_TO_FLOAT; }
}

void bsp_cordic_cos_sin(float theta, float *c, float *s)
{
    bsp_cordic_cos_sin_raw(theta, c, s, 0, 0);
}

uint8_t bsp_cordic_self_test(void)
{
    static const float ang[5] = { 0.0f, 1.5707963f, 3.1415927f,
                                  -1.5707963f, 0.5235988f };
    static const float rc[5]  = { 1.0f, 0.0f, -1.0f,  0.0f, 0.8660254f };
    static const float rs[5]  = { 0.0f, 1.0f,  0.0f, -1.0f, 0.5f };
    uint8_t i;

    for (i = 0u; i < 5u; i++) {
        float c;
        float s;

        bsp_cordic_cos_sin(ang[i], &c, &s);

        if ((fabsf(c - rc[i]) > 0.02f) || (fabsf(s - rs[i]) > 0.02f)) {
            return (uint8_t)(i + 1u);
        }
    }
    return 0u;
}
