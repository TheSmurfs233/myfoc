/**
  ******************************************************************************
  * @file    mid_vofa.c
  * @brief   VOFA+ FireWater framing: plain CSV, 3 decimals, newline terminated.
  *
  * Output looks like:
  *     169.347,1.000,0.000,2.000,-773.000,452.542\n
  *
  * No per-channel names (that would need a colon per value and break the
  * parser). Channel names are configured once in the VOFA+ UI as ch0..ch5.
  ******************************************************************************
  */

#include "mid_vofa.h"
#include "bsp_uart.h"
#include <stddef.h>     /* NULL */

static uint32_t s_drop_cnt;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
static uint16_t u32_to_str(char *dst, uint32_t v)
{
    char    tmp[11];
    uint8_t i = 0u;
    uint8_t j;

    if (v == 0u) {
        tmp[i++] = '0';
    } else {
        while (v != 0u) { tmp[i++] = (char)('0' + (v % 10u)); v /= 10u; }
    }
    for (j = 0u; j < i; j++) { dst[j] = tmp[(uint8_t)(i - 1u - j)]; }
    return (uint16_t)i;
}

/* float with 3 decimals, e.g. -12.345 */
static uint16_t f_to_str3(char *dst, float v)
{
    uint16_t n = 0u;
    int32_t  ip;
    uint32_t frac;
    uint8_t  neg = 0u;

    if (v != v) { v = 0.0f; }                 /* NaN guard: would break the plot */
    if (v >  999999.0f) { v =  999999.0f; }
    if (v < -999999.0f) { v = -999999.0f; }

    if (v < 0.0f) { neg = 1u; v = -v; }

    ip   = (int32_t)v;
    frac = (uint32_t)((v - (float)ip) * 1000.0f + 0.5f);
    if (frac >= 1000u) { frac = 999u; }

    if (neg) { dst[n++] = '-'; }
    n = (uint16_t)(n + u32_to_str(&dst[n], (uint32_t)ip));
    dst[n++] = '.';
    if (frac < 100u) { dst[n++] = '0'; }
    if (frac < 10u)  { dst[n++] = '0'; }
    n = (uint16_t)(n + u32_to_str(&dst[n], frac));
    return n;
}

/* ------------------------------------------------------------------ */
/* Send                                                                */
/* ------------------------------------------------------------------ */
uint8_t vofa_send(const float *ch, uint8_t num)
{
    char     buf[VOFA_MAX_CHANNELS * 16u + 2u];
    uint16_t n = 0u;
    uint8_t  i;

    if ((ch == NULL) || (num == 0u) || (num > VOFA_MAX_CHANNELS)) {
        return 0u;
    }

    for (i = 0u; i < num; i++) {
        if (i != 0u) { buf[n++] = ','; }
        n = (uint16_t)(n + f_to_str3(&buf[n], ch[i]));
    }
    buf[n++] = '\n';

    /* All or nothing: a half row would corrupt the CSV stream. */
    if (bsp_uart_tx_space() < n) {
        s_drop_cnt++;
        return 0u;
    }
    (void)bsp_uart_tx_write((const uint8_t *)buf, n);
    return 1u;
}

uint32_t vofa_drop_count(void)
{
    return s_drop_cnt;
}
