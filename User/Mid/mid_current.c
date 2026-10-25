/**
  ******************************************************************************
  * @file    mid_current.c
  * @brief   Phase current conversion and zero-drift calibration.
  ******************************************************************************
  */

#include "mid_current.h"
#include "bsp_flash.h"
#include "board_cfg.h"
#include <string.h>

/* Calibration record kept in the last flash page. The CRC protects against a
 * half-written page (power loss during the erase/program) and against the
 * "erased page reads as 0xFF" case: 0xFFFFFFFF never matches the magic. */
typedef struct {
    uint32_t magic;       /* CAL_FLASH_MAGIC */
    uint32_t version;     /* CAL_FLASH_VERSION */
    uint32_t gen;         /* generation: +1 on every successful save */
    uint16_t offset[3];   /* zero-drift codes */
    uint16_t crc;         /* CRC16-CCITT over the bytes above */
} cal_record_t;

static current_sample_t s_cs;
static uint32_t         s_sum[3];
static uint16_t         s_n;

/* ------------------------------------------------------------------ */
/* CRC16-CCITT (0x1021, init 0xFFFF)                                   */
/* ------------------------------------------------------------------ */
static uint16_t crc16(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint16_t crc = 0xFFFFu;
    uint32_t i;
    uint8_t  b;

    for (i = 0u; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)p[i] << 8);
        for (b = 0u; b < 8u; b++) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/* Flash persistence                                                   */
/* ------------------------------------------------------------------ */
static void cal_save(void)
{
    cal_record_t rec;

    rec.magic      = CAL_FLASH_MAGIC;
    rec.version    = CAL_FLASH_VERSION;
    rec.gen        = s_cs.gen + 1u;      /* monotonic: proves a new save */
    rec.offset[0]  = s_cs.offset[0];
    rec.offset[1]  = s_cs.offset[1];
    rec.offset[2]  = s_cs.offset[2];
    rec.crc        = crc16(&rec, (uint32_t)(sizeof(rec) - sizeof(rec.crc)));

    if (bsp_flash_erase_page(CAL_FLASH_ADDR) == 0u) {
        if (bsp_flash_program(CAL_FLASH_ADDR, &rec, (uint32_t)sizeof(rec)) == 0u) {
            s_cs.gen = rec.gen;
        }
    }
}

uint8_t mid_current_load(void)
{
    cal_record_t rec;

    bsp_flash_read(CAL_FLASH_ADDR, &rec, (uint32_t)sizeof(rec));

    if (rec.magic != CAL_FLASH_MAGIC) {
        return 0u;
    }
    if (rec.version != CAL_FLASH_VERSION) {
        return 0u;
    }
    if (rec.crc != crc16(&rec, (uint32_t)(sizeof(rec) - sizeof(rec.crc)))) {
        return 0u;
    }
    if ((rec.offset[0] >= (uint16_t)ADC_FULL_SCALE) ||
        (rec.offset[1] >= (uint16_t)ADC_FULL_SCALE) ||
        (rec.offset[2] >= (uint16_t)ADC_FULL_SCALE)) {
        return 0u;
    }

    s_cs.offset[0] = rec.offset[0];
    s_cs.offset[1] = rec.offset[1];
    s_cs.offset[2] = rec.offset[2];
    s_cs.gen       = rec.gen;
    s_cs.ready     = 1u;
    s_cs.src       = CUR_SRC_FLASH;
    return 1u;
}

void mid_current_erase_cal(void)
{
    (void)bsp_flash_erase_page(CAL_FLASH_ADDR);
    mid_current_restart_cal();
    s_cs.gen = 0u;          /* no record left: the next save starts at 1 */
}

/* ------------------------------------------------------------------ */
/* Init / calibration                                                  */
/* ------------------------------------------------------------------ */
void mid_current_restart_cal(void)
{
    uint8_t i;

    for (i = 0u; i < 3u; i++) {
        s_sum[i]      = 0u;
        s_cs.offset[i] = (uint16_t)CUR_ZERO_CODE;   /* nominal until averaged */
    }
    s_n          = 0u;
    s_cs.ready   = 0u;
    s_cs.src     = CUR_SRC_NONE;
    s_cs.ia      = 0.0f;
    s_cs.ib      = 0.0f;
    s_cs.ic      = 0.0f;
    s_cs.saturated = 0u;
}

void mid_current_init(void)
{
    memset(&s_cs, 0, sizeof(s_cs));
    s_cs.frames = 0u;
    mid_current_restart_cal();
}

/* ------------------------------------------------------------------ */
/* One frame                                                           */
/* ------------------------------------------------------------------ */
void mid_current_update(const uint16_t raw[3])
{
    uint8_t i;

    s_cs.raw[0] = raw[0];
    s_cs.raw[1] = raw[1];
    s_cs.raw[2] = raw[2];
    s_cs.frames++;

    /* ---- calibration phase ---- */
    if (!s_cs.ready) {
        for (i = 0u; i < 3u; i++) { s_sum[i] += (uint32_t)raw[i]; }
        s_n++;

        if (s_n >= (uint16_t)CUR_CAL_SAMPLES) {
            for (i = 0u; i < 3u; i++) {
                /* rounded mean */
                s_cs.offset[i] = (uint16_t)((s_sum[i] + ((uint32_t)CUR_CAL_SAMPLES / 2u))
                                            / (uint32_t)CUR_CAL_SAMPLES);
            }
            s_cs.ready = 1u;
            s_cs.src   = CUR_SRC_LIVE;
            /* Persist it: the next power-up loads these instead of calibrating.
             * Blocks for ~20-40ms (flash page erase), bridge is at 50% here. */
            cal_save();
        }

        /* While calibrating the bridge is at the 50% zero vector: 0 A */
        s_cs.ia = 0.0f;
        s_cs.ib = 0.0f;
        s_cs.ic = 0.0f;
        return;
    }

    /* ---- normal conversion ---- */
    s_cs.ia = ((float)raw[0] - (float)s_cs.offset[0]) * CUR_A_PER_LSB * CUR_SIGN;
    s_cs.ib = ((float)raw[1] - (float)s_cs.offset[1]) * CUR_A_PER_LSB * CUR_SIGN;
    s_cs.ic = ((float)raw[2] - (float)s_cs.offset[2]) * CUR_A_PER_LSB * CUR_SIGN;

    s_cs.saturated = 0u;
    if ((raw[0] <= CUR_SAT_MARGIN) || (raw[0] >= (uint16_t)(ADC_FULL_SCALE - CUR_SAT_MARGIN)) ||
        (raw[1] <= CUR_SAT_MARGIN) || (raw[1] >= (uint16_t)(ADC_FULL_SCALE - CUR_SAT_MARGIN)) ||
        (raw[2] <= CUR_SAT_MARGIN) || (raw[2] >= (uint16_t)(ADC_FULL_SCALE - CUR_SAT_MARGIN))) {
        s_cs.saturated = 1u;
    }
}

const current_sample_t *mid_current_get(void)
{
    return &s_cs;
}
