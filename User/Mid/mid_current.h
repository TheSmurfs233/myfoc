/**
  ******************************************************************************
  * @file    mid_current.h
  * @brief   Phase current conversion and zero-drift calibration.
  *
  * Layer: Mid. No HAL access: it only turns raw ADC codes into amps.
  *
  * Calibration: the ADC trigger (TIM1_CC4) only reaches the ADC while the
  * bridge is enabled (MOE=1), so calibration runs at "start" with the duty
  * forced to the 50% zero vector (no phase-to-phase voltage -> no current).
  * The first CUR_CAL_SAMPLES frames (256 -> 12.8ms) are averaged per phase and
  * stored as the zero-drift offset. Until that completes, ready==0, the
  * reported currents are 0 and "pwm" commands are refused.
  ******************************************************************************
  */

#ifndef __MID_CURRENT_H
#define __MID_CURRENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float    ia, ib, ic;     /* amps, offset removed, sign applied */
    uint16_t raw[3];         /* newest ADC codes (debug) */
    uint16_t offset[3];      /* zero-current codes from calibration */
    uint8_t  ready;          /* 1 = offsets are valid */
    uint8_t  saturated;      /* 1 = at least one channel is at a rail */
    uint8_t  src;            /* where the offsets came from, see CUR_SRC_* */
    uint32_t gen;            /* calibration generation (increments per save) */
    uint32_t frames;         /* frames processed */
} current_sample_t;

/* Offset provenance, reported by "cur" / "stat" as src=... */
#define CUR_SRC_NONE   0u    /* nominal 2048, never calibrated */
#define CUR_SRC_FLASH  1u    /* restored from the flash record at boot */
#define CUR_SRC_LIVE   2u    /* just calibrated in this session */

void mid_current_init(void);        /* offsets to nominal, needs calibration */
void mid_current_restart_cal(void); /* re-arm the averaging window */
void mid_current_update(const uint16_t raw[3]);   /* one frame */
const current_sample_t *mid_current_get(void);

/* Calibration persistence: the record lives in the last 2KB flash page and a
 * successful calibration is written back automatically.
 *   mid_current_load()      -> 1 if a valid record was found (ready becomes 1)
 *   mid_current_erase_cal() -> wipe the record, back to "needs calibration" */
uint8_t mid_current_load(void);
void    mid_current_erase_cal(void);

#ifdef __cplusplus
}
#endif

#endif /* __MID_CURRENT_H */
