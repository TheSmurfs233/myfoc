/**
  ******************************************************************************
  * @file    bsp_adc.h
  * @brief   ADC2 injected group: three phase currents, triggered by TIM1_CC4.
  *
  * Hardware path (S3):
  *   TIM1_CC4 rising edge (CH4, PWM2, CCR4 = 4000 -> ~1.5us before the peak)
  *     -> ADC2 injected group, rank1 PA4 / rank2 PA5 / rank3 PA6
  *     -> 3 x 0.871us = 2.61us
  *     -> JEOC interrupt -> this module latches JDR1..3
  *
  * The ISR does the bare minimum (three register reads + counters + profile
  * marks). Conversion to amps happens in Mid (mid_current) from the main loop.
  ******************************************************************************
  */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Calibrate the ADC and arm the injected group (interrupt mode).
 * NOTE: the TIM1_CC4 trigger is gated by MOE, so no conversion happens while
 * the bridge is off. Calibration therefore runs at "start" (50% zero vector). */
void     bsp_adc_init(void);

uint8_t  bsp_adc_started(void);
/* Init result: bit0 = calibration OK, bit1 = injected group armed OK.
 * Printed by "cur" / "stat" so a failed start is visible instead of silent. */
uint8_t  bsp_adc_status(void);
uint32_t bsp_adc_frames(void);       /* completed injected sequences */
uint32_t bsp_adc_ovr_count(void);    /* overrun events seen in the ISR */

/* Copy the newest raw codes (JDR1..3). Safe to call from the main loop. */
void     bsp_adc_get_raw(uint16_t raw[3]);

/* Register snapshot for bench debugging: cfg[0]=CR, cfg[1]=CFGR, cfg[2]=JSQR.
 * ADEN/JADSTART live in CR, the injected trigger selection (JEXTSEL/JEXTEN)
 * in JSQR - enough to tell "not armed" from "armed but never triggered". */
void     bsp_adc_get_cfg(uint32_t cfg[3]);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H */
