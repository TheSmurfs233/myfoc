/**
  ******************************************************************************
  * @file    bsp_prof.h
  * @brief   Cycle-accurate timing profiler built on the DWT cycle counter.
  *
  * Enable it once after the clock is configured, then bracket any code:
  *
  *     uint32_t t0 = bsp_prof_now();
  *     ... work ...
  *     bsp_prof_add(PROF_XXX, t0);          // accumulates now - t0
  *
  * No printing happens here: Bsp must not depend on Mid. The App reads the
  * statistics and prints them ("prof" command).
  *
  * Overhead: one DWT->CYCCNT read + a few instructions per measurement, so
  * profiling a 20kHz ISR costs well under 1% CPU. Bracketing every main-loop
  * task is visible in the loop-rate figure; turn the profiler off with
  * bsp_prof_enable(0) when measuring raw throughput.
  ******************************************************************************
  */

#ifndef __BSP_PROF_H
#define __BSP_PROF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    PROF_ISR = 0,        /* JEOC ISR: entry to exit                        */
    PROF_ISR_PERIOD,     /* time between two ISR entries (expect 50.000us)  */
    PROF_ADC_READ,       /* three JDR reads inside the ISR                  */
    PROF_CONV,           /* mid_current_update() in the main loop           */
    PROF_WAVE,           /* task_wave(): format + push one CSV row          */
    PROF_CMD,            /* app_cmd_poll(): drain RX ring + parse commands  */
    PROF_TXPUMP,         /* bsp_uart_tx_pump(): start one DMA chunk         */
    PROF_LOOP,           /* whole app_loop() iteration                      */
    PROF_ENC,            /* mid_enc_update(): SSI read + decode + speed     */
    PROF_SLOT_COUNT
} prof_slot_t;

void     bsp_prof_init(void);                     /* enable DWT CYCCNT */
void     bsp_prof_reset(void);                    /* clear all statistics */
void     bsp_prof_enable(uint8_t on);
uint8_t  bsp_prof_enabled(void);

uint32_t bsp_prof_now(void);                      /* raw cycle counter */

/* Accumulate (now - t0) into the slot. No-op when disabled. */
void     bsp_prof_add(prof_slot_t slot, uint32_t t0);

/* Same, with an explicit cycle count: keeps the aggregation logic testable
 * on the host, where there is no DWT. */
void     bsp_prof_add_cycles(prof_slot_t slot, uint32_t cycles);

uint32_t bsp_prof_count(prof_slot_t slot);
uint32_t bsp_prof_min_ns(prof_slot_t slot);
uint32_t bsp_prof_avg_ns(prof_slot_t slot);
uint32_t bsp_prof_max_ns(prof_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PROF_H */
