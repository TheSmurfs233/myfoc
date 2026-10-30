/**
  ******************************************************************************
  * @file    mid_vofa.h
  * @brief   VOFA+ waveform upload - FireWater (CSV) on the single debug UART.
  *
  * Official FireWater frame layout:
  *
  *     "<any>:ch0,ch1,ch2,...,chN\n"      optional line label
  *     "ch0,ch1,ch2,...,chN\n"            plain CSV (used here)
  *
  * Examples from the VOFA+ help page:
  *
  *     printf("samples: 1.1, 3.2, -0.6, -0.9\n");
  *     printf("1.1, 3.2, -0.6, -0.9\n");
  *
  * HARD RULES
  *   1. A colon may appear ONLY once, as the optional prefix at the very start
  *      of the line. Writing "t:1.0,st:2.0" is WRONG - the parser breaks at the
  *      second colon, the line is not recognised as data, and VOFA+ then shows
  *      it as raw text (which floods the receive pane).
  *   2. Every frame must end with '\n'. VOFA+ only commits a row on newline.
  *   3. Channel values are comma separated, no spaces are required.
  *
  * Because the waveform rows are valid CSV, VOFA+ consumes them into the plot
  * and never echoes them to the text pane. Human readable log lines carry the
  * "[log] " marker (see mid_log.c) so they can never be mistaken for data.
  ******************************************************************************
  */

#ifndef __MID_VOFA_H
#define __MID_VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define VOFA_MAX_CHANNELS   24u

/**
  * @brief  Send one FireWater CSV row (non-blocking, goes into the TX ring).
  * @param  ch   channel array
  * @param  num  channel count (1 .. VOFA_MAX_CHANNELS)
  * @retval 1 = whole row queued, 0 = not enough TX space, row dropped
  */
uint8_t vofa_send(const float *ch, uint8_t num);

/* Rows dropped because the TX ring did not have room for a whole row */
uint32_t vofa_drop_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __MID_VOFA_H */
