/**
  ******************************************************************************
  * @file    fake_api.h
  * @brief   Test-only hooks exposed by the HAL stub.
  ******************************************************************************
  */

#ifndef __FAKE_API_H
#define __FAKE_API_H

#include <stdint.h>

/* ---- time ---- */
void     fake_tick_reset(void);
void     fake_tick_set(uint32_t ms);
void     fake_tick_add(uint32_t ms);
uint32_t fake_tick_get(void);

/* ---- KEY input (0 = released, 1 = pressed) ---- */
void fake_key_set(int pressed);

/* ---- TX capture ---- */
void            fake_tx_reset(void);
const uint8_t  *fake_tx_data(void);
uint32_t        fake_tx_len(void);
/* Return HAL_BUSY from the next Transmit_DMA call to exercise the retry path */
void            fake_tx_force_busy(int on);

/* ---- RX injection (mimics the DMA writing into the reception buffer) ---- */
void fake_rx_feed(const uint8_t *data, uint16_t len);
void fake_rx_feed_str(const char *s);

#endif /* __FAKE_API_H */
