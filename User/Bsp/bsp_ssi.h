/**
  ******************************************************************************
  * @file    bsp_ssi.h
  * @brief   MT6701 SSI (SPI mode) frame reader, DMA based.
  *
  * Layer: Bsp.
  *
  * SPI1 is configured by CubeMX as master, 8-bit, CPOL=0 / CPHA=1 (mode 1),
  * MSB first, 10.625 MHz, NSS software. CSN is a plain GPIO (PA15).
  * One frame is exactly 24 clocks: D[13:0], Mg[3:0], CRC[5:0].
  *
  * The RX side runs on DMA1 channel 3 (DMAMUX request 10). The CPU only writes
  * the three dummy bytes that generate the clocks (~0.3us); the 2.26us of
  * shifting happens in the background, so the control loop can do useful work
  * while the frame is clocked in.
  ******************************************************************************
  */

#ifndef __BSP_SSI_H
#define __BSP_SSI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void bsp_ssi_init(void);

/* Arm one 24-bit transfer: CSN goes low, the RX DMA is armed and three dummy
 * bytes are queued. Returns immediately (~0.3us). A second call while a
 * transfer is still in flight is ignored. */
void bsp_ssi_start(void);

/* Non-blocking: returns 1 and copies the frame once the transfer finished,
 * 0 while it is still in flight. */
/* 0 = not ready, 1 = frame ready, 2 = ready but the SPI lost a byte (OVR) */
uint8_t bsp_ssi_poll(uint8_t frame[3]);

/* Cumulative SPI RX overruns (one byte overwritten before the DMA read it). */
uint32_t bsp_ssi_ovr_count(void);

/* 1 while a transfer is armed but not yet collected. */
uint8_t bsp_ssi_busy(void);

/* Drop any transfer in flight and release CSN (recovery path). */
void bsp_ssi_abort(void);

/* Blocking convenience wrapper (start + spin until done).
 * Returns 0 on success, non-zero if the transfer failed. */
uint8_t bsp_ssi_read(uint8_t frame[3]);

/* Diagnostic: SPI1 CR1 and RCC_CCIPR2, used by the "enc bench" command. */
void bsp_ssi_diag(uint32_t *cr1, uint32_t *ccipr2);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SSI_H */
