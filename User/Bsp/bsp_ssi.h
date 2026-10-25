/**
  ******************************************************************************
  * @file    bsp_ssi.h
  * @brief   MT6701 SSI (SPI mode) frame reader.
  *
  * Layer: Bsp.
  *
  * SPI1 is configured by CubeMX as master, 8-bit, CPOL=0 / CPHA=1 (mode 1),
  * MSB first, 10.625 MHz, NSS software. CSN is a plain GPIO (PA15).
  * One frame is exactly 24 clocks: D[13:0], Mg[3:0], CRC[5:0].
  ******************************************************************************
  */

#ifndef __BSP_SSI_H
#define __BSP_SSI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void bsp_ssi_init(void);

/* Clock out one 24-bit frame into frame[3] (MSB first).
 * Returns 0 on success, non-zero if the transfer failed. */
uint8_t bsp_ssi_read(uint8_t frame[3]);

/* Diagnostic: SPI1 CFG1 and RCC_CCIPR2, used by the "enc bench" command.
 * The measured frame time is 4x the theoretical 24 bits / 10.625 MHz, so
 * the real baud-rate divider has to be checked. */
void bsp_ssi_diag(uint32_t *cr1, uint32_t *ccipr2);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SSI_H */
