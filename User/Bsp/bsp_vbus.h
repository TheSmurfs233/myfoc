/**
  ******************************************************************************
  * @file    bsp_vbus.h
  * @brief   DC bus voltage measurement (ADC1_IN2 / PA1, divider 12k/2k).
  *
  * Layer: Bsp.
  *
  * CubeMX configures ADC1 as a 12-bit, single, software-triggered regular
  * conversion on channel 2 with a 47.5 cycle sampling time; PA1 is analog.
  * This driver adds the calibration CubeMX does not generate, then reads the
  * channel once per millisecond with a short register-level poll (the
  * conversion takes about 1.4 us at PCLK/4).
  ******************************************************************************
  */

#ifndef __BSP_VBUS_H
#define __BSP_VBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void bsp_vbus_init(void);

/* Start one conversion and read it. Returns 1 when a new value was taken. */
uint8_t bsp_vbus_update(void);

float    bsp_vbus_volts(void);   /* filtered, V                          */
float    bsp_vbus_raw_volts(void); /* unfiltered, V                      */
uint16_t bsp_vbus_raw(void);     /* last raw ADC code                    */
uint8_t  bsp_vbus_valid(void);   /* 1 when the reading is in range       */
uint32_t bsp_vbus_errs(void);    /* out of range / timeout counter       */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_VBUS_H */
