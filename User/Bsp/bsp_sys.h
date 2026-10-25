/**
  ******************************************************************************
  * @file    bsp_sys.h
  * @brief   Core-level system setup that CubeMX does not generate.
  *
  * Layer: Bsp.
  ******************************************************************************
  */

#ifndef __BSP_SYS_H
#define __BSP_SYS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Enable the ART accelerator (flash prefetch + instruction cache).
 * Must be called once, after the clock tree has been configured. */
void bsp_sys_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SYS_H */
