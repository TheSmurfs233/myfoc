/**
  ******************************************************************************
  * @file    bsp_sys.c
  * @brief   ART accelerator enable (see bsp_sys.h).
  *
  * Layer: Bsp.
  *
  * At 170MHz the embedded flash needs 4 wait states, and FLASH_ACR resets with
  * prefetch and both caches off. CubeMX never emits those bits, so without this
  * the core stalls on almost every instruction fetch: measured 2.7 cycles per
  * instruction in the 20kHz control chain (see the "phase" command).
  *
  * Only the instruction side is enabled here. The data cache (DCEN) is left off
  * on purpose: bsp_flash.c erases and programs the flash at runtime (calibration
  * record) and the data cache would have to be cleaned around those operations.
  ******************************************************************************
  */

#include "bsp_sys.h"
#include "stm32g4xx_hal.h"

void bsp_sys_init(void)
{
    FLASH->ACR |= (FLASH_ACR_PRFTEN | FLASH_ACR_ICEN);
}
