/**
  ******************************************************************************
  * @file    bsp_flash.c
  * @brief   Internal flash read / page erase / double-word program.
  *
  * Layer: Bsp.
  *
  * Note on timing: on a single-bank G4 the CPU stalls while the flash is being
  * erased or programmed (~20-40ms for one 2KB page). That is acceptable here
  * because it only happens when a calibration finishes, and the bridge is at
  * the 50% zero vector at that moment: the timer and the gate driver keep
  * running, only the firmware pauses.
  ******************************************************************************
  */

#include "bsp_flash.h"
#include "stm32g4xx_hal.h"
#include <string.h>

void bsp_flash_read(uint32_t addr, void *dst, uint32_t len)
{
    (void)memcpy(dst, (const void *)addr, len);
}

uint8_t bsp_flash_erase_page(uint32_t addr)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0u;
    uint8_t  rc = 0u;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase.NbPages   = 1u;

    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        rc = 1u;
    }
    HAL_FLASH_Lock();

    return rc;
}

uint8_t bsp_flash_program(uint32_t addr, const void *src, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)src;
    uint64_t dword;
    uint32_t i;
    uint8_t  rc = 0u;

    HAL_FLASH_Unlock();

    for (i = 0u; i < len; i += 8u) {
        uint32_t n = len - i;

        if (n > 8u) {
            n = 8u;
        }
        /* Erased flash reads as 0xFF: pad the tail with it so a partial last
         * double word still programs to the intended bytes. */
        dword = 0xFFFFFFFFFFFFFFFFull;
        (void)memcpy(&dword, &p[i], n);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, dword) != HAL_OK) {
            rc = 1u;
            break;
        }
    }

    HAL_FLASH_Lock();

    return rc;
}
