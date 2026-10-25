/**
  ******************************************************************************
  * @file    bsp_flash_fake.c
  * @brief   Host replacement for User/Bsp/bsp_flash.c.
  *
  * The host build cannot map 0x0801F800, so this module provides the same API
  * backed by one RAM page (fake_flash_page). Production code stays free of
  * test-only branches; the switch is made here, at link time.
  ******************************************************************************
  */

#include "bsp_flash.h"
#include "board_cfg.h"
#include "fake_api.h"
#include <string.h>

extern uint8_t fake_flash_page_enc[0x800u];

#define FAKE_FLASH_PAGE_SIZE   0x800u

void bsp_flash_read(uint32_t addr, void *dst, uint32_t len)
{
    fake_flash_read(addr, dst, len);
}

uint8_t bsp_flash_erase_page(uint32_t addr)
{
    if ((addr >= ENC_FLASH_ADDR) && (addr < (ENC_FLASH_ADDR + FAKE_FLASH_PAGE_SIZE))) {
        memset(fake_flash_page_enc, 0xFF, FAKE_FLASH_PAGE_SIZE);
    } else {
        memset(fake_flash_page, 0xFF, FAKE_FLASH_PAGE_SIZE);
    }
    return 0u;
}

uint8_t bsp_flash_program(uint32_t addr, const void *src, uint32_t len)
{
    uint32_t off;

    if ((addr >= ENC_FLASH_ADDR) && (addr < (ENC_FLASH_ADDR + FAKE_FLASH_PAGE_SIZE))) {
        off = addr - ENC_FLASH_ADDR;
        if ((off + len) > FAKE_FLASH_PAGE_SIZE) {
            return 1u;
        }
        memcpy(&fake_flash_page_enc[off], src, len);
        return 0u;
    }

    off = addr - CAL_FLASH_ADDR;
    if ((off + len) > FAKE_FLASH_PAGE_SIZE) {
        return 1u;
    }
    memcpy(&fake_flash_page[off], src, len);
    return 0u;
}
