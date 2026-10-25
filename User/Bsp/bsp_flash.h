/**
  ******************************************************************************
  * @file    bsp_flash.h
  * @brief   Minimal internal-flash helpers: read / erase one page / program.
  *
  * Layer: Bsp. Only the raw flash mechanics; what to store is the caller's
  * business (see mid_current.c for the calibration record).
  *
  * The page containing the target address is erased as a whole (STM32G4 pages
  * are 2KB). Programming is done in 64-bit double words because the G4 flash
  * only accepts that width.
  ******************************************************************************
  */

#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include <stdint.h>

/* Plain copy from flash (it is memory mapped on the target, but the host test
 * build needs an indirection, so everything goes through here). */
void    bsp_flash_read(uint32_t addr, void *dst, uint32_t len);

/* Erase the 2KB page that contains addr. Returns 0 on success. */
uint8_t bsp_flash_erase_page(uint32_t addr);

/* Program len bytes at addr; the page must have been erased first.
 * Returns 0 on success. */
uint8_t bsp_flash_program(uint32_t addr, const void *src, uint32_t len);

#endif /* __BSP_FLASH_H */
