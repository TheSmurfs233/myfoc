/**
  ******************************************************************************
  * @file    mid_log.h
  * @brief   Lightweight formatted output (no stdio / printf / heap).
  *
  *   - Self-contained formatter: avoids pulling in MicroLIB and the floating
  *     point printf machinery (huge code size, deep stack usage).
  *   - Output goes through the bsp_uart TX ring buffer, so it is non-blocking.
  *   - Supports %d %i %u %x %X %s %c %f %%, optional '0' padding and width,
  *     e.g. %08x %4d %-8s. Precision (%.3f) is accepted but ignored: %f always
  *     prints 2 decimals. Use a scaled integer if more precision is needed.
  *   - MAIN LOOP ONLY. Never call from an ISR (it uses a static buffer).
  ******************************************************************************
  */

#ifndef __MID_LOG_H
#define __MID_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void log_printf(const char *fmt, ...);
/* Output a NUL-terminated string as-is (no CRLF appended) */
void log_str(const char *s);
/* Output CRLF */
void log_crlf(void);

#ifdef __cplusplus
}
#endif

#endif /* __MID_LOG_H */
