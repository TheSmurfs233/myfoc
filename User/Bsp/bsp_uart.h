/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   Debug UART (USART1) non-blocking TX/RX driver.
  *
  *   RX: DMA circular + IDLE detect -> ring buffer, zero CPU load
  *   TX: application writes into a 4KB ring buffer, the main loop pumps it
  *       out in 64-byte DMA chunks. Fully non-blocking.
  ******************************************************************************
  */

#ifndef __BSP_UART_H
#define __BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void     bsp_uart_init(void);

/* Queue data for transmission, returns bytes accepted (non-blocking) */
uint16_t bsp_uart_tx_write(const uint8_t *src, uint16_t len);
/* Call repeatedly from the main loop to keep the DMA moving */
void     bsp_uart_tx_pump(void);
uint16_t bsp_uart_tx_pending(void);
/* Free bytes left in the TX ring (use before writing a whole frame) */
uint16_t bsp_uart_tx_space(void);
uint8_t  bsp_uart_tx_idle(void);

/* Read received bytes from the ring buffer */
uint16_t bsp_uart_rx_read(uint8_t *dst, uint16_t max);
uint16_t bsp_uart_rx_avail(void);
uint32_t bsp_uart_rx_drop_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */
