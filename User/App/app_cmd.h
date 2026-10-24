/**
  ******************************************************************************
  * @file    app_cmd.h
  * @brief   Host text command parser (ASCII line protocol).
  *
  * Protocol: PC -> MCU sends one ASCII line terminated by \r or \n, tokens
  *           separated by spaces. Examples:
  *               stat\r\n      stop\r\n      mode 2\r\n      rate 500\r\n
  *           MCU -> PC replies with human readable text.
  *
  *           Text replies and JustFloat binary frames can share one port:
  *           JustFloat is delimited by its 00 00 80 7F tail, text lines by
  *           CRLF, so neither confuses the other.
  ******************************************************************************
  */

#ifndef __APP_CMD_H
#define __APP_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void app_cmd_init(void);
/* Main loop: pull bytes from the RX ring, assemble a line, execute it */
void app_cmd_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CMD_H */
