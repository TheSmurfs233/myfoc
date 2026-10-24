/**
  ******************************************************************************
  * @file    usart.h   (HOST TEST STUB)
  * @brief   Shadows Core/Inc/usart.h during host tests.
  ******************************************************************************
  */

#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart1;
extern USART_TypeDef      fake_usart1;

#define USART1 (&fake_usart1)

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
