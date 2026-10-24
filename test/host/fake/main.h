/**
  ******************************************************************************
  * @file    main.h   (HOST TEST STUB)
  * @brief   Shadows Core/Inc/main.h during host tests.
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/* Fake GPIO ports. KEY is on port A, LED on port F (matches the real board). */
extern GPIO_TypeDef fake_gpio_a;
extern GPIO_TypeDef fake_gpio_f;

#define LED_Pin         GPIO_PIN_1
#define LED_GPIO_Port   (&fake_gpio_f)
#define KEY_Pin         GPIO_PIN_0
#define KEY_GPIO_Port   (&fake_gpio_a)

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
