/**
  ******************************************************************************
  * @file    bsp_gpio.h
  * @brief   LED / KEY driver with debouncing and edge events.
  ******************************************************************************
  */

#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void    bsp_gpio_init(void);

/* LED: on=1 lights up (active-low handled internally) */
void    bsp_led_write(uint8_t on);
void    bsp_led_toggle(void);
uint8_t bsp_led_is_on(void);

/* KEY: must be polled periodically from the main loop (>= 1kHz).
 * 20ms debounce is done internally. */
void    bsp_key_poll(uint32_t now_ms);
/* Debounced stable level: 1 = pressed */
uint8_t bsp_key_is_down(void);
/* Take a "pressed" event (cleared on read). 1 = a press happened. */
uint8_t bsp_key_take_press(void);
/* Take a "released" event */
uint8_t bsp_key_take_release(void);
/* Total press count */
uint32_t bsp_key_press_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GPIO_H */
