/**
  ******************************************************************************
  * @file    bsp_gpio.c
  * @brief   LED / KEY driver.
  *
  * Hardware (schematic Board_v2):
  *   LED : 3V3 -> R37 470R -> LED anode..cathode -> PF1  => PF1 LOW = ON
  *   KEY : 3V3 -> R24 10k -> PA0 -> push button -> GND   => pressed = LOW
  *
  * MX_GPIO_Init() already configured PA0 as floating input and PF1 as output
  * with initial level LOW (= LED on). This module reconfigures PA0 with the
  * internal pull-up (external 10k already present, belt and braces) and turns
  * the LED off at boot so it does not look like a fault.
  ******************************************************************************
  */

#include "bsp_gpio.h"
#include "board_cfg.h"

/* ------------------------------------------------------------------ */
/* Internal state                                                      */
/* ------------------------------------------------------------------ */
static uint8_t  s_led_on       = 0u;
static uint8_t  s_key_raw      = 0u;   /* raw level: 1 = pressed */
static uint8_t  s_key_stable   = 0u;   /* debounced level */
static uint32_t s_key_t_change = 0u;   /* last raw level change time */
static uint8_t  s_key_press_evt   = 0u;
static uint8_t  s_key_release_evt = 0u;
static uint32_t s_key_press_cnt   = 0u;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
static uint8_t key_read_raw(void)
{
    GPIO_PinState st = HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN_HW);
#if (KEY_ACTIVE_LOW == 1)
    return (st == GPIO_PIN_RESET) ? 1u : 0u;
#else
    return (st == GPIO_PIN_SET) ? 1u : 0u;
#endif
}

static void led_apply(uint8_t on)
{
#if (LED_ACTIVE_LOW == 1)
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_HW, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_HW, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
    s_led_on = on ? 1u : 0u;
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */
void bsp_gpio_init(void)
{
    GPIO_InitTypeDef gi = {0};

    /* KEY: input with internal pull-up (external 10k to 3V3, button to GND) */
    gi.Pin   = KEY_PIN_HW;
    gi.Mode  = GPIO_MODE_INPUT;
    gi.Pull  = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(KEY_PORT, &gi);

    /* LED off at boot */
    led_apply(0u);

    /* Seed the debounce state from the current level to avoid a phantom event */
    s_key_raw         = key_read_raw();
    s_key_stable      = s_key_raw;
    s_key_t_change    = HAL_GetTick();
    s_key_press_evt   = 0u;
    s_key_release_evt = 0u;
    s_key_press_cnt   = 0u;
}

/* ------------------------------------------------------------------ */
/* LED                                                                 */
/* ------------------------------------------------------------------ */
void bsp_led_write(uint8_t on)
{
    led_apply(on ? 1u : 0u);
}

void bsp_led_toggle(void)
{
    led_apply(s_led_on ? 0u : 1u);
}

uint8_t bsp_led_is_on(void)
{
    return s_led_on;
}

/* ------------------------------------------------------------------ */
/* KEY: 20ms debounce, events only on stable-level transitions         */
/* ------------------------------------------------------------------ */
void bsp_key_poll(uint32_t now_ms)
{
    uint8_t raw = key_read_raw();

    if (raw != s_key_raw) {
        s_key_raw      = raw;
        s_key_t_change = now_ms;
        return;
    }

    /* Only accept the new level after it has been stable long enough */
    if ((raw != s_key_stable) && ((uint32_t)(now_ms - s_key_t_change) >= KEY_DEBOUNCE_MS)) {
        s_key_stable = raw;
        if (s_key_stable) {
            s_key_press_evt = 1u;
            s_key_press_cnt++;
        } else {
            s_key_release_evt = 1u;
        }
    }
}

uint8_t bsp_key_is_down(void)
{
    return s_key_stable;
}

uint8_t bsp_key_take_press(void)
{
    uint8_t evt = s_key_press_evt;
    s_key_press_evt = 0u;
    return evt;
}

uint8_t bsp_key_take_release(void)
{
    uint8_t evt = s_key_release_evt;
    s_key_release_evt = 0u;
    return evt;
}

uint32_t bsp_key_press_count(void)
{
    return s_key_press_cnt;
}
