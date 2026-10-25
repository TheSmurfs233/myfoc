/**
  ******************************************************************************
  * @file    hal_stub.c
  * @brief   Host-side implementation of the fake HAL + test hooks.
  ******************************************************************************
  */

#include "fake_api.h"
#include "stm32g4xx_hal.h"
#include "board_cfg.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* Objects the production code expects to exist                        */
/* ------------------------------------------------------------------ */
GPIO_TypeDef        fake_gpio_a = { 1 };
GPIO_TypeDef        fake_gpio_f = { 2 };
USART_TypeDef       fake_usart1 = { 1 };
UART_HandleTypeDef  huart1      = { &fake_usart1, 0u };

/* TIM1 register model. BDTR defaults to DTG=4 (as programmed in tim.c) so the
 * boot-time dead-time check in app_init() stays quiet. */
TIM_TypeDef         fake_tim1_regs = { 0u, 0u, 0u, 0u, 0u, 0u, 0u, 4u, 0u };

/* ADC2 injected-group model + DWT cycle counter model */
ADC_TypeDef         fake_adc2_regs = { 2048u, 2048u, 2048u, 0u, 0u, 0u, 0u };
ADC_HandleTypeDef   hadc2          = { &fake_adc2_regs };
DWT_TypeDef         fake_dwt       = { 0u, 0u };
CoreDebug_TypeDef   fake_coredebug = { 0u };

HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t SingleDiff)
{
    (void)hadc; (void)SingleDiff;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    return HAL_OK;
}

DMA_Channel_TypeDef fake_dma_ch1;
DMA_Channel_TypeDef fake_dma_ch2;
DMA_HandleTypeDef   hdma_usart1_rx = { &fake_dma_ch1, { 0u, 0u }, 0 };
DMA_HandleTypeDef   hdma_usart1_tx = { &fake_dma_ch2, { 0u, 0u }, 0 };

/* Callbacks implemented by the firmware (bsp_uart.c) */
extern void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
extern void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/* ------------------------------------------------------------------ */
/* Time                                                                */
/* ------------------------------------------------------------------ */
static uint32_t s_tick;

uint32_t HAL_GetTick(void)          { return s_tick; }
void     HAL_Delay(uint32_t ms)     { s_tick += ms; }   /* never blocks on the host */

/* ---------------- FLASH: fake ACR + one 2KB page at CAL_FLASH_ADDR -------- */
FLASH_TypeDef fake_flash_regs;
uint8_t fake_flash_page[0x800u];
static int s_flash_ready;

static void fake_flash_init(void)
{
    if (!s_flash_ready) {
        memset(fake_flash_page, 0xFF, sizeof(fake_flash_page));
        s_flash_ready = 1;
    }
}

void fake_flash_reset(void)
{
    memset(fake_flash_page, 0xFF, sizeof(fake_flash_page));
    s_flash_ready = 1;
}

void fake_flash_read(uint32_t addr, void *dst, uint32_t len)
{
    uint32_t off = addr - CAL_FLASH_ADDR;
    fake_flash_init();
    if (off + len > sizeof(fake_flash_page)) { memset(dst, 0xFF, len); return; }
    memcpy(dst, &fake_flash_page[off], len);
}
void     fake_tick_reset(void)      { s_tick = 0u; }
void     fake_tick_set(uint32_t ms) { s_tick = ms; }
void     fake_tick_add(uint32_t ms) { s_tick += ms; }
uint32_t fake_tick_get(void)        { return s_tick; }

/* ------------------------------------------------------------------ */
/* GPIO                                                                */
/* ------------------------------------------------------------------ */
static int s_key_pressed;
static int s_led_level;      /* 0 = PF1 low (LED on), 1 = PF1 high */

void fake_key_set(int pressed) { s_key_pressed = pressed; }
int  fake_led_level(void)      { return s_led_level; }

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port; (void)init;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState state)
{
    if ((port == &fake_gpio_f) && (pin == LED_Pin)) {
        s_led_level = (state == GPIO_PIN_SET) ? 1 : 0;
    }
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint32_t pin)
{
    if ((port == &fake_gpio_a) && (pin == KEY_Pin)) {
        /* active low: pressed -> RESET */
        return s_key_pressed ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    return GPIO_PIN_SET;
}

/* ------------------------------------------------------------------ */
/* DMA                                                                 */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma)
{
    if (hdma == NULL) { return HAL_ERROR; }
    hdma->Instance->CCR = hdma->Init.Mode;
    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/* UART TX: capture and complete synchronously                         */
/* ------------------------------------------------------------------ */
#define FAKE_TX_MAX (256u * 1024u)

static uint8_t  s_tx_buf[FAKE_TX_MAX];
static uint32_t s_tx_len;
static int      s_tx_force_busy;
static int      s_tx_call_count;
static int      s_tx_no_complete;
static int      s_tx_abort_count;

void fake_tx_reset(void)
{
    s_tx_len = 0u;
    s_tx_force_busy = 0;
    s_tx_call_count = 0;
    s_tx_no_complete = 0;
    s_tx_abort_count = 0;
}
const uint8_t *fake_tx_data(void) { return s_tx_buf; }
uint32_t fake_tx_len(void)        { return s_tx_len; }
int      fake_tx_call_count(void) { return s_tx_call_count; }
void     fake_tx_force_busy(int on) { s_tx_force_busy = on; }
void     fake_tx_no_complete(int on) { s_tx_no_complete = on; }
int      fake_tx_abort_count(void) { return s_tx_abort_count; }

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart,
                                        uint8_t *pData, uint16_t size)
{
    (void)huart;
    s_tx_call_count++;

    if (s_tx_force_busy) {
        s_tx_force_busy = 0;     /* only the next call fails */
        return HAL_BUSY;
    }
    if ((uint32_t)size > (FAKE_TX_MAX - s_tx_len)) {
        return HAL_ERROR;
    }
    memcpy(&s_tx_buf[s_tx_len], pData, size);
    s_tx_len += size;

    /* Simulate the DMA transfer completing immediately, unless the test wants
     * to model a completion that never arrives */
    if (!s_tx_no_complete) {
        HAL_UART_TxCpltCallback(&huart1);
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart)
{
    (void)huart;
    s_tx_abort_count++;
    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/* UART RX: the test injects bytes into the reception buffer           */
/* ------------------------------------------------------------------ */
static uint8_t *s_rx_buf;
static uint16_t s_rx_size;
static uint16_t s_rx_pos;
static int      s_rx_started;

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
                                               uint8_t *pData, uint16_t size)
{
    (void)huart;
    s_rx_buf     = pData;
    s_rx_size    = size;
    s_rx_pos     = 0u;
    s_rx_started = 1;
    return HAL_OK;
}

uint16_t fake_rx_pos(void)     { return s_rx_pos; }
int      fake_rx_started(void) { return s_rx_started; }

void fake_rx_feed(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (!s_rx_started || (s_rx_buf == NULL) || (s_rx_size == 0u)) { return; }

    for (i = 0u; i < len; i++) {
        s_rx_buf[s_rx_pos] = data[i];
        s_rx_pos++;
        if (s_rx_pos >= s_rx_size) {
            s_rx_pos = 0u;
            /* Circular DMA raises a Transfer-Complete event (Size = full
             * buffer) every time it wraps, exactly like real hardware. */
            huart1.RxEventType = HAL_UART_RXEVENT_TC;
            HAL_UARTEx_RxEventCallback(&huart1, s_rx_size);
        }
    }
    /* Line idle: report the current write position */
    huart1.RxEventType = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart1, s_rx_pos);
}

void fake_rx_feed_str(const char *s)
{
    fake_rx_feed((const uint8_t *)s, (uint32_t)0u + (uint16_t)strlen(s));
}
