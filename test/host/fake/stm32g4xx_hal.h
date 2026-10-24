/**
  ******************************************************************************
  * @file    stm32g4xx_hal.h   (HOST TEST STUB)
  * @brief   Minimal fake HAL used to compile the pure-logic modules of the
  *          firmware on a Windows/Linux host and run unit tests.
  *
  *          Only the symbols actually referenced by the User folder are
  *          provided. Never compile this into the firmware image.
  ******************************************************************************
  */

#ifndef __FAKE_STM32G4XX_HAL_H
#define __FAKE_STM32G4XX_HAL_H

#include <stdint.h>
#include <stddef.h>

/* ---- status / enums ---- */
typedef enum {
    HAL_OK = 0x00,
    HAL_ERROR = 0x01,
    HAL_BUSY = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

/* ---- GPIO ---- */
typedef struct { int id; } GPIO_TypeDef;

typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
    uint32_t Alternate;
} GPIO_InitTypeDef;

#define GPIO_PIN_0          (1u << 0)
#define GPIO_PIN_1          (1u << 1)
#define GPIO_MODE_INPUT     0u
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_PULLUP         1u
#define GPIO_NOPULL         0u
#define GPIO_SPEED_FREQ_LOW 0u

/* ---- DMA ---- */
typedef struct {
    volatile uint32_t CCR;
    volatile uint32_t CNDTR;
} DMA_Channel_TypeDef;

typedef struct {
    uint32_t Mode;
    uint32_t Request;
} DMA_InitTypeDef;

typedef struct DMA_HandleTypeDef {
    DMA_Channel_TypeDef *Instance;
    DMA_InitTypeDef      Init;
    void                *Parent;
} DMA_HandleTypeDef;

#define DMA_CIRCULAR    (0x1u << 5)
#define DMA_CCR_CIRC    (0x1u << 5)
#define SET_BIT(REG, BIT) ((REG) |= (BIT))

/* ---- USART ---- */
typedef struct { int id; } USART_TypeDef;

typedef struct {
    USART_TypeDef *Instance;
    uint32_t       RxEventType;   /* HAL_UART_RXEVENT_TC / HT / IDLE */
} UART_HandleTypeDef;

#define HAL_UART_RXEVENT_TC     (0x00000000u)
#define HAL_UART_RXEVENT_HT     (0x00000001u)
#define HAL_UART_RXEVENT_IDLE   (0x00000002u)

/* ---- function stubs ---- */
uint32_t          HAL_GetTick(void);
void              HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void              HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState state);
GPIO_PinState     HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint32_t pin);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
                                               uint8_t *pData, uint16_t size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart,
                                        uint8_t *pData, uint16_t size);

#define __HAL_UART_CLEAR_OREFLAG(h) ((void)(h))
#define __HAL_UART_CLEAR_NEFLAG(h)  ((void)(h))
#define __HAL_UART_CLEAR_FEFLAG(h)  ((void)(h))
#define __HAL_UART_CLEAR_PEFLAG(h)  ((void)(h))

#endif /* __FAKE_STM32G4XX_HAL_H */
