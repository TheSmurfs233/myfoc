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

/* ---- ADC ---- */
typedef struct {
    volatile uint32_t JDR1;
    volatile uint32_t JDR2;
    volatile uint32_t JDR3;
    volatile uint32_t ISR;
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t JSQR;
    volatile uint32_t DR;      /* regular data register (S7.5 Vbus) */
} ADC_TypeDef;

#define ADC_ISR_EOC     (1u << 2)
#define ADC_CR_ADSTART  (1u << 2)
#ifndef ADC1
extern ADC_TypeDef fake_adc1_regs;
#define ADC1 (&fake_adc1_regs)
#endif

typedef struct {
    ADC_TypeDef *Instance;
} ADC_HandleTypeDef;

extern ADC_TypeDef       fake_adc2_regs;
extern ADC_HandleTypeDef hadc2;
#define ADC2            (&fake_adc2_regs)
#define ADC_ISR_OVR     (1u << 4)
#define ADC_SINGLE_ENDED 0u

/* ---- DWT cycle counter (used by bsp_prof) ---- */
typedef struct { volatile uint32_t DEMCR; } CoreDebug_TypeDef;
typedef struct { volatile uint32_t CTRL; volatile uint32_t CYCCNT; } DWT_TypeDef;

extern CoreDebug_TypeDef fake_coredebug;
extern DWT_TypeDef       fake_dwt;
#define CoreDebug (&fake_coredebug)
#define DWT       (&fake_dwt)

#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24)
#define DWT_CTRL_CYCCNTENA_Msk     (1u << 0)

/* ---- function stubs ---- */
/* ---------------- FLASH ---------------- */
/* The host build links test/host/fake/bsp_flash_fake.c instead of the real
 * bsp_flash.c, so no HAL flash stubs are needed here. */
typedef struct {
    volatile uint32_t ACR;
} FLASH_TypeDef;

extern FLASH_TypeDef fake_flash_regs;
#define FLASH (&fake_flash_regs)

#define FLASH_ACR_PRFTEN    (1u << 8)
#define FLASH_ACR_ICEN      (1u << 9)
#define FLASH_ACR_DCEN      (1u << 10)

uint32_t          HAL_GetTick(void);
void              HAL_Delay(uint32_t Delay);
void              HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void              HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState state);
GPIO_PinState     HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint32_t pin);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
                                               uint8_t *pData, uint16_t size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart,
                                        uint8_t *pData, uint16_t size);
/* Used by the TX watchdog in bsp_uart_tx_pump() when a completion is lost */
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);

HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t SingleDiff);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *hadc);
/* Implemented by bsp_adc.c; called directly by the tests to simulate a JEOC */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);

#define __HAL_UART_CLEAR_OREFLAG(h) ((void)(h))
#define __HAL_UART_CLEAR_NEFLAG(h)  ((void)(h))
#define __HAL_UART_CLEAR_FEFLAG(h)  ((void)(h))
#define __HAL_UART_CLEAR_PEFLAG(h)  ((void)(h))

#endif /* __FAKE_STM32G4XX_HAL_H */
