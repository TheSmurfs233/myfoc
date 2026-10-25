/**
  ******************************************************************************
  * @file    tim.h   (HOST TEST STUB)
  * @brief   Minimal TIM1 register model, just enough for bsp_pwm.c.
  *
  *          Shadows Core/Inc/tim.h when building the host tests (-I test/host/fake
  *          comes first). Never compile this into the firmware image.
  ******************************************************************************
  */

#ifndef __FAKE_TIM_H
#define __FAKE_TIM_H

#include <stdint.h>

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t SR;
    volatile uint32_t CCER;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t EGR;
} TIM_TypeDef;

extern TIM_TypeDef fake_tim1_regs;
#define TIM1 (&fake_tim1_regs)

/* Only the bits bsp_pwm.c touches (same positions as the real device) */
#define TIM_CR1_CEN     (1u << 0)
#define TIM_SR_UIF      (1u << 0)
#define TIM_EGR_UG      (1u << 0)
#define TIM_CCER_CC1E   (1u << 0)
#define TIM_CCER_CC1NE  (1u << 2)
#define TIM_CCER_CC2E   (1u << 4)
#define TIM_CCER_CC2NE  (1u << 6)
#define TIM_CCER_CC3E   (1u << 8)
#define TIM_CCER_CC3NE  (1u << 10)
#define TIM_CCER_CC4E   (1u << 12)
#define TIM_BDTR_MOE    (1u << 15)
#define TIM_BDTR_DTG    (0xFFu << 0)

#endif /* __FAKE_TIM_H */
