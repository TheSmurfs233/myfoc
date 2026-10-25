/**
  ******************************************************************************
  * @file    bsp_pwm.h
  * @brief   TIM1 three-phase complementary PWM output.
  *
  * S2 scope: open-loop, fixed duty. No SVPWM (S4), no current loop (S6).
  * The module owns every write to TIM1->CCER / TIM1->BDTR / CCR1..4, so the
  * App layer never touches timer registers.
  *
  * Safety contract:
  *   - after bsp_pwm_init() the bridge is OFF (MOE=0, CCxE=0) and the preloaded
  *     duty is PWM_START_DUTY (50% = zero vector, no phase-to-phase voltage);
  *   - bsp_pwm_disable() clears MOE first, which takes effect immediately;
  *   - every duty is clamped to PWM_CCR_MIN..PWM_CCR_MAX, so 0% / 100% (which
  *     would kill the low-side sampling window) can never be programmed.
  ******************************************************************************
  */

#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Configure the bridge, start the counter, keep all outputs disabled. */
void    bsp_pwm_init(void);

/* Enable / disable the six outputs. enable() preloads the requested duty and
 * latches it before setting MOE, so the first pulse is already correct. */
void    bsp_pwm_enable(void);
void    bsp_pwm_disable(void);
uint8_t bsp_pwm_is_enabled(void);

/* duty is the high-side on-time fraction, 0..1, clamped internally. Writes are
 * preloaded and applied at the next update event (all three phases together). */
void    bsp_pwm_set_duty(float a, float b, float c);
void    bsp_pwm_set_all_duty(float d);
void    bsp_pwm_get_duty(float *a, float *b, float *c);

/* Pure conversion helpers (no hardware access, unit-tested on the host). */
uint16_t bsp_pwm_duty_to_ccr(float d);
float    bsp_pwm_ccr_to_duty(uint16_t ccr);

/* Dead time actually programmed in BDTR.DTG. Compared against PWM_DTG_REG at
 * boot so a CubeMX regeneration that silently restores the old value is seen. */
uint16_t bsp_pwm_get_dtg(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H */
