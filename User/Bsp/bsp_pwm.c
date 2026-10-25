/**
  ******************************************************************************
  * @file    bsp_pwm.c
  * @brief   TIM1 three-phase complementary PWM output (S2, open loop).
  *
  * Register-level implementation on purpose:
  *   HAL_TIM_PWM_Start() sets MOE automatically at the end of the call, which
  *   would enable the bridge before the requested duty is latched. Here the
  *   order is always: preload CCR -> latch (UG) -> CCxE -> MOE.
  *
  * TIM1 configuration (tim.c):
  *   center aligned 1, ARR=4249 -> 20.000 kHz, DTG=4, complementary CH1..CH3
  *   in PWM mode 2 (high pulse centred on the counter peak).
  *   CH4 is PWM2 and is configured entirely by CubeMX (tim.c): its CCR4 sets
   *   both the width of the PA11 debug pulse and how far before the counter
   *   peak the ADC trigger fires. The firmware never overwrites it - it is
   *   user configuration. The shunts are in series with the phase lines, so
   *   the phase current is continuous and may be sampled at any time.
  *   CH4 is also brought out on PA11 for scope debugging.
  *
  * No log output here: Bsp must not depend on Mid (see docs/10).
  ******************************************************************************
  */

#include "bsp_pwm.h"
#include "board_cfg.h"
#include "tim.h"
#include <stddef.h>     /* NULL */

/* Six main outputs: three high sides and three low sides. CH4 is handled
 * separately: it must stay enabled even while the bridge is off, because the
 * ADC injected group is triggered by the CH4 compare event and the zero-drift
 * calibration at boot needs ADC conversions with MOE=0. */
#define PWM_CCER_MAIN  (TIM_CCER_CC1E  | TIM_CCER_CC1NE | \
                        TIM_CCER_CC2E  | TIM_CCER_CC2NE | \
                        TIM_CCER_CC3E  | TIM_CCER_CC3NE)

static float s_duty[3] = { PWM_START_DUTY, PWM_START_DUTY, PWM_START_DUTY };

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
static float clamp01(float d)
{
    if (d != d)      { return 0.0f; }   /* NaN guard */
    if (d < 0.0f)    { return 0.0f; }
    if (d > 1.0f)    { return 1.0f; }
    return d;
}

uint16_t bsp_pwm_duty_to_ccr(float d)
{
    uint32_t ccr;

    /* CH1..CH3 run in PWM mode 2: the high side is active while CNT >= CCR,
     * i.e. the high pulse is centred on the counter peak. High time is
     * (period - CCR), so this mapping is inverted with respect to PWM1:
     * duty 1.0 -> CCR 0, duty 0.0 -> CCR = full period. */
    d   = clamp01(d);
    ccr = (uint32_t)((1.0f - d) * (float)PWM_CCR_PERIOD + 0.5f);

    if (ccr < (uint32_t)PWM_CCR_MIN) { ccr = PWM_CCR_MIN; }
    if (ccr > (uint32_t)PWM_CCR_MAX) { ccr = PWM_CCR_MAX; }
    return (uint16_t)ccr;
}

float bsp_pwm_ccr_to_duty(uint16_t ccr)
{
    return 1.0f - ((float)ccr / (float)PWM_CCR_PERIOD);
}

uint16_t bsp_pwm_get_dtg(void)
{
    return (uint16_t)(TIM1->BDTR & TIM_BDTR_DTG);
}

/* Position inside the current PWM period, in timer counts:
 * 0 .. PWM_ARR, one count = 1/(20kHz*4250) = 11.76ns, full scale = 50us.
 * Used by the "phase" command to see where the control loop lands. */
uint16_t bsp_pwm_get_cnt(void)
{
    return (uint16_t)(TIM1->CNT & 0xFFFFu);
}

/* ------------------------------------------------------------------ */
/* Init: counter running, all outputs off, safe duty preloaded         */
/* ------------------------------------------------------------------ */
void bsp_pwm_init(void)
{
    uint16_t ccr;

    /* Outputs off before anything else is touched, but keep CH4 enabled:
     * its compare event is what triggers the ADC (S3), and calibration has to
     * run with MOE=0. With MOE=0 the PA11 pin is forced low by OSSI anyway. */
    TIM1->CCER &= ~PWM_CCER_MAIN;
    TIM1->CCER |= TIM_CCER_CC4E;
    TIM1->BDTR &= ~TIM_BDTR_MOE;

    /* CCR preload. HAL_TIM_PWM_ConfigChannel() always sets OCxPE, so with the
     * CubeMX boxes on "Output compare preload = Enable" (all three channels,
     * .ioc: OC1/OC2/OC3Preload_PWM=ENABLE) these bits are already set. Re-assert
     * them here so a future regeneration that flips one box off cannot silently
     * bring back mid-pulse CCR updates. CH4 is left alone: CCR4 is never written. */
    TIM1->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;

    /* Zero vector preloaded: whenever the bridge is enabled it starts at 50% */
    ccr          = bsp_pwm_duty_to_ccr(PWM_START_DUTY);
    s_duty[0]    = bsp_pwm_ccr_to_duty(ccr);
    s_duty[1]    = s_duty[0];
    s_duty[2]    = s_duty[0];
    TIM1->CCR1   = ccr;
    TIM1->CCR2   = ccr;
    TIM1->CCR3   = ccr;
    /* CCR4 is NOT written here: it is a CubeMX setting (tim.c Pulse for CH4),
     * and the firmware must not override user configuration. The value only
     * decides how wide the PA11 debug pulse is and how far before the peak the
     * ADC trigger fires. */
    TIM1->CR1 |= TIM_CR1_CEN;

    /* Latch the preloaded values now instead of waiting for the first UEV */
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR  = 0u;
}

/* ------------------------------------------------------------------ */
/* Enable / disable                                                    */
/* ------------------------------------------------------------------ */
void bsp_pwm_enable(void)
{
    /* Requested duty must be in the shadow registers before MOE goes high */
    TIM1->CCR1 = bsp_pwm_duty_to_ccr(s_duty[0]);
    TIM1->CCR2 = bsp_pwm_duty_to_ccr(s_duty[1]);
    TIM1->CCR3 = bsp_pwm_duty_to_ccr(s_duty[2]);
    TIM1->EGR  = TIM_EGR_UG;
    TIM1->SR   = 0u;

    TIM1->CCER |= (PWM_CCER_MAIN | TIM_CCER_CC4E);
    TIM1->BDTR |= TIM_BDTR_MOE;
}

void bsp_pwm_disable(void)
{
    /* MOE first: it takes effect immediately. With OSSI enabled the pins are
     * forced to their idle level (low), which is HIN=LIN=0 for the EG3033.
     * CH4 (CC4E) is left enabled so the ADC keeps triggering while stopped:
     * the zero-drift calibration must run with the bridge off. */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER &= ~PWM_CCER_MAIN;
}

uint8_t bsp_pwm_is_enabled(void)
{
    return ((TIM1->BDTR & TIM_BDTR_MOE) != 0u) ? 1u : 0u;
}

/* ------------------------------------------------------------------ */
/* Duty control                                                        */
/* ------------------------------------------------------------------ */
void bsp_pwm_set_duty(float a, float b, float c)
{
    uint16_t ca = bsp_pwm_duty_to_ccr(a);
    uint16_t cb = bsp_pwm_duty_to_ccr(b);
    uint16_t cc = bsp_pwm_duty_to_ccr(c);

    /* Preloaded: the three phases change together at the next update event */
    TIM1->CCR1 = ca;
    TIM1->CCR2 = cb;
    TIM1->CCR3 = cc;

    /* Store what is really programmed, not what was asked: PWM_CCR_MIN/MAX
     * mean 0% and 100% are not representable, so "pwm 0" reads back 0.94%. */
    s_duty[0] = bsp_pwm_ccr_to_duty(ca);
    s_duty[1] = bsp_pwm_ccr_to_duty(cb);
    s_duty[2] = bsp_pwm_ccr_to_duty(cc);
}

void bsp_pwm_set_all_duty(float d)
{
    bsp_pwm_set_duty(d, d, d);
}

void bsp_pwm_get_duty(float *a, float *b, float *c)
{
    if (a != NULL) { *a = s_duty[0]; }
    if (b != NULL) { *b = s_duty[1]; }
    if (c != NULL) { *c = s_duty[2]; }
}
