/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @brief   ADC2 injected group driver (three phase currents).
  *
  * The injected group is armed once with HAL_ADCEx_InjectedStart_IT(); from
  * then on every TIM1_CC4 rising edge starts one sequence and JEOC fires the
  * callback below. Nothing else in the firmware touches ADC2.
  ******************************************************************************
  */

#include "bsp_adc.h"
#include "board_cfg.h"
#include "adc.h"
#include "bsp_prof.h"
#include "bsp_pwm.h"
#include "bsp_ssi.h"

static volatile uint16_t s_raw[3];
static volatile uint32_t s_frames;
static volatile uint32_t s_ovr;
static uint8_t           s_started;
static uint8_t           s_status;      /* bit0 = cal OK, bit1 = start OK */
static uint32_t          s_last_isr_cyc;
static volatile uint16_t s_isr_cnt;     /* TIM1 counter at interrupt entry */

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */
void bsp_adc_init(void)
{
    s_raw[0]      = 0u;
    s_raw[1]      = 0u;
    s_raw[2]      = 0u;
    s_frames      = 0u;
    s_ovr         = 0u;
    s_started     = 0u;
    s_status      = 0u;
    s_last_isr_cyc = 0u;
    s_isr_cnt     = 0u;

    /* Calibration must run with the ADC disabled and the bridge off. */
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) == HAL_OK) {
        s_status |= 0x01u;
    }

    if (HAL_ADCEx_InjectedStart_IT(&hadc2) == HAL_OK) {
        s_status |= 0x02u;
        s_started = 1u;
    } else {
        /* The analog regulator may still be settling right after power-up:
         * one retry is cheap and turns a silent "no ADC" into a working one. */
        HAL_Delay(1u);
        if (HAL_ADCEx_InjectedStart_IT(&hadc2) == HAL_OK) {
            s_status |= 0x02u;
            s_started = 1u;
        }
    }
}

uint8_t bsp_adc_started(void)
{
    return s_started;
}

uint8_t bsp_adc_status(void)
{
    return s_status;
}

uint32_t bsp_adc_frames(void)
{
    return s_frames;
}

uint16_t bsp_adc_isr_cnt(void)
{
    return s_isr_cnt;
}

uint32_t bsp_adc_isr_cyc(void)
{
    return s_last_isr_cyc;
}

uint32_t bsp_adc_ovr_count(void)
{
    return s_ovr;
}

void bsp_adc_get_raw(uint16_t raw[3])
{
    raw[0] = s_raw[0];
    raw[1] = s_raw[1];
    raw[2] = s_raw[2];
}

void bsp_adc_get_cfg(uint32_t cfg[3])
{
    cfg[0] = ADC2->CR;
    cfg[1] = ADC2->CFGR;
    cfg[2] = ADC2->JSQR;
}

/* ------------------------------------------------------------------ */
/* JEOC callback: hardware context, keep it short                      */
/* ------------------------------------------------------------------ */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint32_t t_isr = 0u;
    uint8_t  probe;

    if (hadc->Instance != ADC2) { return; }

    /* "prof off" also silences this probe, so the interrupt can be measured
     * without any instrumentation cost. */
    probe = bsp_prof_enabled();
    if (probe != 0u) {
        t_isr = bsp_prof_now();

        /* Where inside the 50us PWM period the interrupt actually fires. */
        s_isr_cnt = bsp_pwm_get_cnt();

        /* Period between two triggers; the first sample has no predecessor */
        if (s_last_isr_cyc != 0u) {
            bsp_prof_add_cycles(PROF_ISR_PERIOD, (uint32_t)(t_isr - s_last_isr_cyc));
        }
        s_last_isr_cyc = t_isr;
    }

    /* Start the MT6701 frame here: it takes 2.26us of shifting, which then
     * overlaps the rest of this ISR and the main loop pickup, so the angle is
     * ready before the control math needs it (S6). */
    bsp_ssi_start();

    /* Direct register reads: HAL_ADCEx_InjectedGetValue() costs hundreds of ns */
    s_raw[0] = (uint16_t)ADC2->JDR1;
    s_raw[1] = (uint16_t)ADC2->JDR2;
    s_raw[2] = (uint16_t)ADC2->JDR3;

    if (probe != 0u) {
        bsp_prof_add(PROF_ADC_READ, t_isr);
    }

    /* An overrun means a previous result was overwritten before we read it.
     * Counted (not fatal): OVR_DATA_PRESERVED keeps the newest data. */
    if ((ADC2->ISR & ADC_ISR_OVR) != 0u) {
        ADC2->ISR = ~((uint32_t)ADC_ISR_OVR);
        s_ovr++;
    }

    s_frames++;

    if (probe != 0u) {
        bsp_prof_add(PROF_ISR, t_isr);
    }
}
