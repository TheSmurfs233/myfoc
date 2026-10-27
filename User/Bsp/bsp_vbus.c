/**
  ******************************************************************************
  * @file    bsp_vbus.c
  * @brief   DC bus voltage measurement (see bsp_vbus.h).
  *
  * Layer: Bsp.
  *
  * The divider (R20 12k / R21 2k) maps 0..23.1 V onto 0..3.3 V, so one LSB is
  * 5.64 mV of bus voltage. A first order filter with about a 5 ms time constant
  * removes the switching ripple without hiding a real supply sag.
  ******************************************************************************
  */

#include "bsp_vbus.h"
#include "board_cfg.h"
#include "stm32g4xx_hal.h"

extern ADC_HandleTypeDef hadc1;

#define VBUS_FILTER_K     0.2f      /* 1 kHz update -> ~5 ms time constant */
#define VBUS_SPIN_LIMIT   2000u

static float    s_volts;      /* filtered                          */
static float    s_raw_volts;  /* last unfiltered sample            */
static uint16_t s_raw;
static uint8_t  s_valid;
static uint32_t s_errs;

void bsp_vbus_init(void)
{
    /* Same pattern as ADC2 in bsp_adc.c: CubeMX only initialises the
     * peripheral, the calibration is application code. Without it the
     * reading is off by tens of millivolts. */
    (void)HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    /* HAL_ADC_Init() only configures the ADC; ADEN stays 0 until a start.
     * Without this the register-level ADSTART below is ignored and every
     * conversion times out (measured: code 0, one error per millisecond). */
    (void)HAL_ADC_Start(&hadc1);

    s_volts     = VBUS_NOMINAL_V;
    s_raw_volts = VBUS_NOMINAL_V;
    s_raw       = 0u;
    s_valid     = 0u;
    s_errs      = 0u;
}

uint8_t bsp_vbus_update(void)
{
    uint32_t spin = 0u;
    uint16_t raw;

    /* Software-triggered single conversion; ~1.4 us at PCLK/4. */
    ADC1->CR |= ADC_CR_ADSTART;

    while (((ADC1->ISR & ADC_ISR_EOC) == 0u) && (spin < VBUS_SPIN_LIMIT)) {
        spin++;
    }
    if (spin >= VBUS_SPIN_LIMIT) {
        s_errs++;
        s_valid = 0u;
        return 0u;
    }

    raw  = (uint16_t)(ADC1->DR & 0x0FFFu);   /* DR read also clears EOC */
    s_raw = raw;
    s_raw_volts = (float)raw * VBUS_V_PER_LSB;

    /* Plausibility: the supply is 9..24 V in this application. Outside that a
     * broken divider or a floating pin would otherwise poison the limiter. */
    if ((s_raw_volts > 5.0f) && (s_raw_volts < VBUS_MAX_MEASURABLE_V)) {
        s_valid = 1u;
        s_volts += VBUS_FILTER_K * (s_raw_volts - s_volts);
    } else {
        s_errs++;
        s_valid = 0u;
        s_volts = VBUS_NOMINAL_V;            /* fall back to the nominal */
    }

    return 1u;
}

float bsp_vbus_volts(void)
{
    return s_volts;
}

float bsp_vbus_raw_volts(void)
{
    return s_raw_volts;
}

uint16_t bsp_vbus_raw(void)
{
    return s_raw;
}

uint8_t bsp_vbus_valid(void)
{
    return s_valid;
}

uint32_t bsp_vbus_errs(void)
{
    return s_errs;
}
