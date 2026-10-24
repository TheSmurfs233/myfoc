/**
  ******************************************************************************
  * @file    board_cfg.h
  * @brief   Board-level hardware constants (Board_v2 / STM32G431KBT6).
  *
  *          Every number that is tied to this specific board lives here.
  *          No magic numbers are allowed in the rest of the code.
  *
  *          NOTE: this file must stay pure ASCII. Keil ARMCC V5 parses source
  *          as GBK; UTF-8 multibyte characters inside string literals break
  *          the tokenizer. Chinese documentation lives in /docs instead.
  ******************************************************************************
  */

#ifndef __BOARD_CFG_H
#define __BOARD_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==========================================================================
 * 0. Board identity
 * ========================================================================== */
#define BOARD_NAME              "Board_v2/STM32G431KBT6"

/* Firmware build id - bump by 1 on EVERY firmware change.
 *
 * Why this exists: Keil ARMCC V5 builds incrementally. __DATE__/__TIME__ are
 * frozen into whichever .c file happens to contain them, so a build that only
 * recompiles some files keeps the OLD timestamp in the image. That makes
 * "does the board run the newest hex?" impossible to answer from the console.
 * This id is the authoritative answer.
 *
 * Bonus: editing this header touches every file that includes board_cfg.h, so
 * bumping it forces a full rebuild and refreshes __DATE__/__TIME__ as well.
 *
 * The acceptance script (test/accept/s1_accept.py) greps this value out of the
 * source and compares it with what the board reports. */
#define FW_BUILD_ID             13u

/* ==========================================================================
 * 1. System clock / control period
 * ========================================================================== */
#define SYS_CORE_CLK_HZ         170000000u
#define FOC_TS                  0.00005f        /* 20kHz current loop = 50us */
#define FOC_FREQ_HZ             20000u
/* Integer version for display: 0.00005f * 1e6 evaluates to 49.99999 and would
 * truncate to 49, so always use this macro when printing Ts. */
#define FOC_TS_US               (1000000u / FOC_FREQ_HZ)

/* ==========================================================================
 * 2. TIM1 / PWM generation
 *    f_pwm = 170MHz / (2 * (ARR+1)) = 20.000kHz
 * ========================================================================== */
#define PWM_ARR                 4249u
#define PWM_CCR_PERIOD          (PWM_ARR + 1u)          /* 4250 duty steps */
#define PWM_FREQ_HZ_CALC        (SYS_CORE_CLK_HZ / (2u * PWM_CCR_PERIOD))

/* Dead time: DTG=4 -> 4 * 5.882ns = 23.5ns (TIM1 side, nominal value).
 *
 * IMPORTANT: EG3033 has a built-in 60ns dead time with cross-lock logic, so
 * the effective dead time is max(23.5ns, 60ns) = 60ns. Any DTG <= 10 gives
 * the same 60ns result. AO4606 also has ~96ns of "natural" dead time
 * (P-MOS turns off in ~80ns, N-MOS needs ~157ns to reach Vgs(th)), so shoot
 * through is impossible even with DTG = 0.
 * Minimum dead time keeps the phase current waveform clean. */
#define PWM_DTG_REG             4u
/* Duty clamp: keep a low-side conduction window for current sampling and
 * never reach 0% / 100% (which would kill the low-side shunt signal). */
#define PWM_CCR_MIN             40u                     /* 0.94% ~ 470ns */
#define PWM_CCR_MAX             (PWM_ARR - 40u)

/* ==========================================================================
 * 3. Three-phase current sensing (3 x INA240A2 + 25mOhm low-side shunt)
 *    Vout = 50 * Vsense + VREF,  VREF = (REF1+REF2)/2 = (0+3.3)/2 = 1.65V
 * ========================================================================== */
#define RSHUNT_OHM              0.025f          /* 25mOhm fitted (schematic says 10mOhm) */
#define INA240_GAIN             50.0f           /* INA240A2 = 50 V/V */
#define ADC_VREF                3.3f
#define ADC_FULL_SCALE          4096.0f

/* Amps per LSB: 0.806mV / (50 * 0.025) = 0.645mA */
#define CUR_A_PER_LSB           ((ADC_VREF / ADC_FULL_SCALE) / (INA240_GAIN * RSHUNT_OHM))
/* Zero-current ADC code (1.65V bias) */
#define CUR_ZERO_CODE           (ADC_FULL_SCALE * 0.5f)
/* Bipolar range is only +-1.32A (1.65V / 1.25V per A) */
#define CUR_RANGE_A             ((ADC_VREF * 0.5f) / (INA240_GAIN * RSHUNT_OHM))

/* Software over-current trip (used from S7 on) */
#define CUR_TRIP_A              1.2f

/* ==========================================================================
 * 4. DC bus voltage sensing
 *    DC_BUS -> R20 12k -> PA1 (VBUS_A) -> R21 2k -> GND
 *    Divider ratio = (12k+2k)/2k = 7
 *    12V bus -> 1.714V -> code ~2128 (max measurable 23.1V)
 * ========================================================================== */
#define VBUS_DIV_R_TOP          12000.0f
#define VBUS_DIV_R_BOT          2000.0f
#define VBUS_DIV_RATIO          ((VBUS_DIV_R_TOP + VBUS_DIV_R_BOT) / VBUS_DIV_R_BOT)
#define VBUS_V_PER_LSB          ((ADC_VREF / ADC_FULL_SCALE) * VBUS_DIV_RATIO)  /* 5.64mV */

#define VBUS_NOMINAL_V          12.0f
#define VBUS_OV_V               13.8f           /* 1.15 x nominal */
#define VBUS_UV_V               9.0f            /* 0.75 x nominal */
#define VBUS_MAX_MEASURABLE_V   (ADC_VREF * VBUS_DIV_RATIO)   /* 23.1V */

/* ==========================================================================
 * 5. LED / KEY
 *    LED : 3V3 -> R37 470R -> LED anode..cathode -> PF1  => active LOW
 *    KEY : 3V3 -> R24 10k -> PA0 -> push button -> GND   => pressed = LOW
 * ========================================================================== */
#define LED_PORT                LED_GPIO_Port
#define LED_PIN_HW              LED_Pin
#define LED_ACTIVE_LOW          1

#define KEY_PORT                KEY_GPIO_Port
#define KEY_PIN_HW              KEY_Pin
#define KEY_ACTIVE_LOW          1
#define KEY_DEBOUNCE_MS         20u

/* ==========================================================================
 * 6. Debug UART
 *    USART1 (PB6=TX / PB7=RX) goes to header CN3 -> external USB-TTL.
 *    The on-board CH340N is wired to USART2 (PA2/PA3) and is not used here.
 * ========================================================================== */
#define DBG_UART_BAUD           1000000u
#define DBG_RX_BUF_SIZE         128u            /* DMA circular buffer (power of 2) */
#define DBG_RX_RING_SIZE        1024u           /* parser ring buffer (power of 2) */
#define DBG_TX_RING_SIZE        4096u           /* TX ring buffer (power of 2) */
#define DBG_TX_DMA_CHUNK        64u             /* bytes per DMA transfer */

/* Waveform output goes out on the SAME UART as the text (USART1) using the
 * FireWater text protocol, so VOFA+ can plot and you can still read the log. */
/* FireWater CSV with 6 channels is ~44 B/frame:
 *   100 Hz ->  4.4 kB/s ( 4% of 1Mbaud)
 *   500 Hz -> 22.0 kB/s (22% of 1Mbaud)   <- cap
 *  1000 Hz -> 44.0 kB/s (44% of 1Mbaud)   <- theoretical ceiling of
 *                                            HAL_GetTick (1ms), too tight */
#define DBG_WAVE_RATE_MAX       500u
/* Default = max: user asked to run at the highest rate by default.
 * Drop it with the "rate <hz>" command when a lighter stream is wanted. */
#define DBG_WAVE_RATE_HZ        DBG_WAVE_RATE_MAX


/* ==========================================================================
 * 7. Motor parameters (placeholders until measured)
 * ========================================================================== */
#define MOTOR_POLE_PAIRS        7u              /* [TBD] pole pairs */
#define MOTOR_RS_OHM            0.5f            /* [TBD] phase resistance */
#define MOTOR_LS_H              0.001f          /* [TBD] phase inductance */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_CFG_H */
