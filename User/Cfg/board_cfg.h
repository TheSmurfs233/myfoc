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
#define FW_BUILD_ID             64u

/* ---- zero-drift calibration record in flash ---------------------------- */
/* The last 2KB page of the 128KB part is reserved for it; the Keil target ROM
 * region is shrunk to 0x1F800 so the linker never places code there.
 * magic 'CAL1', version bumps if the layout changes (v2 added the generation). */
#define CAL_FLASH_ADDR          0x0801F800u
#define CAL_FLASH_MAGIC         0x314C4143u     /* "CAL1" little endian */
#define CAL_FLASH_VERSION       2u

/* ==========================================================================
 * 1. System clock / control period
 * ========================================================================== */
#define SYS_CORE_CLK_HZ         170000000u
#define FOC_TS                  0.00005f        /* 20kHz current loop = 50us */
#define FOC_FREQ_HZ             20000u
/* Integer version for display: 0.00005f * 1e6 evaluates to 49.99999 and would
 * truncate to 49, so always use this macro when printing Ts. */
#define FOC_TS_US               (1000000u / FOC_FREQ_HZ)
/* DWT cycles per microsecond; used by the "phase" probe to print delays
 * without a 64-bit division (170 cycles = 1us at 170MHz). */
#define PHASE_CYC_PER_US        (SYS_CORE_CLK_HZ / 1000000u)
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
/* CH1..CH3 run in PWM mode 2: the high pulse is centred on the counter peak
 * and the low sides conduct around the valley. CCR is therefore the *inactive*
 * (low-side) duration, so CCR_MIN gives the MAXIMUM duty and CCR_MAX the
 * minimum. Keeping 40..4209 never reaches 0% / 100% and always leaves a
 * low-side conduction window for the current sampling at the valley (S3). */
#define PWM_CCR_MIN             40u                     /* duty  99.06% */
#define PWM_CCR_MAX             (PWM_ARR - 40u)         /* duty   0.94% */

/* Duty applied when the bridge is first enabled. 50% on all three phases is
 * the zero vector: no phase-to-phase voltage, so it is safe with or without a
 * motor connected. */
#define PWM_START_DUTY          0.5f

/* CH4 / PA11 (ADC trigger for S3) is configured ENTIRELY by CubeMX: the
 * "Pulse" value of TIM1 CH4 sets CCR4. With PWM2 the pin is high while
 * CNT >= CCR4, so:
 *   - CCR4 = ARR   -> rising edge exactly on the counter peak, ~6ns pulse
 *                     (correct, but invisible on a scope);
 *   - CCR4 = 4000  -> ~2.9us pulse, edge ~1.5us before the peak
 *                     (current CubeMX value: visible, still near the centre).
 * The firmware deliberately does NOT write CCR4, so the CubeMX setting stands. */

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

/* Zero-drift calibration: average this many injected frames at 20kHz (12.8ms).
 * Runs at "start" with the duty forced to the 50% zero vector, because the
 * TIM1_CC4 trigger only reaches the ADC while the bridge is enabled (MOE=1). */
#define CUR_CAL_SAMPLES         256u

/* Saturation flag threshold: raw code within this many LSB of a rail. */
#define CUR_SAT_MARGIN          8u

/* Sign convention: positive = current flowing OUT of the bridge into the motor.
 * Which way the INA240 actually reads depends on how IN+/IN- are wired across
 * the shunt. Measured on the bench 2026-10-25 (motor, 0.55/0.45/0.45):
 * A phase high -> current leaves the A bridge, but raw dropped below the offset,
 * so the reading is inverted; -1.0f makes "out of the bridge" positive. */
#define CUR_SIGN                (-1.0f)

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
 * 5. Open-loop V/f (S4)
 *    Motor: 7 pole pairs, 12V, 330KV  ->  no-load 3960rpm = 462Hz electrical
 *    V/f curve: V_LL_rms(f) = VF_BOOST_V + VF_K_V_PER_HZ * f
 *    The modulation index is derived from the line-to-line RMS voltage:
 *      V_LL_rms = (sqrt3/2)/sqrt2 * m * Vdc = 0.6124 * m * Vdc
 *      => m = 1.633 * V_LL_rms / Vdc
 * ========================================================================== */
#define VF_BOOST_V              1.5f            /* low-speed IR boost, V_LL_rms */
#define VF_K_V_PER_HZ           0.026f          /* 12V / 462Hz */
#define VF_M_PER_V              (1.632993f / VBUS_NOMINAL_V)
#define VF_M_MAX                0.95f           /* keep 5% headroom before clamping */
#define VF_F_MAX                200.0f          /* first release: 1714 rpm */
#define VF_ACC_HZ_PER_S         20.0f           /* 0 -> 200Hz in 10s */
#define VF_ACC_MIN_HZ_PER_S     1.0f
#define VF_ACC_MAX_HZ_PER_S     200.0f
#define VF_TS_S                 (1.0f / 20000.0f)

/* Guard bands: the ADC range is only +-1.32A and the motor would draw 3.5A at
 * 12V stall, so the open loop must throttle itself before the sensor clips. */
#define VF_I_FOLD_A             1.00f           /* start folding the modulation */
#define VF_I_FOLD_BACK_A        0.80f           /* recover below this */
#define VF_I_TRIP_A             1.25f           /* hard trip -> FAULT */
#define VF_I_FOLD_FRAMES        20u             /* consecutive frames */
#define VF_I_TRIP_FRAMES        5u
#define VF_M_SCALE_MIN          0.30f

/* ==========================================================================
 * 6. Trigonometry back end (S4 bench experiment)
 *    TRIG_MODE_LUT    : 256-entry table + linear interpolation (default)
 *    TRIG_MODE_CORDIC : hardware CORDIC co-processor
 *    Switching is a one-line rebuild; the LUT stays the default so the
 *    verified behaviour does not change unless explicitly asked for.
 * ========================================================================== */
#define TRIG_MODE_LUT           0
#define TRIG_MODE_CORDIC        1

#ifndef TRIG_MODE
#define TRIG_MODE               TRIG_MODE_LUT
#endif

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

/* TX watchdog. One 64-byte chunk at 1Mbaud takes ~0.64 ms, so a completion
 * event that has not arrived 100 ms later is lost (UART error, DMA glitch).
 * Without this, s_tx_busy would stay set forever and the debug link would go
 * permanently silent while the motor keeps running. See bsp_uart_tx_pump(). */
#define DBG_TX_STUCK_MS         100u

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
 * 7. MT6701 magnetic encoder (SSI over SPI1)
 *    PA15 = CSN, PB3 = SCK, PB4 = MISO, PB5 = MOSI, SPI mode 1, 10.625 MHz
 *    Frame: D[13:0] angle, Mg[3:0] status, CRC[5:0] (x^6+x+1)
 * ========================================================================== */
#define ENC_POLE_PAIRS          7               /* motor pole pairs */
#define ENC_DIR_DEFAULT         0u              /* 0 = CCW positive */

/* Zero offset + direction live in their own page so the current-calibration
 * record (0x0801F800) is never touched by "enc zero". */
#define ENC_FLASH_ADDR          0x0801F000u
#define ENC_FLASH_MAGIC         0x31434E45u     /* ENC1 little endian */
#define ENC_FLASH_VERSION       1u


/* ---- ALIGN: forced alignment for the electrical zero (S5) ------------- */
/* A fixed voltage vector at theta = 0 pulls the rotor d-axis onto the phase A
 * axis. m = 0.20 gives ~1.5V line-to-line, i.e. ~0.33A through 4.6 ohm. */
/* Alignment strategy: bidirectional sweep averaging.
 *
 * A static d-axis vector (the classic method) does not work on this motor:
 * the magnetic torque falls as sin(dtheta) near the equilibrium, and the
 * bearing stiction plus cogging stop the rotor a few degrees short (measured:
 * 0.5A moved it 0.83 deg, 1.0A only 0.27 deg). Spinning the field drags the
 * rotor along, but when the field stops the rotor keeps its lag angle, so the
 * final position varied by +/-9 deg mechanical between runs.
 *
 * So instead of trying to "catch" the rotor, MEASURE the offset:
 *   - sweep the field forward N revs and back N revs at ALIGN_SWEEP_HZ
 *   - every frame accumulate err = encoder_elec - field_elec
 *   - the friction lag has opposite signs in the two directions and cancels,
 *     the encoder zero offset stays, the cogging ripple averages out
 *   - finally shift the stored zero by err_mean / pole_pairs
 * This does not need the torque to beat the stiction, only the rotor to
 * roughly follow the field, which it does. */
#define ALIGN_M_START           0.15f
#define ALIGN_M_MAX             0.45f   /* ~1.17A in phase A (ADC range 1.32A) */
#define ALIGN_RAMP_MS           200u    /* spin up the magnitude first */
#define ALIGN_SWEEP_HZ          2.0f    /* electrical Hz during the sweeps */
/* 7 electrical revs = exactly ONE mechanical rev (7 pole pairs): that makes
 * the encoder eccentricity (period = 1 mech rev) average to zero, which a
 * non-integer number of revs cannot do. */
#define ALIGN_SWEEP_MS          7000u   /* 2 Hz * 7 s = 14 elec revs = 2 mech revs */
#define ALIGN_SETTLE_MS         1000u   /* let the rotor catch up before averaging */
#define ALIGN_I_MAX_A           1.28f   /* abort above this (ADC range 1.32 A) */
#define ALIGN_TIMEOUT_MS        18000u  /* two sweeps plus margin */

/* Read the encoder every N control frames (1 = 20kHz). */
#define ENC_DIV                 1u

/* Poll period while the bridge is off (ms). The ADC only runs in RUN, so a
 * stopped motor needs its own timer: 1ms = 1kHz, plenty for "enc" and
 * "enc zero". */
#define ENC_POLL_MS             1u

/* Speed is measured over this window: the angle delta accumulated over
 * ENC_SPD_WINDOW_MS divided by the elapsed time. A time window instead of
 * a per-sample delta keeps the 14-bit quantisation from dominating at low
 * speed (0.022 deg/LSB). */
#define ENC_SPD_WINDOW_MS       10u


/* ==========================================================================
 * 8. Motor parameters (placeholders until measured)
 * ========================================================================== */
#define MOTOR_POLE_PAIRS        7u              /* [TBD] pole pairs */
#define MOTOR_RS_OHM            0.5f            /* [TBD] phase resistance */
#define MOTOR_LS_H              0.001f          /* [TBD] phase inductance */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_CFG_H */
