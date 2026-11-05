/**
  ******************************************************************************
  * @file    motor_cfg.h
  * @brief   Everything that is specific to the motor, in one place.
  *
  * Layer: Cfg.
  *
  * Why: the S10 sensorless work targets "swap the motor, keep the firmware".
  * Every number that depends on the motor lives here; board_cfg.h only aliases
  * them, so a new motor means editing this file and re-running the
  * identification procedure in docs/34 (sident / align ol / obs health).
  *
  * Values are written as exact integers in a convenient unit and converted to
  * floats below. That is deliberate:
  *
  *   - ARMCC 5 has no _Static_assert and rejects floating-point constant
  *     expressions in array sizes, so compile-time range checks are only
  *     possible on integers.
  *   - It also removes any doubt about the intended value (2.3 ohm is 2300
  *     milliohm, not "about 2.3").
  ******************************************************************************
  */

#ifndef __MOTOR_CFG_H
#define __MOTOR_CFG_H

/* ------------------------------------------------ canonical integer values */
/* 7 pole pairs, 12 V. R/L/KE measured with "sident" on the bench
 * (2026-11-05): R 2.87/2.88/2.92 ohm in three runs, L 0.93 mH (|z| 11.98 ohm
 * at 2 kHz, phase fit 0.95 mH), KE 2.59..2.93 mV/(elec rad/s) over six drags.
 * The previous motor was 2.3 ohm / 0.5 mH / 4.13 mV.
 * J is NOT measured (sident cannot): 90000 is the old motor's value and only
 * feeds the acceleration term, so it is a seed until the ramp is observed. */
#define MOTOR_POLE_PAIRS        7           /* pole pairs                  */
#define MOTOR_R_MOHM            2890        /* phase resistance, mohm      */
#define MOTOR_L_UH              930         /* phase inductance, uH        */
#define MOTOR_KE_UV_RADS        2700        /* back-EMF, uV per elec rad/s */
#define MOTOR_J_NKGM2           90000       /* inertia, n kg*m^2 (9e-5)    */

/* The board's shunt chain saturates at +-1.32 A, so the usable current is
 * capped there no matter what the motor could take. */
#define MOTOR_I_RATED_MA        1000        /* nameplate continuous rating, mA;
                                             * documentation only - the actual
                                             * clamp is MOTOR_I_MAX_MA below */
#define MOTOR_I_MAX_MA          1200        /* current-loop clamp, mA      */
#define MOTOR_I_TRIP_MA         1280        /* over-current trip, mA       */

/* Command limit. With KE = 2.70 mV the 5.7 V budget runs out near 2880 rpm,
 * so 2000 leaves room; the old motor (4.13 mV) topped out near 1800 rpm. */
#define MOTOR_RPM               2000

/* Control seeds: starting points that follow from R/L/J; re-tune with the
 * step/sstep tools after a motor swap. */
/* The current loop was tuned for L = 0.5 mH: kp = 3.0 gave a crossover of
 * kp/(2*pi*L) = 955 Hz.  The new motor is 0.93 mH, which would halve that to
 * 513 Hz and leave the back-EMF observer (tuned against the 955 Hz loop) short
 * of phase margin - it diverged right after hand-over.  kp 5.5 puts the
 * crossover back at 941 Hz; ki is scaled by the same 1.83 so the PI zero stays
 * at ki/(2*pi*kp) = 579 Hz instead of sliding down with the plant pole. */
#define MOTOR_FOC_KP            5.50f       /* V/A                         */
#define MOTOR_FOC_KI            20000.0f    /* V/(A*s)                     */
#define MOTOR_SPD_KP            0.006f      /* A/rpm                       */
#define MOTOR_SPD_KI            0.080f      /* A/(rpm*s)                   */

/* ------------------------------------------------------- float views */
#define MOTOR_R_OHM             ((float)MOTOR_R_MOHM / 1000.0f)
#define MOTOR_L_H               ((float)MOTOR_L_UH / 1000000.0f)
#define MOTOR_KE_V_RAD_S        ((float)MOTOR_KE_UV_RADS / 1000000.0f)
#define MOTOR_J_KGM2            ((float)MOTOR_J_NKGM2 / 1000000000.0f)
#define MOTOR_I_RATED_A         ((float)MOTOR_I_RATED_MA / 1000.0f)
#define MOTOR_I_MAX_A           ((float)MOTOR_I_MAX_MA / 1000.0f)
#define MOTOR_I_TRIP_A          ((float)MOTOR_I_TRIP_MA / 1000.0f)
#define MOTOR_RPM_MAX_F         ((float)MOTOR_RPM)

/* Kt(SI) equals the back-EMF constant per MECHANICAL rad/s, i.e. KE_elec times
 * the pole-pair count: 0.00270 * 7 = 0.0189 N*m/A (was 0.0289).
 *
 * CAREFUL: using KE_elec alone makes the acceleration feedforward 7x too
 * large. That mistake was made once during the S10.0 refactor and the build
 * stayed at 0 errors / 0 warnings, hence the assertions at the bottom. */
#define MOTOR_KT_UNM            (MOTOR_KE_UV_RADS * MOTOR_POLE_PAIRS)     /* 18900 uN*m/A      */
#define MOTOR_FF_ACC_U          ((MOTOR_J_NKGM2 * 1000L) / MOTOR_KT_UNM)  /* 4761 uA/(rad/s^2) */

#define MOTOR_KT_NM_A           ((float)MOTOR_KT_UNM / 1000000.0f)
#define MOTOR_FF_ACC_A_RADS2    ((float)MOTOR_FF_ACC_U / 1000000.0f)

/* ------------------------------------------------- compile-time sanity
 * ARMCC 5 has no _Static_assert, so this is the classic negative-array trick,
 * driven by the integer values above. It catches the "looks plausible but is
 * off by a factor" class of error that the compiler otherwise accepts in
 * silence. */
#define MOTOR_CFG_ASSERT(cond, tag) \
    typedef char motor_cfg_assert_##tag[(cond) ? 1 : -1]

MOTOR_CFG_ASSERT((MOTOR_POLE_PAIRS > 0) && (MOTOR_POLE_PAIRS < 64), pp);
MOTOR_CFG_ASSERT((MOTOR_R_MOHM > 50) && (MOTOR_R_MOHM < 50000), r);
MOTOR_CFG_ASSERT((MOTOR_L_UH > 50) && (MOTOR_L_UH < 50000), l);
MOTOR_CFG_ASSERT((MOTOR_KE_UV_RADS > 1000) && (MOTOR_KE_UV_RADS < 20000), ke);
MOTOR_CFG_ASSERT((MOTOR_J_NKGM2 > 1000) && (MOTOR_J_NKGM2 < 10000000), j);
MOTOR_CFG_ASSERT((MOTOR_I_MAX_MA <= 1300), imax_adc);      /* ADC range is 1.32 A */
MOTOR_CFG_ASSERT((MOTOR_I_TRIP_MA > MOTOR_I_MAX_MA), trip_above_clamp);
MOTOR_CFG_ASSERT((MOTOR_RPM > 100) && (MOTOR_RPM < 20000), rpm);
/* 15000 lower bound: the bench motor (2.70 mV * 7 = 18900) sits below the
 * original 20000 window, and a lower-KV motor is a legitimate swap. */
MOTOR_CFG_ASSERT((MOTOR_KT_UNM > 15000) && (MOTOR_KT_UNM < 45000), kt);
MOTOR_CFG_ASSERT((MOTOR_FF_ACC_U > 1500) && (MOTOR_FF_ACC_U < 8000), ff_acc);

#endif /* __MOTOR_CFG_H */
