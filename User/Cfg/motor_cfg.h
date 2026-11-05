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
/* 7 pole pairs, 12 V, KV 330, phase R 2.3 ohm, L ~0.5 mH (see docs/33). */
#define MOTOR_POLE_PAIRS        7           /* pole pairs                  */
#define MOTOR_R_MOHM            2300        /* phase resistance, mohm      */
#define MOTOR_L_UH              500         /* phase inductance, uH        */
#define MOTOR_KE_UV_RADS        4130        /* back-EMF, uV per elec rad/s */
#define MOTOR_J_NKGM2           90000       /* inertia, n kg*m^2 (9e-5)    */

/* The board's shunt chain saturates at +-1.32 A, so the usable current is
 * capped there no matter what the motor could take. */
#define MOTOR_I_RATED_MA        1000        /* nameplate continuous rating, mA;
                                             * documentation only - the actual
                                             * clamp is MOTOR_I_MAX_MA below */
#define MOTOR_I_MAX_MA          1200        /* current-loop clamp, mA      */
#define MOTOR_I_TRIP_MA         1280        /* over-current trip, mA       */

/* Command limit, kept at 2000 so the existing behaviour is unchanged. The
 * practical ceiling is lower: 12 V, 0.95 modulation -> 5.7 V available, which
 * with the KE above tops out near 1800 rpm (2000 rpm would need 6.06 V). */
#define MOTOR_RPM               2000

/* Control seeds: starting points that follow from R/L/J; re-tune with the
 * step/sstep tools after a motor swap. */
#define MOTOR_FOC_KP            3.00f       /* V/A                         */
#define MOTOR_FOC_KI            11000.0f    /* V/(A*s)                     */
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
 * the pole-pair count: 0.00413 * 7 = 0.0289 N*m/A.
 *
 * CAREFUL: using KE_elec alone makes the acceleration feedforward 7x too
 * large. That mistake was made once during the S10.0 refactor and the build
 * stayed at 0 errors / 0 warnings, hence the assertions at the bottom. */
#define MOTOR_KT_UNM            (MOTOR_KE_UV_RADS * MOTOR_POLE_PAIRS)     /* 28910 uN*m/A      */
#define MOTOR_FF_ACC_U          ((MOTOR_J_NKGM2 * 1000L) / MOTOR_KT_UNM)  /* 3113 uA/(rad/s^2) */

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
MOTOR_CFG_ASSERT((MOTOR_KT_UNM > 20000) && (MOTOR_KT_UNM < 45000), kt);
MOTOR_CFG_ASSERT((MOTOR_FF_ACC_U > 2000) && (MOTOR_FF_ACC_U < 5000), ff_acc);

#endif /* __MOTOR_CFG_H */
