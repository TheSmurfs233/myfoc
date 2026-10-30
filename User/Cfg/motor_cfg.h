/**
  ******************************************************************************
  * @file    motor_cfg.h
  * @brief   Everything that is specific to the motor, in one place.
  *
  * Layer: Cfg.
  *
  * Why: the S10 sensorless work targets "swap the motor, keep the firmware".
  * Every number that depends on the motor (pole pairs, R, L, back-EMF, inertia,
  * current limits, top speed) lives here; board_cfg.h only aliases them, so a
  * new motor means editing this file and re-running the identification
  * procedure in docs/34 (sident / align ol / obs health).
  *
  * Units are SI: ohm, henry, V per electrical rad/s, kg*m^2, ampere, rpm.
  ******************************************************************************
  */

#ifndef __MOTOR_CFG_H
#define __MOTOR_CFG_H

/* ---------------------------------------------------------------- identity */
/* 7 pole pairs, 12 V, KV 330, phase R 2.3 ohm, L ~0.5 mH (see docs/33). */
#define MOTOR_POLE_PAIRS        7u
#define MOTOR_R_OHM             2.3f        /* phase resistance          */
#define MOTOR_L_H               0.0005f     /* phase inductance          */
#define MOTOR_KE_V_RAD_S        0.00413f    /* back-EMF, per elec rad/s  */
#define MOTOR_J_KGM2            9.0e-5f     /* rotor + load inertia      */

/* ----------------------------------------------------------------- limits */
/* The board's shunt chain saturates at +-1.32 A, so the usable current is
 * capped there no matter what the motor could take. */
#define MOTOR_I_RATED_A         1.00f       /* continuous target         */
#define MOTOR_I_MAX_A           1.20f       /* current-loop clamp        */
#define MOTOR_I_TRIP_A          1.28f       /* over-current trip         */

/* Command limit, kept at 2000 so the existing behaviour is unchanged. The
 * practical ceiling is lower: 12 V, 0.95 modulation -> 5.7 V available, which
 * with the KE above tops out near 1800 rpm (2000 rpm would need 6.06 V). */
#define MOTOR_RPM_MAX           2000.0f

/* --------------------------------------------------------- control seeds */
/* These are *starting points* that depend on R/L/J above; re-tune with the
 * sstep/step tools after a motor swap. */
#define MOTOR_FOC_KP            3.00f       /* V/A                       */
#define MOTOR_FOC_KI            11000.0f    /* V/(A*s)                   */
#define MOTOR_SPD_KP            0.006f      /* A/rpm                     */
#define MOTOR_SPD_KI            0.080f      /* A/(rpm*s)                 */

/* J/Kt in A per (rad/s^2): acceleration feedforward for the speed loop.
 *
 * CAREFUL: Kt in SI equals the back-EMF constant expressed per MECHANICAL
 * rad/s, i.e. KE_elec * pole_pairs (0.00413 * 7 = 0.0289 N*m/A). Using
 * KE_elec alone makes the feedforward 7x too large.
 *
 *   9e-5 / 0.0289 = 3.11e-3 A per rad/s^2   (matches the value validated in
 *   S8.3, where it cut the 100 rpm step overshoot from 96% to 8%)
 */
#define MOTOR_KT_NM_A           (MOTOR_KE_V_RAD_S * (float)MOTOR_POLE_PAIRS)
#define MOTOR_FF_ACC_A_RADS2    (MOTOR_J_KGM2 / MOTOR_KT_NM_A)

#endif /* __MOTOR_CFG_H */
