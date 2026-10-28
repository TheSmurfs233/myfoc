/**
  ******************************************************************************
  * @file    app_main.h
  * @brief   Application layer: state machine skeleton + main loop scheduling.
  *
  * S1 scope:
  *   - KEY -> LED toggle
  *   - state machine skeleton (IDLE / RUN / FAULT) switchable by command
  *   - serial debug (text commands + VOFA+ waveform)
  * Motor control (SVPWM / current loop) is added in S2..S6 by hooking into
  * the placeholders marked [S2] / [S3] ...
  ******************************************************************************
  */

#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Run state */
typedef enum {
    APP_ST_INIT = 0,
    APP_ST_IDLE,
    APP_ST_RUN,
    APP_ST_ALIGN,       /* forced alignment: fixed vector, rotor pulled to theta=0 */
    APP_ST_FAULT
} app_state_t;

/* ALIGN (electrical zero calibration) outcome */
typedef struct {
    uint8_t  active;        /* running right now                      */
    uint8_t  done;          /* finished at least once                  */
    uint8_t  fail;          /* 0 = ok, 1 = timeout, 2 = overcurrent,
                             * 3 = encoder or flash problem            */
    float    zero_deg;      /* captured zero when fail == 0            */
    float    m;             /* current modulation index during align   */
    float    err_deg;       /* measured electrical offset (deg)        */
    uint32_t n;             /* error samples accumulated               */
    float    err_fwd;       /* mean error, forward sweep (deg elec)    */
    float    err_rev;       /* mean error, reverse sweep (deg elec)    */
} app_align_t;

/* Start the alignment; requires the bridge running (start first).
 * Returns 0 on success, 2 = not running, 3 = already aligning. */
uint8_t app_align_start(void);
const app_align_t *app_align_info(void);

/* Control mode */
typedef enum {
    CTRL_IDLE = 0,      /* no PWM output */
    CTRL_OPENLOOP,      /* V/F open loop (S4) */
    CTRL_FOC            /* closed current loop (S6) */
} ctrl_mode_t;

typedef struct {
    uint32_t uptime_ms;
    uint8_t  state;
    uint8_t  mode;
    uint32_t key_cnt;
    uint32_t loop_khz;
    uint8_t  wave_on;
    uint16_t wave_rate_hz;
    uint32_t wave_frames;
    uint32_t wave_drop;
    uint32_t rx_drop;
    uint32_t tx_pending;
} app_status_t;

/* Open-loop V/f snapshot (S4), filled once per ADC frame while modulating. */
typedef struct {
    uint8_t on;          /* 1 = V/f is driving the bridge */
    uint8_t dir;         /* 0/1: 1 = reversed phase sequence */
    float   f_hz;        /* present electrical frequency */
    float   f_target;    /* ramp target */
    float   m;           /* modulation index actually applied */
    float   i_max;       /* largest of |ia|,|ib|,|ic| this frame */
} app_vf_info_t;

/* Control-chain phase probe. The ADC interrupt is the reference: "isr" records
 * where in the PWM period it fired (TIM1 CNT), the other three record how many
 * microseconds later each step ran (DWT). */
typedef struct {
    uint16_t cnt;
    uint16_t cnt_min;
    uint16_t cnt_max;
    uint8_t  valid;
} app_phase_isr_t;

typedef struct {
    uint32_t last_cyc;   /* DWT cycles since the ADC interrupt */
    uint32_t min_cyc;
    uint32_t max_cyc;
    uint8_t  valid;
} app_phase_t;

typedef struct {
    app_phase_isr_t isr;     /* ADC interrupt entry (the sample instant) */
    app_phase_t     pick;    /* ISR -> main loop picked the frame up    */
    app_phase_t     comp;    /* pick -> duties computed (V/f + SVPWM)   */
    app_phase_t     wr;      /* duties computed -> CCR1..3 written      */
} app_phase_info_t;

void                app_init(void);
void                app_loop(void);

const app_status_t *app_status(void);

/* 0 = OK, non-zero = rejection reason */
uint8_t app_start(void);
uint8_t app_stop(void);
uint8_t app_set_mode(uint8_t m);
void    app_wave_set(uint8_t on, uint16_t rate_hz);
void    app_counters_clear(void);

/* Open-loop V/f. set() starts modulating towards hz.
 * Returns: 0 = OK, 2 = not running (send start first), 3 = still calibrating. */
uint8_t app_vf_set(float hz);
uint8_t app_vf_off(void);
void    app_vf_set_acc(float hz_per_s);
uint8_t app_vf_set_dir(uint8_t dir);
const app_vf_info_t *app_vf_info(void);

/* ---------------- FOC current loop (S6) ---------------- */
typedef struct {
    float id_ref;
    float iq_ref;
    float id;
    float iq;
    float vd;
    float vq;
    float m;          /* modulation index used                */
    uint32_t faults;  /* frames aborted by the current guard  */
    float    spd_ref; /* ramped speed reference, rpm              */
    float    spd_target; /* commanded target, rpm                 */
    float    spd_rpm; /* measured speed used by the speed loop     */
    uint8_t  ready;   /* at least one FOC step has run        */
} app_foc_info_t;

/* Set the d/q current references (A). Requires RUN in CTRL_FOC.
 * Returns 0 = OK, 2 = not running, 4 = wrong mode, 5 = |I| out of range. */
uint8_t app_foc_set(float id, float iq);

/* Runtime PI tuning (takes effect immediately, integrators are reset).
 * Returns 0 = OK, 1 = value out of range, 4 = wrong mode. */
uint8_t app_foc_set_pi(float kp, float ki);

/* S7.2 feedforward: k_emf in V/(elec rad/s), l_h in H, gain 0..1. */
void app_foc_set_ff(float k_emf, float l_h, float gain);
void app_foc_get_ff(float *k_emf, float *l_h, float *gain);
void    app_foc_get_pi(float *kp, float *ki);

typedef struct {
    float   amp;          /* commanded iq step, A            */
    float   rise_ms;      /* 10% -> 90%                   */
    float   overshoot;    /* percent of the step             */
    float   err;          /* steady state error, percent     */
    float   ripple;       /* peak to peak in the tail, A     */
    float   ss;           /* measured steady state value, A   */
    float   peak;         /* highest measured value, A        */
    uint8_t valid;        /* results ready                   */
} app_step_info_t;

/* Run an automatic iq step (needs RUN + CTRL_FOC). The metric is computed
 * in firmware at 20 kHz, so it does not depend on the waveform rate.
 * Returns 0 = OK, 2 = not running, 4 = wrong mode, 5 = |I| out of range. */
/* ---------------- speed loop (S7.3) ---------------- */
/* Target in rpm, signed. 0 = speed loop off. */
uint8_t app_spd_set(float rpm);
void    app_spd_off(void);
uint8_t app_spd_set_pi(float kp, float ki);
void    app_spd_get_pi(float *kp, float *ki);

typedef struct {
    float   target;       /* commanded speed, rpm            */
    float   rise_ms;      /* 10% -> 90%                      */
    float   overshoot;    /* percent of the step             */
    float   err;          /* steady state error, percent     */
    float   settle_ms;    /* time to enter +-5% and stay     */
    float   ripple;       /* peak to peak in the tail, rpm   */
    float   ss;           /* measured steady state, rpm      */
    float   peak;         /* highest measured, rpm           */
    uint8_t valid;
} app_sstep_info_t;

/* Automated speed step (needs RUN + CTRL_FOC).
 * Returns 0 = OK, 2 = not running, 4 = wrong mode, 5 = too fast. */
uint8_t app_sstep_start(float rpm);
const app_sstep_info_t *app_sstep_info(void);

/* Friction feedforward: base current + slope per rpm (option C, S7.3).
 * Returns 0 = OK, 1 = out of range. */
uint8_t app_spd_set_ff(float base_a, float slope_a_per_rpm);
void    app_spd_get_ff(float *base_a, float *slope_a_per_rpm);

uint8_t app_step_start(float amp);
const app_step_info_t *app_step_info(void);
/* ---------------- protections (S7.6) ---------------- */
typedef enum {
    APP_FAULT_NONE = 0,
    APP_FAULT_OVERCURRENT = 1,
    APP_FAULT_OVERSPEED  = 2,
    APP_FAULT_STALL      = 3,
    APP_FAULT_UNDERVOLT  = 4,
    APP_FAULT_OVERVOLT   = 5,
    APP_FAULT_ENCODER    = 6,
    APP_FAULT_COUNT      = 7
} app_fault_t;

typedef struct {
    float    overspeed_rpm;
    uint16_t overspeed_ms;
    float    stall_rpm;
    float    stall_iq_a;
    uint16_t stall_ms;
    float    uv_v;
    float    ov_v;
    uint16_t vbus_ms;
} app_prot_t;

typedef struct {
    uint8_t  code;        /* last fault, APP_FAULT_NONE when clean   */
    float    value;       /* the measurement that tripped it         */
    uint32_t uptime_ms;   /* when it happened                        */
    uint16_t count[APP_FAULT_COUNT];
} app_fault_info_t;

/* Raise a fault: bridge off, speed loop released, references cleared. */
void app_fault_raise(uint8_t code, float value);
const app_fault_info_t *app_fault_info(void);
const app_prot_t       *app_prot_info(void);

typedef struct {
    float iq_min, iq_max;     /* measured iq over the window, A   */
    float spd_min, spd_max;   /* measured speed, rpm              */
    uint16_t n;               /* samples collected                */
} app_rip_t;

/* Start (or restart) a ripple measurement. */
void app_rip_start(void);
const app_rip_t *app_rip_info(void);
uint8_t app_prot_set(uint8_t which, float value);

const app_foc_info_t *app_foc_info(void);

const app_phase_info_t *app_phase_info(void);
void                    app_phase_reset(void);

const char *app_state_name(uint8_t st);
const char *app_mode_name(uint8_t m);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MAIN_H */
