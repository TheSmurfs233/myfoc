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
    APP_ST_FAULT
} app_state_t;

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

const app_phase_info_t *app_phase_info(void);
void                    app_phase_reset(void);

const char *app_state_name(uint8_t st);
const char *app_mode_name(uint8_t m);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MAIN_H */
