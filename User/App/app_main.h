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

void                app_init(void);
void                app_loop(void);

const app_status_t *app_status(void);

/* 0 = OK, non-zero = rejection reason */
uint8_t app_start(void);
uint8_t app_stop(void);
uint8_t app_set_mode(uint8_t m);
void    app_wave_set(uint8_t on, uint16_t rate_hz);
void    app_counters_clear(void);

const char *app_state_name(uint8_t st);
const char *app_mode_name(uint8_t m);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MAIN_H */
