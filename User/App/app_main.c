/**
  ******************************************************************************
  * @file    app_main.c
  * @brief   Main loop and task scheduling.
  *
  * while(1) {
  *     bsp_key_poll()      key debounce (called every loop, >10kHz)
  *     key_action()        key event -> LED toggle
  *     app_cmd_poll()      serial command parsing (non-blocking)
  *     bsp_uart_tx_pump()  push DMA transmission
  *     task_1ms()          uptime / loop rate statistics
  *     task_wave()         VOFA+ waveform upload
  * }
  *
  * 2026-10-25 S1: LED/KEY/UART/state machine skeleton
  ******************************************************************************
  */

#include "app_main.h"
#include "app_cmd.h"
#include "board_cfg.h"
#include "bsp_adc.h"
#include "bsp_gpio.h"
#include "bsp_prof.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "mid_current.h"
#include "mid_log.h"
#include "mid_vofa.h"

/* ------------------------------------------------------------------ */
/* Internal state                                                      */
/* ------------------------------------------------------------------ */
static app_status_t s_st;
static uint32_t     s_loop_cnt;
static uint32_t     s_t_1ms;
static uint32_t     s_t_wave;
static uint16_t     s_wave_period_ms; /* upload period in ms */
static uint32_t     s_adc_seen;       /* ADC frame counter already consumed */

/* ------------------------------------------------------------------ */
/* Name helpers                                                        */
/* ------------------------------------------------------------------ */
const char *app_state_name(uint8_t st)
{
    switch (st) {
    case APP_ST_INIT:  return "INIT";
    case APP_ST_IDLE:  return "IDLE";
    case APP_ST_RUN:   return "RUN";
    case APP_ST_FAULT: return "FAULT";
    default:           return "?";
    }
}

const char *app_mode_name(uint8_t m)
{
    switch (m) {
    case CTRL_IDLE:     return "IDLE";
    case CTRL_OPENLOOP: return "OPENLOOP";
    case CTRL_FOC:      return "FOC";
    default:            return "?";
    }
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */
void app_init(void)
{
    s_st.uptime_ms    = 0u;
    s_st.state        = APP_ST_INIT;
    s_st.mode         = CTRL_IDLE;
    s_st.key_cnt      = 0u;
    s_st.loop_khz     = 0u;
    s_st.wave_on      = 0u;
    s_st.wave_rate_hz = DBG_WAVE_RATE_HZ;
    s_st.wave_frames  = 0u;
    s_st.wave_drop    = 0u;
    s_st.rx_drop      = 0u;
    s_st.tx_pending   = 0u;

    s_loop_cnt       = 0u;
    s_t_1ms          = HAL_GetTick();
    s_t_wave         = s_t_1ms;
    s_wave_period_ms = (uint16_t)(1000u / s_st.wave_rate_hz);
    s_adc_seen       = 0u;

    /* ---- peripherals ---- */
    bsp_prof_init();         /* DWT cycle counter, must be first */
    bsp_gpio_init();
    bsp_uart_init();
    bsp_pwm_init();          /* counter runs, six outputs stay off (MOE=0) */
    mid_current_init();      /* offsets to nominal, needs calibration */
    bsp_adc_init();          /* arm the injected group (triggered by TIM1_CC4) */
    /* Restore the zero-drift calibration from flash: with a valid record there
     * is nothing to calibrate and "start" goes straight to work. */
    (void)mid_current_load();
    app_cmd_init();

    /* ---- boot banner ---- */
    log_printf("");
    log_printf("=============================================");
    log_printf(" %s", BOARD_NAME);
    log_printf(" build = %s %s #%u", __DATE__, __TIME__, (unsigned)FW_BUILD_ID);
    log_printf(" core = %u MHz, FOC = %u Hz (Ts=%u us)",
               (unsigned)(SYS_CORE_CLK_HZ / 1000000u),
               (unsigned)FOC_FREQ_HZ,
               (unsigned)FOC_TS_US);
    log_printf(" pwm  = %u Hz (ARR=%u, DTG=%u)",
               (unsigned)PWM_FREQ_HZ_CALC, (unsigned)PWM_ARR, (unsigned)PWM_DTG_REG);
    log_printf(" cur  = Rs=%u mOhm x%u, %u uA/LSB, +-%u mA",
               (unsigned)(RSHUNT_OHM * 1000.0f + 0.5f), (unsigned)INA240_GAIN,
               (unsigned)(CUR_A_PER_LSB * 1000000.0f + 0.5f),
               (unsigned)(CUR_RANGE_A * 1000.0f + 0.5f));
    {
        /* Power-on self-check result: where the zero-drift offsets came from.
         * With a valid record nothing is calibrated later; without one the
         * first "start" calibrates and writes the record back. */
        const current_sample_t *cs = mid_current_get();

        if (cs->ready != 0u) {
            log_printf(" cal  = flash ok, off %u/%u/%u, gen=%u",
                       (unsigned)cs->offset[0], (unsigned)cs->offset[1],
                       (unsigned)cs->offset[2], (unsigned)cs->gen);
        } else {
            log_printf(" cal  = none, will calibrate at first start");
        }
    }
    log_printf(" uart = %u baud, VOFA+ = FireWater",
               (unsigned)DBG_UART_BAUD);
    log_printf("---------------------------------------------");
    log_printf(" KEY -> LED toggle");
    log_printf(" type 'help' for the command list");
    log_printf("=============================================");

    /* CubeMX regeneration silently restores TIM1.DeadTime from the .ioc; the
     * effective dead time is identical (60ns from the EG3033 either way) but a
     * mismatch means tim.c and board_cfg.h disagree, so say it out loud. */
    if (bsp_pwm_get_dtg() != (uint16_t)PWM_DTG_REG) {
        log_printf("ERR bdtr dtg=%u expected=%u (tim.c / CubeMX)",
                   (unsigned)bsp_pwm_get_dtg(), (unsigned)PWM_DTG_REG);
    }

    s_st.state = APP_ST_IDLE;
}

/* ------------------------------------------------------------------ */
/* State control                                                       */
/* ------------------------------------------------------------------ */
uint8_t app_start(void)
{
    if (s_st.state == APP_ST_FAULT) {
        return 1u;                       /* FAULT blocks start */
    }
    if (s_st.mode == CTRL_IDLE) {
        return 2u;                       /* no control mode selected */
    }
    if (s_st.mode != CTRL_OPENLOOP) {
        return 3u;                       /* CTRL_FOC has no current loop yet (S6) */
    }

    /* The ADC trigger (TIM1_CC4) only reaches the ADC while MOE=1, so the
     * zero-drift calibration can only run with the bridge enabled. Start at
     * the 50% zero vector: no phase-to-phase voltage, hence no current. The
     * 256 calibration frames take 12.8ms and complete in the background. */
    bsp_pwm_set_all_duty(PWM_START_DUTY);
    bsp_pwm_enable();                    /* preload duty, CCxE, then MOE */
    if (mid_current_get()->ready == 0u) {
        mid_current_restart_cal();
    }
    s_st.state = APP_ST_RUN;
    return 0u;
}

uint8_t app_stop(void)
{
    bsp_pwm_disable();                   /* MOE=0 first: six outputs low */
    bsp_pwm_set_all_duty(PWM_START_DUTY);
    s_st.state = APP_ST_IDLE;
    return 0u;
}

uint8_t app_set_mode(uint8_t m)
{
    if (m > (uint8_t)CTRL_FOC) { return 1u; }
    if (s_st.state == APP_ST_RUN) { return 2u; }   /* not while running */
    s_st.mode  = m;
    s_st.state = APP_ST_IDLE;
    return 0u;
}

void app_wave_set(uint8_t on, uint16_t rate_hz)
{
    /* FireWater CSV: 6 channels ~ 44B/frame, so 500Hz is ~22kB/s
     * (22% of a 1Mbaud link). HAL_GetTick only resolves 1ms, so 1000Hz would
     * leave no margin at all. */
    if (rate_hz < 1u)                { rate_hz = 1u; }
    if (rate_hz > DBG_WAVE_RATE_MAX) { rate_hz = DBG_WAVE_RATE_MAX; }
    s_st.wave_rate_hz = rate_hz;
    s_st.wave_on      = (on != 0u) ? 1u : 0u;
    s_wave_period_ms  = (uint16_t)(1000u / rate_hz);
    if (s_wave_period_ms == 0u) { s_wave_period_ms = 1u; }
    s_t_wave          = HAL_GetTick();
}

void app_counters_clear(void)
{
    s_st.wave_frames = 0u;
    s_st.wave_drop   = 0u;
    s_st.rx_drop     = 0u;
}

const app_status_t *app_status(void)
{
    return &s_st;
}

/* ------------------------------------------------------------------ */
/* Key action                                                          */
/* ------------------------------------------------------------------ */
static void key_action(void)
{
    if (bsp_key_take_press()) {
        bsp_led_toggle();
        s_st.key_cnt = bsp_key_press_count();
        log_printf("[KEY] pressed #%u -> LED %s",
                   (unsigned)s_st.key_cnt,
                   bsp_led_is_on() ? "ON" : "OFF");
    }
}

/* ------------------------------------------------------------------ */
/* 1ms task: uptime and loop rate statistics                           */
/* ------------------------------------------------------------------ */
static void task_1ms(uint32_t now)
{
    uint32_t dt = (uint32_t)(now - s_t_1ms);

    if (dt >= 1000u) {
        /* s_loop_cnt counts iterations over dt milliseconds.
         *   loops per second  = s_loop_cnt * 1000 / dt
         *   kHz               = loops per second / 1000 = s_loop_cnt / dt
         * (previous version forgot the second division and reported Hz as kHz) */
        s_st.loop_khz = (dt != 0u) ? (uint32_t)(s_loop_cnt / dt) : 0u;
        s_loop_cnt    = 0u;
        s_t_1ms       = now;
        s_st.rx_drop  = bsp_uart_rx_drop_count();
    }
}

/* ------------------------------------------------------------------ */
/* ADC task: consume the newest injected frame (20kHz)                 */
/* ------------------------------------------------------------------ */
static void task_adc(void)
{
    uint32_t n = bsp_adc_frames();
    uint32_t t0;
    uint16_t raw[3];

    if (n == s_adc_seen) { return; }
    s_adc_seen = n;

    t0 = bsp_prof_now();
    bsp_adc_get_raw(raw);
    mid_current_update(raw);
    bsp_prof_add(PROF_CONV, t0);
}

/* ------------------------------------------------------------------ */
/* VOFA+ waveform task                                                 */
/* ------------------------------------------------------------------ */
static void task_wave(uint32_t now)
{
    float    ch[7];
    float    a, b, c;
    uint32_t t0;
    const current_sample_t *cs;

    if (!s_st.wave_on) { return; }

    /* "now - last >= period" also absorbs HAL_GetTick jitter */
    if ((uint32_t)(now - s_t_wave) < (uint32_t)s_wave_period_ms) { return; }
    s_t_wave = now;

    t0 = bsp_prof_now();

    bsp_pwm_get_duty(&a, &b, &c);
    cs = mid_current_get();

    ch[0] = (float)now / 1000.0f;      /* uptime, s            */
    ch[1] = a;                         /* duty A               */
    ch[2] = b;                         /* duty B               */
    ch[3] = c;                         /* duty C               */
    ch[4] = cs->ia;                    /* phase currents, A    */
    ch[5] = cs->ib;
    ch[6] = cs->ic;

    if (vofa_send(ch, 7u) != 0u) {
        s_st.wave_frames++;
    } else {
        s_st.wave_drop++;
    }

    bsp_prof_add(PROF_WAVE, t0);
}

/* ------------------------------------------------------------------ */
/* Main loop                                                           */
/* ------------------------------------------------------------------ */
void app_loop(void)
{
    uint32_t now    = HAL_GetTick();
    uint32_t t_loop = bsp_prof_now();
    uint32_t t;

    s_st.uptime_ms = now;

    bsp_key_poll(now);
    key_action();

    t = bsp_prof_now();
    app_cmd_poll();
    bsp_prof_add(PROF_CMD, t);

    t = bsp_prof_now();
    bsp_uart_tx_pump();
    bsp_prof_add(PROF_TXPUMP, t);

    task_adc();
    task_1ms(now);
    task_wave(now);

    s_loop_cnt++;
    bsp_prof_add(PROF_LOOP, t_loop);
}
