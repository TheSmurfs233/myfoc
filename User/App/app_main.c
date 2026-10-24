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
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "mid_log.h"
#include "mid_vofa.h"

/* ------------------------------------------------------------------ */
/* Internal state                                                      */
/* ------------------------------------------------------------------ */
static app_status_t s_st;
static uint32_t     s_loop_cnt;
static uint32_t     s_t_1ms;
static uint32_t     s_t_wave;
static float        s_wave_phase01;   /* 0..1 wrap, for the test sine */
static uint16_t     s_wave_period_ms; /* upload period in ms */

/* 64-entry sine table (sin * 1000) used to verify the VOFA+ chain */
static const int16_t s_sin_tab[64] = {
       0,   98,  195,  290,  383,  471,  556,  634,
     707,  773,  831,  883,  924,  957,  981,  995,
    1000,  995,  981,  957,  924,  883,  831,  773,
     707,  634,  556,  471,  383,  290,  195,   98,
       0,  -98, -195, -290, -383, -471, -556, -634,
    -707, -773, -831, -883, -924, -957, -981, -995,
   -1000, -995, -981, -957, -924, -883, -831, -773,
    -707, -634, -556, -471, -383, -290, -195,  -98
};

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
    s_wave_phase01   = 0.0f;
    s_wave_period_ms = (uint16_t)(1000u / s_st.wave_rate_hz);

    /* ---- peripherals ---- */
    bsp_gpio_init();
    bsp_uart_init();
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
    log_printf(" uart = %u baud, VOFA+ = FireWater",
               (unsigned)DBG_UART_BAUD);
    log_printf("---------------------------------------------");
    log_printf(" KEY -> LED toggle");
    log_printf(" type 'help' for the command list");
    log_printf("=============================================");

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
    s_st.state = APP_ST_RUN;
    /* [S2] hook here: bsp_pwm_enable(1) + ALIGN sequence */
    return 0u;
}

uint8_t app_stop(void)
{
    /* [S2] hook here: bsp_pwm_enable(0) to shut the bridge down */
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
/* VOFA+ waveform task                                                 */
/* ------------------------------------------------------------------ */
static void task_wave(uint32_t now)
{
    float    ch[6];
    float    pos, frac, v0, v1;
    uint16_t i0;

    if (!s_st.wave_on) { return; }

    /* "now - last >= period" also absorbs HAL_GetTick jitter */
    if ((uint32_t)(now - s_t_wave) < (uint32_t)s_wave_period_ms) { return; }
    s_t_wave = now;

    /* 1Hz test sine: phase advances by f/rate per frame.
     * Linear interpolation between table entries - at 500Hz a raw 64-entry
     * table repeats each value for ~8 frames and looks like a staircase. */
    s_wave_phase01 += (1.0f / (float)s_st.wave_rate_hz);
    if (s_wave_phase01 >= 1.0f) { s_wave_phase01 -= 1.0f; }
    pos = s_wave_phase01 * 64.0f;
    i0  = (uint16_t)pos;
    if (i0 > 63u) { i0 = 63u; }
    frac = pos - (float)i0;
    v0   = (float)s_sin_tab[i0];
    v1   = (float)s_sin_tab[(uint16_t)((i0 + 1u) & 63u)];
    v0   = v0 + ((v1 - v0) * frac);

    ch[0] = (float)now / 1000.0f;              /* uptime, s */
    ch[1] = (float)s_st.state;                 /* state machine */
    ch[2] = (float)s_st.mode;                  /* control mode */
    ch[3] = (float)s_st.key_cnt;               /* key press count */
    ch[4] = v0;                                /* 1Hz test sine, +-1000 */
    ch[5] = (float)s_st.loop_khz;              /* main loop rate, kHz */

    if (vofa_send(ch, 6u) != 0u) {
        s_st.wave_frames++;
    } else {
        s_st.wave_drop++;
    }
}

/* ------------------------------------------------------------------ */
/* Main loop                                                           */
/* ------------------------------------------------------------------ */
void app_loop(void)
{
    uint32_t now = HAL_GetTick();

    s_st.uptime_ms = now;

    bsp_key_poll(now);
    key_action();

    app_cmd_poll();

    bsp_uart_tx_pump();

    task_1ms(now);
    task_wave(now);

    s_loop_cnt++;
}
