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
#include "alg_svpwm.h"
#include "alg_vf.h"
#include "bsp_adc.h"
#include "bsp_gpio.h"
#include "bsp_prof.h"
#include "bsp_pwm.h"
#include "bsp_sys.h"
#include "bsp_uart.h"
#include "mid_current.h"
#include "mid_log.h"
#include "mid_trig.h"
#include <math.h>
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

/* ---- open-loop V/f (S4) ---- */
static alg_vf_t      s_vf;
static app_vf_info_t s_vf_info;
static uint16_t      s_i_fold_cnt;
static uint16_t      s_i_trip_cnt;
static uint8_t       s_trig_status;     /* mid_trig_init() result */

/* ---- where each control step lands inside the 50us PWM period ---- */
static app_phase_info_t s_phase;

static void phase_mark_isr(uint16_t cnt)
{
    if (bsp_prof_enabled() == 0u) { return; }

    if (s_phase.isr.valid == 0u) {
        s_phase.isr.cnt_min = cnt;
        s_phase.isr.cnt_max = cnt;
        s_phase.isr.valid   = 1u;
    } else {
        if (cnt < s_phase.isr.cnt_min) { s_phase.isr.cnt_min = cnt; }
        if (cnt > s_phase.isr.cnt_max) { s_phase.isr.cnt_max = cnt; }
    }
    s_phase.isr.cnt = cnt;
}

/* Microseconds since t_ref (the ADC interrupt that produced this frame),
 * kept in DWT cycles: converting to us here would cost a 64-bit division per
 * mark (__aeabi_uldivmod, ~1us) and inflate the very number being measured.
 * The command handler converts once when printing. */
#define PHASE_MAX_CYC   340000u        /* 2ms at 170MHz: anything larger is bogus */

static void phase_mark(app_phase_t *p, uint32_t t_ref)
{
    uint32_t cyc;

    /* The probe follows the profiler switch: "prof off" leaves the control
     * chain completely uninstrumented, which is how S6's real-time budget
     * should be evaluated. */
    if (bsp_prof_enabled() == 0u) {
        return;
    }

    cyc = bsp_prof_now() - t_ref;

    if (cyc > PHASE_MAX_CYC) {
        return;                        /* the ADC interrupted again meanwhile */
    }
    if (p->valid == 0u) {
        p->min_cyc = cyc;
        p->max_cyc = cyc;
        p->valid   = 1u;
    } else {
        if (cyc < p->min_cyc) { p->min_cyc = cyc; }
        if (cyc > p->max_cyc) { p->max_cyc = cyc; }
    }
    p->last_cyc = cyc;
}

const app_phase_info_t *app_phase_info(void)
{
    return &s_phase;
}

void app_phase_reset(void)
{
    s_phase.isr.valid  = 0u;
    s_phase.isr.cnt    = 0u;
    s_phase.isr.cnt_min = 0u;
    s_phase.isr.cnt_max = 0u;
    s_phase.pick.valid = 0u;
    s_phase.comp.valid = 0u;
    s_phase.wr.valid   = 0u;
    s_phase.pick.last_cyc = s_phase.pick.min_cyc = s_phase.pick.max_cyc = 0u;
    s_phase.comp.last_cyc = s_phase.comp.min_cyc = s_phase.comp.max_cyc = 0u;
    s_phase.wr.last_cyc   = s_phase.wr.min_cyc   = s_phase.wr.max_cyc   = 0u;
}

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

    alg_vf_init(&s_vf);
    s_vf_info.on       = 0u;
    s_vf_info.dir      = 0u;
    s_vf_info.f_hz     = 0.0f;
    s_vf_info.f_target = 0.0f;
    s_vf_info.m        = 0.0f;
    s_vf_info.i_max    = 0.0f;
    s_i_fold_cnt       = 0u;
    s_i_trip_cnt       = 0u;
    app_phase_reset();

    /* ---- peripherals ---- */
    bsp_sys_init();          /* ART: flash prefetch + I-cache, must be first */
    bsp_prof_init();         /* DWT cycle counter, must be first */
    s_trig_status = mid_trig_init();   /* LUT: always OK; CORDIC: self-test */
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
    log_printf(" trig = %s%s", mid_trig_name(),
               (s_trig_status != 0u) ? " (SELF TEST FAILED)" : "");
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
    alg_vf_stop(&s_vf);                  /* drop the V/f ramp first */
    s_vf_info.on       = 0u;
    s_vf_info.f_hz     = 0.0f;
    s_vf_info.f_target = 0.0f;
    s_vf_info.m        = 0.0f;
    s_i_fold_cnt       = 0u;
    s_i_trip_cnt       = 0u;

    bsp_pwm_disable();                   /* MOE=0 first: six outputs low */
    bsp_pwm_set_all_duty(PWM_START_DUTY);
    s_st.state = APP_ST_IDLE;            /* also clears FAULT */
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

/* ------------------------------------------------------------------ */
/* Open-loop V/f (S4)                                                  */
/* ------------------------------------------------------------------ */
uint8_t app_vf_set(float hz)
{
    if (s_st.state != APP_ST_RUN) {
        return 2u;                       /* bridge not running */
    }
    if (mid_current_get()->ready == 0u) {
        return 3u;                       /* zero drift still being measured */
    }
    alg_vf_start(&s_vf, hz);
    s_i_fold_cnt = 0u;
    s_i_trip_cnt = 0u;

    /* Publish immediately: "pwm" is refused while vf is on, and that check
     * must not wait for the next ADC frame. */
    s_vf_info.on       = 1u;
    s_vf_info.dir      = s_vf.dir;
    s_vf_info.f_target = s_vf.f_target;
    s_vf_info.f_hz     = s_vf.f_hz;
    s_vf_info.m        = 0.0f;
    return 0u;
}

uint8_t app_vf_off(void)
{
    alg_vf_stop(&s_vf);
    s_vf_info.on       = 0u;
    s_vf_info.f_hz     = 0.0f;
    s_vf_info.f_target = 0.0f;
    s_vf_info.m        = 0.0f;
    s_vf_info.i_max    = 0.0f;
    if (s_st.state == APP_ST_RUN) {
        bsp_pwm_set_all_duty(PWM_START_DUTY);
    }
    return 0u;
}

void app_vf_set_acc(float hz_per_s)
{
    if (hz_per_s < VF_ACC_MIN_HZ_PER_S) { hz_per_s = VF_ACC_MIN_HZ_PER_S; }
    if (hz_per_s > VF_ACC_MAX_HZ_PER_S) { hz_per_s = VF_ACC_MAX_HZ_PER_S; }
    s_vf.acc_hz_s = hz_per_s;
}

uint8_t app_vf_set_dir(uint8_t dir)
{
    if (dir > 1u) { return 1u; }
    s_vf.dir = dir;
    s_vf_info.dir = dir;
    return 0u;
}

const app_vf_info_t *app_vf_info(void)
{
    return &s_vf_info;
}

/* One V/f step per ADC frame (20kHz). ts is exact, so the angle integration
 * has no jitter; S6 will move this into the ADC interrupt.
 * t_pick = DWT timestamp taken as the main loop picked the frame up, so the
 * "comp" and "wr" marks measure stage durations, not absolute offsets. */
static void task_vf(uint32_t t_pick)
{
    const current_sample_t *cs;
    float theta;
    float m;
    float duty[3];
    float imax;
    float t;

    if (s_vf.on == 0u) {
        return;
    }

    theta = alg_vf_step(&s_vf, VF_TS_S);

    /* ---- current guard: there is no hardware protection until S7 ---- */
    cs = mid_current_get();
    imax = fabsf(cs->ia);
    t = fabsf(cs->ib); if (t > imax) { imax = t; }
    t = fabsf(cs->ic); if (t > imax) { imax = t; }

    if (imax >= VF_I_TRIP_A) {
        if (++s_i_trip_cnt >= VF_I_TRIP_FRAMES) {
            alg_vf_stop(&s_vf);
            bsp_pwm_disable();           /* MOE=0 immediately */
            bsp_pwm_set_all_duty(PWM_START_DUTY);
            s_st.state           = APP_ST_FAULT;
            s_vf_info.on         = 0u;
            s_vf_info.m          = 0.0f;
            s_i_trip_cnt         = 0u;
            log_printf("ERR overcurrent trip I=%d mA, type stop to reset",
                       (int)(imax * 1000.0f));
            return;
        }
    } else {
        s_i_trip_cnt = 0u;
    }

    if (imax > VF_I_FOLD_A) {
        if (++s_i_fold_cnt >= VF_I_FOLD_FRAMES) {
            s_i_fold_cnt = 0u;
            s_vf.m_scale *= 0.98f;
            if (s_vf.m_scale < VF_M_SCALE_MIN) { s_vf.m_scale = VF_M_SCALE_MIN; }
        }
    } else {
        s_i_fold_cnt = 0u;
        if (imax < VF_I_FOLD_BACK_A) {
            s_vf.m_scale *= 1.001f;
            if (s_vf.m_scale > 1.0f) { s_vf.m_scale = 1.0f; }
        }
    }

    m = alg_vf_mod_index(&s_vf);
    alg_svpwm(theta, m, duty);
    {
        uint32_t t_comp = bsp_prof_now();

        phase_mark(&s_phase.comp, t_pick);      /* V/f + SVPWM cost */
        bsp_pwm_set_duty(duty[0], duty[1], duty[2]);
        phase_mark(&s_phase.wr, t_comp);        /* CCR write cost */
    }

    s_vf_info.on       = 1u;
    s_vf_info.dir      = s_vf.dir;
    s_vf_info.f_hz     = s_vf.f_hz;
    s_vf_info.f_target = s_vf.f_target;
    s_vf_info.m        = m;
    s_vf_info.i_max    = imax;
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
    uint32_t t_ref;                  /* ISR timestamp, snapshotted once */
    uint16_t raw[3];

    if (n == s_adc_seen) { return; }
    s_adc_seen = n;

    t_ref = bsp_adc_isr_cyc();

    t0 = bsp_prof_now();
    phase_mark(&s_phase.pick, t_ref);    /* how late the loop noticed the frame */
    bsp_adc_get_raw(raw);
    mid_current_update(raw);
    bsp_prof_add(PROF_CONV, t0);

    phase_mark_isr(bsp_adc_isr_cnt());

    /* S4: the V/f ramp runs once per ADC frame, right after the currents are
     * fresh. S6 will fold the current loop into the same place. */
    task_vf(t0);
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
