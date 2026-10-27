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
#include "alg_foc.h"
#include "alg_spd.h"
#include "bsp_adc.h"
#include "bsp_vbus.h"
#include "bsp_gpio.h"
#include "bsp_prof.h"
#include "bsp_pwm.h"
#include "bsp_sys.h"
#include "bsp_uart.h"
#include "mid_current.h"
#include "mid_enc.h"
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
static uint32_t     s_t_enc;
static uint16_t     s_wave_period_ms; /* upload period in ms */
static uint32_t     s_adc_seen;       /* ADC frame counter already consumed */

/* ---- open-loop V/f (S4) ---- */
static alg_vf_t      s_vf;
static app_vf_info_t s_vf_info;

/* ---- FOC current loop (S6) ---- */
static alg_foc_t      s_foc;
static app_foc_info_t s_foc_info;
static uint8_t        s_foc_trip_cnt;

/* S7.3 speed loop. */
static alg_spd_t s_spd;
static uint8_t   s_spd_div;

/* S6.4: automatic current step measurement. */
static app_step_info_t s_step;
static uint8_t         s_step_phase;    /* 0 idle, 1 settle, 2 record  */
static uint32_t        s_step_t0;
static uint32_t        s_step_n;
static uint32_t        s_step_t10;
static uint32_t        s_step_t90;
static uint8_t         s_step_got10;
static uint8_t         s_step_got90;
static float           s_step_peak;
static float           s_step_tail_min;
static float           s_step_tail_max;
static float           s_step_tail_sum;
static uint32_t        s_step_tail_n;
static uint16_t      s_i_fold_cnt;
static uint16_t      s_i_trip_cnt;
static uint8_t       s_trig_status;     /* mid_trig_init() result */
static uint8_t       s_enc_div;         /* ENC_DIV prescaler */

/* ALIGN (S5 zero calibration) */
static app_align_t   s_align;
static uint32_t      s_align_t0;        /* tick at start                */
static float         s_align_err_sum;   /* accum. electrical error (rad)  */
static uint32_t      s_align_err_n;     /* samples accumulated            */
static float         s_align_fwd_sum;   /* forward sweep only (rad)       */
static uint32_t      s_align_fwd_n;
static float         s_align_rev_sum;   /* reverse sweep only (rad)       */
static uint32_t      s_align_rev_n;
static uint32_t      s_align_frames0;   /* encoder frame count at start   */

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
    case APP_ST_ALIGN: return "ALIGN";
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
    s_t_enc          = s_t_1ms;
    s_wave_period_ms = (uint16_t)(1000u / s_st.wave_rate_hz);
    s_adc_seen       = 0u;

    /* FOC defaults; the "pi" command changes them at runtime. */
    alg_foc_init(&s_foc, FOC_VDC_NOMINAL, FOC_M_MAX,
                 FOC_KP, FOC_KI, FOC_V_LIM);
    alg_foc_set_ff(&s_foc, FOC_K_EMF, FOC_L_H, FOC_FF_GAIN);
    s_foc.ff_lo = FOC_FF_LO_W;
    s_foc.ff_hi = FOC_FF_HI_W;

    bsp_vbus_init();

    alg_spd_init(&s_spd, SPD_KP, SPD_KI, FOC_I_MAX_A,
                 SPD_FF_BASE, SPD_FF_SLOPE, SPD_KICK, SPD_KICK_RPM);
    s_spd_div = 0u;
    s_foc.ff_lo = FOC_FF_LO_W;
    s_foc.ff_hi = FOC_FF_HI_W;

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
    mid_enc_init();                    /* MT6701: zero/dir from flash */
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
    log_printf(" enc  = %s, off %.1f deg, dir %u",
               (mid_enc_info()->src == ENC_SRC_FLASH) ? "flash" : "default",
               (double)mid_enc_info()->zero_deg,
               (unsigned)mid_enc_info()->dir);
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
/* ALIGN: pull the rotor onto the phase A axis to find the electrical  */
/* zero. A fixed voltage vector at theta = 0 makes the stator field    */
/* point along phase A; the permanent magnet turns to follow it.       */
/* ------------------------------------------------------------------ */
#define ALIGN_TWO_PI   6.283185307179586f

static void align_finish(uint8_t fail)
{
    bsp_pwm_disable();                   /* six outputs low before anything else */
    bsp_pwm_set_all_duty(PWM_START_DUTY);

    s_align.active = 0u;
    s_align.done   = 1u;
    s_align.fail   = fail;
    s_st.state     = (fail == 2u) ? APP_ST_FAULT : APP_ST_IDLE;

    if (fail == 0u) {
        log_printf("OK align done, zero %.2f deg, err %.2f (fwd %.2f rev %.2f)",
                   (double)s_align.zero_deg, (double)s_align.err_deg,
                   (double)s_align.err_fwd, (double)s_align.err_rev);
    } else if (fail == 1u) {
        log_printf("ERR align timeout, rotor did not settle");
    } else if (fail == 2u) {
        log_printf("ERR align overcurrent, trip");
    } else {
        log_printf("ERR align encoder or flash");
    }
}

static void task_align(void)
{
    const mid_enc_t        *e = mid_enc_info();
    const current_sample_t *cs;
    uint32_t now = HAL_GetTick();
    uint32_t el  = (uint32_t)(now - s_align_t0);
    uint32_t t_fwd_end = (uint32_t)ALIGN_RAMP_MS + (uint32_t)ALIGN_SWEEP_MS;
    uint32_t t_rev_end = t_fwd_end + (uint32_t)ALIGN_SWEEP_MS;
    float    m;
    float    theta = 0.0f;
    float    duty[3];
    float    imax;
    float    t;
    float    revs = ALIGN_SWEEP_HZ * ((float)ALIGN_SWEEP_MS / 1000.0f);
    uint8_t  accumulate = 0u;
    uint8_t  i;

    if (s_align.active == 0u) {
        return;
    }

    /* Current guard first: the vector is the thing that can hurt. */
    cs = mid_current_get();
    imax = fabsf(cs->ia);
    t = fabsf(cs->ib); if (t > imax) { imax = t; }
    t = fabsf(cs->ic); if (t > imax) { imax = t; }
    if (imax > ALIGN_I_MAX_A) {
        align_finish(2u);
        return;
    }

    /* ---- magnitude ramp ---- */
    if (el < (uint32_t)ALIGN_RAMP_MS) {
        m = ALIGN_M_START +
            ((ALIGN_M_MAX - ALIGN_M_START) * (float)el / (float)ALIGN_RAMP_MS);
        theta = 0.0f;
    }
    /* ---- forward sweep: 0 -> revs, accumulate after the transient ---- */
    else if (el < t_fwd_end) {
        float ts = (float)(el - (uint32_t)ALIGN_RAMP_MS) / 1000.0f;

        m = ALIGN_M_MAX;
        theta = ALIGN_TWO_PI * ALIGN_SWEEP_HZ * ts;
        if ((uint32_t)(el - (uint32_t)ALIGN_RAMP_MS) > (uint32_t)ALIGN_SETTLE_MS) {
            accumulate = 1u;
        }
    }
    /* ---- reverse sweep: revs -> 0 ---- */
    else if (el < t_rev_end) {
        float ts = (float)(el - t_fwd_end) / 1000.0f;

        m = ALIGN_M_MAX;
        theta = ALIGN_TWO_PI * (revs - (ALIGN_SWEEP_HZ * ts));
        if (theta < 0.0f) {
            theta += ALIGN_TWO_PI;
        }
        if ((uint32_t)(el - t_fwd_end) > (uint32_t)ALIGN_SETTLE_MS) {
            accumulate = 1u;
        }
    }
    /* ---- done: apply the averaged offset ---- */
    else {
        m = ALIGN_M_MAX;
        theta = 0.0f;
    }

    while (theta >= ALIGN_TWO_PI) { theta -= ALIGN_TWO_PI; }
    while (theta < 0.0f)          { theta += ALIGN_TWO_PI; }

    alg_svpwm(theta, m, duty);
    for (i = 0u; i < 3u; i++) {
        if (duty[i] < 0.0f) { duty[i] = 0.0f; }
        if (duty[i] > 1.0f) { duty[i] = 1.0f; }
    }
    bsp_pwm_set_duty(duty[0], duty[1], duty[2]);
    s_align.m = m;

    /* Abort when no NEW encoder frame arrives (frames stops advancing): the
     * "valid" flag only says that a frame arrived once, not that it still does. */
    if (e->frames == s_align_frames0) {
        if (el > 300u) {
            align_finish(3u);
        }
        return;
    }

    /* ---- accumulate the tracking error ---- */
    if (accumulate != 0u) {
        float enc_rad = e->elec_deg * 0.0174532925199433f;
        float err = enc_rad - theta;

        while (err >  3.14159265f) { err -= ALIGN_TWO_PI; }
        while (err < -3.14159265f) { err += ALIGN_TWO_PI; }

        s_align_err_sum += err;
        s_align_err_n++;
        if (el < t_fwd_end) {
            s_align_fwd_sum += err;
            s_align_fwd_n++;
        } else {
            s_align_rev_sum += err;
            s_align_rev_n++;
        }
    }

    if (el >= t_rev_end) {
        if (s_align_err_n == 0u) {
            align_finish(1u);
            return;
        }

        /* mean electrical error -> mechanical correction */
        {
            float err_elec_deg = (s_align_err_sum / (float)s_align_err_n)
                                 * 57.29577951308232f;
            float corr = err_elec_deg / (float)ENC_POLE_PAIRS;
            float nz   = mid_enc_info()->zero_deg + corr;

            s_align.err_deg = err_elec_deg;
            s_align.n       = s_align_err_n;
            s_align.err_fwd = (s_align_fwd_n != 0u)
                              ? (s_align_fwd_sum / (float)s_align_fwd_n)
                                * 57.29577951308232f : 0.0f;
            s_align.err_rev = (s_align_rev_n != 0u)
                              ? (s_align_rev_sum / (float)s_align_rev_n)
                                * 57.29577951308232f : 0.0f;

            (void)mid_enc_zero_set(nz);
            s_align.zero_deg = mid_enc_info()->zero_deg;
            align_finish(0u);
        }
        return;
    }

    if (el > ALIGN_TIMEOUT_MS) {
        align_finish(1u);
    }
}
uint8_t app_align_start(void)
{
    if (s_st.state != APP_ST_RUN) {
        return 2u;                       /* start the bridge first */
    }
    if (s_align.active != 0u) {
        return 3u;
    }

    /* The vector itself is written by task_align every frame (it ramps), so
     * nothing to do here beyond arming the state. */

    s_align.active     = 1u;
    s_align.done       = 0u;
    s_align.fail       = 0u;
    s_align.zero_deg   = mid_enc_info()->zero_deg;
    s_align_t0         = HAL_GetTick();
    s_align_err_sum    = 0.0f;
    s_align_err_n      = 0u;
    s_align_fwd_sum    = 0.0f;
    s_align_fwd_n      = 0u;
    s_align_rev_sum    = 0.0f;
    s_align_rev_n      = 0u;
    s_align_frames0    = mid_enc_info()->frames;
    s_align.err_deg    = 0.0f;
    s_align.n          = 0u;

    s_st.state = APP_ST_ALIGN;
    return 0u;
}

const app_align_t *app_align_info(void)
{
    return &s_align;
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
    if (s_st.mode == CTRL_FOC) {
        /* Only the STATE is cleared here. The gains are set once in
         * app_init(), so "pi <kp> <ki>" survives start/stop: that is the
         * order a person tunes in (set the gains, then start). */
        alg_foc_reset(&s_foc);
        s_foc_info.id_ref  = 0.0f;
        s_foc_info.iq_ref  = 0.0f;
        s_foc_info.ready   = 0u;
        s_foc_trip_cnt     = 0u;
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
    alg_foc_reset(&s_foc);
    app_spd_off();
    s_step_phase       = 0u;             /* cancel any running step */
    s_foc_info.id_ref  = 0.0f;
    s_foc_info.iq_ref  = 0.0f;
    s_foc_info.ready   = 0u;
    s_foc_trip_cnt     = 0u;
    s_vf_info.on       = 0u;
    s_vf_info.f_hz     = 0.0f;
    s_vf_info.f_target = 0.0f;
    s_vf_info.m        = 0.0f;
    s_i_fold_cnt       = 0u;
    s_i_trip_cnt       = 0u;

    bsp_pwm_disable();                   /* MOE=0 first: six outputs low */
    bsp_pwm_set_all_duty(PWM_START_DUTY);
    s_align.active = 0u;                 /* abort any alignment */
    s_st.state = APP_ST_IDLE;            /* also clears FAULT */
    return 0u;
}

uint8_t app_set_mode(uint8_t m)
{
    if (m > (uint8_t)CTRL_FOC) { return 1u; }
    if ((s_st.state == APP_ST_RUN) || (s_st.state == APP_ST_ALIGN)) {
        return 2u;                       /* not while running or aligning */
    }
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
    if (hz <= 0.0f) {
        return app_vf_off();             /* "0 Hz" means stop modulating */
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

/* ------------------------------------------------------------------ */
/* FOC current loop control (S6)                                       */
/* ------------------------------------------------------------------ */
uint8_t app_foc_set(float id, float iq)
{
    float mag;

    if (s_st.state != APP_ST_RUN) {
        return 2u;                       /* send start first */
    }
    if (s_st.mode != CTRL_FOC) {
        return 4u;                       /* mode 2 required */
    }
    if (mid_current_get()->ready == 0u) {
        return 3u;                       /* zero drift still being measured */
    }
    if (s_spd.on != 0u) {
        /* The speed loop rewrites iq_ref every 1 ms, so accepting this would
         * look like it worked and then do nothing. */
        return 6u;
    }

    mag = sqrtf((id * id) + (iq * iq));
    if (mag > FOC_I_MAX_A) {
        return 5u;                       /* caller asked for too much current */
    }

    s_foc_info.id_ref = id;
    s_foc_info.iq_ref = iq;
    return 0u;
}

uint8_t app_foc_set_pi(float kp, float ki)
{
    if ((kp < FOC_KP_MIN) || (kp > FOC_KP_MAX)) { return 1u; }
    if ((ki < FOC_KI_MIN) || (ki > FOC_KI_MAX)) { return 1u; }
    if (s_st.mode != CTRL_FOC) { return 4u; }

    s_foc.pi_d.kp = kp;
    s_foc.pi_q.kp = kp;
    s_foc.pi_d.ki = ki;
    s_foc.pi_q.ki = ki;
    alg_pi_reset(&s_foc.pi_d);          /* no stale wind-up with new gains */
    alg_pi_reset(&s_foc.pi_q);
    return 0u;
}

void app_foc_set_ff(float k_emf, float l_h, float gain)
{
    alg_foc_set_ff(&s_foc, k_emf, l_h, gain);
}

void app_foc_get_ff(float *k_emf, float *l_h, float *gain)
{
    if (k_emf != 0) { *k_emf = s_foc.k_emf; }
    if (l_h   != 0) { *l_h   = s_foc.l_h; }
    if (gain  != 0) { *gain  = s_foc.ff_gain; }
}

void app_foc_get_pi(float *kp, float *ki)
{
    if (kp != 0) { *kp = s_foc.pi_d.kp; }
    if (ki != 0) { *ki = s_foc.pi_d.ki; }
}

uint8_t app_spd_set(float rpm)
{
    if (s_st.state != APP_ST_RUN) { return 2u; }
    if (s_st.mode != CTRL_FOC)    { return 4u; }
    if ((rpm > SPD_RPM_MAX) || (rpm < -SPD_RPM_MAX)) { return 5u; }

    if ((rpm < 0.5f) && (rpm > -0.5f)) {
        app_spd_off();
        return 0u;
    }

    s_spd.spd_ref = rpm;
    s_spd.on      = 1u;
    s_spd_div     = (uint8_t)SPD_DIV;   /* run a step on the next frame */
    alg_pi_reset(&s_spd.pi);
    s_foc_info.spd_ref = rpm;
    return 0u;
}

void app_spd_off(void)
{
    s_spd.on      = 0u;
    s_spd.spd_ref = 0.0f;
    s_spd.out     = 0.0f;
    alg_pi_reset(&s_spd.pi);
    s_foc_info.spd_ref = 0.0f;
    s_foc_info.id_ref  = 0.0f;
    s_foc_info.iq_ref  = 0.0f;
}

uint8_t app_spd_set_pi(float kp, float ki)
{
    if ((kp < 0.0f) || (kp > 5.0f))   { return 1u; }
    if ((ki < 0.0f) || (ki > 50.0f))  { return 1u; }

    s_spd.pi.kp = kp;
    s_spd.pi.ki = ki;
    alg_pi_reset(&s_spd.pi);
    return 0u;
}

void app_spd_get_pi(float *kp, float *ki)
{
    if (kp != 0) { *kp = s_spd.pi.kp; }
    if (ki != 0) { *ki = s_spd.pi.ki; }
}

uint8_t app_step_start(float amp)
{
    if (s_st.state != APP_ST_RUN) { return 2u; }
    if (s_st.mode != CTRL_FOC)    { return 4u; }
    if ((amp > FOC_I_MAX_A) || (amp < -FOC_I_MAX_A)) { return 5u; }

    s_step.amp   = amp;
    s_step.valid = 0u;
    s_step_phase = 1u;
    s_step_t0    = HAL_GetTick();
    s_foc_info.id_ref = 0.0f;
    s_foc_info.iq_ref = 0.0f;
    return 0u;
}

const app_step_info_t *app_step_info(void)
{
    return &s_step;
}

const app_foc_info_t *app_foc_info(void)
{
    return &s_foc_info;
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
/* ------------------------------------------------------------------ */
/* FOC current loop (S6)                                               */
/* ------------------------------------------------------------------ */
static void task_foc(uint32_t t_pick)
{
    const current_sample_t *cs = mid_current_get();
    const mid_enc_t        *e  = mid_enc_info();
    float duty[3];
    float imax;
    float t;
    float id_ref;
    float iq_ref;
    float mag;
    float k;
    uint32_t t_comp;
    uint8_t  i;

    /* No encoder frame yet: hold the zero vector rather than invent an angle. */
    if (e->valid == 0u) {
        bsp_pwm_set_all_duty(PWM_START_DUTY);
        return;
    }

    /* ---- current guard (S7 will replace this with a hardware trip) ---- */
    imax = fabsf(cs->ia);
    t = fabsf(cs->ib); if (t > imax) { imax = t; }
    t = fabsf(cs->ic); if (t > imax) { imax = t; }

    if (imax >= FOC_I_TRIP_A) {
        if (++s_foc_trip_cnt >= FOC_TRIP_FRAMES) {
            bsp_pwm_disable();
            bsp_pwm_set_all_duty(PWM_START_DUTY);
            s_st.state        = APP_ST_FAULT;
            app_spd_off();
            s_foc_info.faults++;
            s_foc_trip_cnt    = 0u;
            log_printf("ERR overcurrent trip I=%d mA, type stop to reset",
                       (int)(imax * 1000.0f));
            return;
        }
    } else {
        s_foc_trip_cnt = 0u;
    }

    /* ---- S6.4 step generator: overrides the references while running ---- */
    if (s_step_phase != 0u) {
        uint32_t el = HAL_GetTick() - s_step_t0;

        if (s_step_phase == 1u) {        /* settle at zero current */
            s_foc_info.id_ref = 0.0f;
            s_foc_info.iq_ref = 0.0f;
            if (el >= (uint32_t)STEP_SETTLE_MS) {
                s_step_phase  = 2u;
                s_step_t0     = HAL_GetTick();
                s_step_n      = 0u;
                s_step_t10    = 0u;
                s_step_t90    = 0u;
                s_step_got10  = 0u;
                s_step_got90  = 0u;
                s_step_peak   = 0.0f;
                s_step_tail_min =  1e9f;
                s_step_tail_max = -1e9f;
                s_step_tail_sum = 0.0f;
                s_step_tail_n   = 0u;
                s_foc_info.iq_ref = s_step.amp;
            }
        }
        else if (s_step_phase == 2u) {   /* record the response */
            /* Recompute the elapsed time: s_step_t0 was just reset when this
             * phase started, and reusing the value computed at the top of the
             * block made the window look 100 ms old, so the report fired on
             * the first frame and the measurement was always empty. */
            uint32_t er = HAL_GetTick() - s_step_t0;

            s_foc_info.id_ref = 0.0f;
            s_foc_info.iq_ref = s_step.amp;

            /* measured iq from the previous step is the sample for this frame */
            s_step_n++;
            if (s_step_n > 4u) {         /* skip the one-frame delay */
                /* Magnitudes, so "step -0.30" measures the same way. */
                float y    = fabsf(s_foc.iq);
                float amp  = fabsf(s_step.amp);
                float y10  = 0.1f * amp;
                float y90  = 0.9f * amp;

                if ((s_step_got10 == 0u) && (y >= y10)) {
                    s_step_got10 = 1u;
                    s_step_t10   = s_step_n;
                }
                if ((s_step_got90 == 0u) && (y >= y90)) {
                    s_step_got90 = 1u;
                    s_step_t90   = s_step_n;
                }
                if (y > s_step_peak) { s_step_peak = y; }

                /* tail window = last 5 ms, used for error and ripple */
                if (er >= ((uint32_t)STEP_RECORD_MS - 5u)) {
                    if (y < s_step_tail_min) { s_step_tail_min = y; }
                    if (y > s_step_tail_max) { s_step_tail_max = y; }
                    s_step_tail_sum += y;
                    s_step_tail_n++;
                }
            }

            if (er >= (uint32_t)STEP_RECORD_MS) {
                s_step_phase = 0u;
                s_step.valid = 1u;
                s_step.rise_ms = -1.0f;      /* -1 = not measurable */
                if ((s_step_got10 != 0u) && (s_step_got90 != 0u) &&
                    (s_step_t90 >= s_step_t10)) {
                    s_step.rise_ms = (float)(s_step_t90 - s_step_t10) * 0.05f;
                }
                s_step.overshoot = 0.0f;
                if (fabsf(s_step.amp) > 0.05f) {
                    s_step.overshoot =
                        ((s_step_peak - fabsf(s_step.amp)) / fabsf(s_step.amp))
                        * 100.0f;
                    if (s_step.overshoot < 0.0f) { s_step.overshoot = 0.0f; }
                }
                s_step.err = 0.0f;
                s_step.ss  = 0.0f;
                if (s_step_tail_n != 0u) {
                    float avg = s_step_tail_sum / (float)s_step_tail_n;
                    s_step.ss = avg;
                    if (fabsf(s_step.amp) > 0.05f) {
                        s_step.err = ((fabsf(avg) - fabsf(s_step.amp))
                                      / fabsf(s_step.amp)) * 100.0f;
                        if (s_step.err < 0.0f) { s_step.err = -s_step.err; }
                    }
                    s_step.ripple = s_step_tail_max - s_step_tail_min;
                }
                s_step.peak = s_step_peak;
                s_foc_info.iq_ref = 0.0f;
                s_foc_info.id_ref = 0.0f;
                /* rise == 0 means the current crossed 10% and 90% inside a
                 * single 50 us sample: at 0.3 A the slew is voltage limited
                 * (di/dt = V/L ~ 11 A/ms), so there is no ramp to resolve. */
                if (s_step.rise_ms < 0.0f) {
                    log_printf("OK step %.2f A, rise n/a, over %.1f%%, err %.1f%%, rip %.0f mA",
                               (double)s_step.amp, (double)s_step.overshoot,
                               (double)s_step.err,
                               (double)(s_step.ripple * 1000.0f));
                } else {
                    log_printf("OK step %.2f A, rise %.2f ms, over %.1f%%, err %.1f%%, rip %.0f mA",
                               (double)s_step.amp, (double)s_step.rise_ms,
                               (double)s_step.overshoot, (double)s_step.err,
                               (double)(s_step.ripple * 1000.0f));
                }
                log_printf(" ss %.3f A, peak %.3f A, tail n=%u",
                           (double)s_step.ss, (double)s_step.peak,
                           (unsigned)s_step_tail_n);
            }
        }
    }

    /* ---- S7.3 speed loop: 1 kHz, drives iq_ref (d stays at zero) ---- */
    if (s_spd.on != 0u) {
        s_spd_div++;
        if (s_spd_div >= (uint8_t)SPD_DIV) {
            s_spd_div = 0u;
            (void)alg_spd_step(&s_spd, e->spd_rpm, SPD_TS_S);
            s_foc_info.iq_ref = s_spd.out;
            s_foc_info.id_ref = 0.0f;
        }
        s_foc_info.spd_rpm = e->spd_rpm;
    }

    /* ---- reference vector limit: never ask for more than the ADC range ---- */
    id_ref = s_foc_info.id_ref;
    iq_ref = s_foc_info.iq_ref;
    mag = sqrtf((id_ref * id_ref) + (iq_ref * iq_ref));
    if (mag > FOC_I_MAX_A) {
        k = FOC_I_MAX_A / mag;
        id_ref *= k;
        iq_ref *= k;
    }

    /* S7.2: electrical angular speed for the feedforward, signed. */
    (void)alg_foc_step(&s_foc, cs->ia, cs->ib, cs->ic,
                       e->elec_deg * 0.0174532925199433f,
                       e->spd_hz * 6.283185307179586f,
                       id_ref, iq_ref, FOC_TS, duty);

    for (i = 0u; i < 3u; i++) {
        if (duty[i] < 0.0f) { duty[i] = 0.0f; }
        if (duty[i] > 1.0f) { duty[i] = 1.0f; }
    }

    t_comp = bsp_prof_now();
    phase_mark(&s_phase.comp, t_pick);
    bsp_pwm_set_duty(duty[0], duty[1], duty[2]);
    phase_mark(&s_phase.wr, t_comp);
    bsp_prof_add(PROF_CONV, t_pick);

    s_foc_info.id    = s_foc.id;
    s_foc_info.iq    = s_foc.iq;
    s_foc_info.vd    = s_foc.vd;
    s_foc_info.vq    = s_foc.vq;
    s_foc_info.m     = s_foc.m;
    s_foc_info.ready = 1u;
}

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
/* Encoder poll while stopped (1kHz)                                   */
/*                                                                     */
/* The ADC trigger is gated by MOE, so no frames arrive while the      */
/* bridge is off and the 20kHz path in task_adc never runs. Poll the   */
/* encoder here instead so "enc" and "enc zero" work at standstill.    */
/* ------------------------------------------------------------------ */
static void task_enc_slow(uint32_t now)
{
    if (s_st.state == APP_ST_RUN) {
        return;                      /* task_adc already reads at 20kHz */
    }
    if ((uint32_t)(now - s_t_enc) < ENC_POLL_MS) {
        return;
    }
    s_t_enc = now;
    {
        uint32_t t0 = bsp_prof_now();

        mid_enc_update();
        bsp_prof_add(PROF_ENC, t0);
    }
}

/* ------------------------------------------------------------------ */
/* 1ms task: uptime and loop rate statistics                           */
/* ------------------------------------------------------------------ */
static void task_1ms(uint32_t now)
{
    uint32_t dt = (uint32_t)(now - s_t_1ms);

    /* S7.5: the bus voltage moves slowly; 1 kHz with a 5 ms filter is
     * plenty and keeps the control loop free of extra work. */
    if (bsp_vbus_update() != 0u) {
        alg_foc_set_vdc(&s_foc, bsp_vbus_volts());
    }

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
    uint32_t t_enc;
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

    if (s_st.mode == CTRL_FOC) {
        /* The current loop needs the angle for Park, so the encoder frame is
         * collected first. The SPI RX DMA was started in the ADC ISR, so the
         * data is normally already there (measured: 0 late frames at 20 kHz). */
        t_enc = bsp_prof_now();
        mid_enc_update();
        bsp_prof_add(PROF_ENC, t_enc);
    } else {
        /* S4: the V/f ramp runs once per ADC frame, right after the currents
         * are fresh. */
        task_vf(t0);
    }

    if ((s_st.mode == CTRL_FOC) && (s_st.state == APP_ST_RUN)) {
        task_foc(t0);
    }

    /* S5: in V/f the encoder is read AFTER the control chain, so its transfer
     * can never delay the duty update. FOC already collected it above. */
    if (s_st.mode == CTRL_FOC) {
        /* already read */
    } else if (ENC_DIV <= 1u) {
        t_enc = bsp_prof_now();
        mid_enc_update();
        bsp_prof_add(PROF_ENC, t_enc);
    } else {
        s_enc_div++;
        if (s_enc_div >= ENC_DIV) {
            s_enc_div = 0u;
            t_enc = bsp_prof_now();
            mid_enc_update();
            bsp_prof_add(PROF_ENC, t_enc);
        }
    }

    /* S5: supervised forced alignment (fixed vector already written) */
    if (s_st.state == APP_ST_ALIGN) {
        task_align();
    }
}

/* ------------------------------------------------------------------ */
/* VOFA+ waveform task                                                 */
/* ------------------------------------------------------------------ */
static void task_wave(uint32_t now)
{
    float    ch[WAVE_CH];
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
    /* S5: encoder angles and speed so VOFA+ can compare them with the
     * commanded V/f frequency (same unit: electrical Hz). */
    ch[7] = mid_enc_info()->mech_deg;
    ch[8] = mid_enc_info()->elec_deg;
    ch[9] = mid_enc_info()->spd_hz;
    /* S6: FOC currents and references. Plotting the measured and the
     * commanded value together is what makes a current step visible. */
    ch[10] = s_foc_info.id;              /* measured id, A   */
    ch[11] = s_foc_info.iq;              /* measured iq, A   */
    ch[12] = s_foc_info.id_ref;          /* commanded id, A  */
    ch[13] = s_foc_info.iq_ref;          /* commanded iq, A  */
    ch[14] = s_foc_info.vd;              /* V                */
    ch[15] = s_foc_info.vq;              /* V                */
    ch[16] = bsp_vbus_volts();           /* DC bus voltage, V */

    if (vofa_send(ch, (uint8_t)WAVE_CH) != 0u) {
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
    task_enc_slow(now);
    task_1ms(now);
    task_wave(now);

    s_loop_cnt++;
    bsp_prof_add(PROF_LOOP, t_loop);
}
