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
#include "alg_obs.h"
#include "alg_if.h"
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
static uint32_t     s_t_1k;   /* 1 kHz task timestamp */
static uint32_t     s_t_wave;
static uint32_t     s_t_enc;
static uint16_t     s_wave_period_ms; /* upload period in ms */
static uint32_t     s_adc_seen;       /* ADC frame counter already consumed */

/* ---- open-loop V/f (S4) ---- */
static alg_vf_t      s_vf;
static uint8_t       s_obs_on;    /* 1 = observer running (shadow) */
/* S10.5 hand-over */
static uint8_t   s_sfoc;      /* 1 = sensorless: angle and speed from the observer */
static uint8_t   s_ho_state;   /* 0 idle, 1 watching, 2 blending, 3 done */
static uint16_t  s_ho_cnt;
static float     s_ho_w;       /* blend weight 0..1 */
static float     s_ho_dth;     /* |theta_obs - theta_if|, rad */

static alg_if_t      s_if;       /* S10.4 open-loop ramp */
static alg_obs_t     s_obs;      /* S10.1 shadow observer */
static app_vf_info_t s_vf_info;

/* ---- FOC current loop (S6) ---- */
static alg_foc_t      s_foc;
static app_foc_info_t s_foc_info;
static uint8_t        s_foc_trip_cnt;

/* S7.6 protections. */
static app_prot_t       s_prot;
static app_fault_info_t s_fault;
static uint32_t s_prot_start_ms;   /* start of the grace window        */
static uint32_t s_prot_ov_ms;      /* overspeed debounce               */
static uint32_t s_prot_stall_ms;   /* stall debounce                   */
static uint32_t s_prot_uv_ms;      /* under-voltage debounce           */
static uint32_t s_prot_ovv_ms;     /* over-voltage debounce            */
static uint16_t s_prot_enc_cnt;    /* consecutive bad encoder frames   */
static uint32_t s_prot_bad_seen;   /* last encoder good-frame counter  */
static uint16_t s_prot_enc_zero_cnt;/* consecutive raw==0 frames        */

/* S7.3 speed loop. */
static alg_spd_t s_spd;
static uint8_t   s_spd_div;
static uint32_t  s_spd_kick_t0;

/* Ripple diagnostic: min/max of iq and speed over a short window. */
static app_rip_t  s_rip;
static uint32_t   s_rip_t0;
static uint8_t    s_rip_on;  /* start of the kick window */
static float     s_spd_target;    /* commanded, before the ramp  */
static float     s_spd_ramp_hz;   /* rpm per second              */

/* S7.4 speed step measurement. */
static app_sstep_info_t s_sstep;
static uint8_t   s_sstep_phase;   /* 0 idle, 1 settle, 2 record  */
static uint32_t  s_sstep_t0;
static uint16_t  s_sstep_n;
static uint16_t  s_sstep_t10;
static uint16_t  s_sstep_t90;
static uint16_t  s_sstep_settle;
static uint8_t   s_sstep_got10;
static uint8_t   s_sstep_got90;
static uint8_t   s_sstep_in_band;
static float     s_sstep_peak;
static float     s_sstep_tail_min;
static float     s_sstep_tail_max;
static float     s_sstep_tail_sum;
static uint16_t  s_sstep_tail_n;

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
static uint32_t      s_align_frames0;
static uint8_t       s_align_ol;      /* 1 = open-loop pre-position */   /* encoder frame count at start   */

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
    /* S10.1: shadow observer. Starts disabled; "obs on" arms it. */
    alg_obs_init(&s_obs, MOTOR_R_OHM, MOTOR_L_H, OBS_LPF_HZ,
                  OBS_PLL_WN_HZ, OBS_W_MAX_RADS);
    s_obs_on = 0u;

    /* S10.4: open-loop I/f ramp (also configured on every start). */
    alg_if_init(&s_if, IF_ACC_HZ_S, IF_I_AMP_A, IF_F_MIN_HZ);
    alg_if_set_align_time(&s_if, IF_ALIGN_MS / 1000.0f);

    /* S10.2: calibration measured on hardware, see board_cfg.h. */
    s_obs.comp      = OBS_COMP_MODE;
    s_obs.delay_s   = OBS_DELAY_US * 0.000001f;
    s_obs.theta_off = OBS_TRIM_DEG * 0.0174532925199433f;
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
    s_t_1k           = s_t_1ms;
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

    s_prot.oc_a          = FOC_I_TRIP_A;
    s_prot.overspeed_rpm = PROT_OVERSPEED_RPM;
    s_prot.overspeed_ms  = PROT_OVERSPEED_MS;
    s_prot.stall_rpm     = PROT_STALL_RPM;
    s_prot.stall_iq_a    = PROT_STALL_IQ_A;
    s_prot.stall_ms      = PROT_STALL_MS;
    s_prot.uv_v          = VBUS_UV_V;
    s_prot.ov_v          = VBUS_OV_V;
    s_prot.vbus_ms       = PROT_VBUS_MS;

    s_fault.code      = (uint8_t)APP_FAULT_NONE;
    s_fault.value     = 0.0f;
    s_fault.uptime_ms = 0u;
    { uint8_t i; for (i = 0u; i < (uint8_t)APP_FAULT_COUNT; i++) {
        s_fault.count[i] = 0u; } }

    bsp_vbus_init();

    s_spd_ramp_hz = SPD_RAMP_RPM_S;
    alg_spd_init(&s_spd, SPD_KP, SPD_KI, FOC_I_MAX_A,
                 SPD_FF_BASE, SPD_FF_SLOPE, SPD_KICK, SPD_KICK_MS);
    /* Must come AFTER init: init resets gain_hi_scale to 1.0 (no
     * scheduling), which silently disabled the feature earlier. */
    alg_spd_set_sched(&s_spd, SPD_GAIN_LO_RPM, SPD_GAIN_HI_RPM, SPD_GAIN_HI_SCALE);
    alg_spd_set_lpf(&s_spd, SPD_LPF_LO_RPM, SPD_LPF_HI_RPM, SPD_LPF_LO_K, SPD_LPF_HI_K);
    s_spd.ff_acc = SPD_FF_ACC;
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
    /* ---- S10.0c: open-loop pre-position (no encoder involved) ----
     * Hold theta = 0 while the magnitude ramps: the rotor is pulled onto
     * that axis and stays there. Nothing to measure, so this runs without
     * any encoder frame and without the sweep. ---- */
    if (s_align_ol != 0u) {
        if (el < (uint32_t)ALIGN_OL_MS) {
            m = ALIGN_M_START +
                ((ALIGN_M_MAX - ALIGN_M_START) * (float)el / (float)ALIGN_OL_MS);

            alg_svpwm(0.0f, m, duty);
            for (i = 0u; i < 3u; i++) {
                if (duty[i] < 0.0f) { duty[i] = 0.0f; }
                if (duty[i] > 1.0f) { duty[i] = 1.0f; }
            }
            bsp_pwm_set_duty(duty[0], duty[1], duty[2]);
            s_align.m = m;
            return;
        }

        /* The rotor now sits on the theta = 0 axis, which is all an
         * encoder-less drive needs: the forced frame is in phase by
         * construction.
         *
         * Deliberately NOT writing the encoder zero here: measured on
         * hardware (#128) the aligned position is ~5 deg electrical short
         * of the vector (cogging/friction equilibrium), while the S5 sweep
         * zero is accurate to +-5.3 deg. Storing the aligned position moved
         * zero_deg 131.91 -> 234.01 and would have made the encoder path
         * worse. Report the offset instead - the I/f hand-over needs it. */
        s_align_ol = 0u;
        if (mid_enc_info()->valid != 0u) {
            float off = mid_enc_info()->elec_deg;

            log_printf("OK align ol done, forced frame offset %.1f deg (elec)",
                       (double)off);
        } else {
            log_printf("OK align ol done, rotor at theta=0 (no encoder)");
        }
        align_finish(0u);
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
    /* Order matters: starting an alignment already moves the state away
     * from RUN, so checking the state first would make "already aligning"
     * (3) unreachable. */
    if (s_align.active != 0u) {
        return 3u;
    }
    if (s_st.state != APP_ST_RUN) {
        return 2u;                       /* start the bridge first */
    }

    /* The vector itself is written by task_align every frame (it ramps), so
     * nothing to do here beyond arming the state. */

    s_align_ol         = 0u;
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

uint8_t app_align_ol_start(void)
{
    /* Order matters: starting an alignment already moves the state away
     * from RUN, so checking the state first would make "already aligning"
     * (3) unreachable. */
    if (s_align.active != 0u) {
        return 3u;
    }
    if (s_st.state != APP_ST_RUN) {
        return 2u;                       /* start the bridge first */
    }

    /* Arm the same state as the sweep alignment, plus the open-loop flag:
     * task_align then holds theta = 0 and skips the sweep and every
     * encoder check (S10.0c). */
    s_align_ol         = 1u;
    s_align.active     = 1u;
    s_align.done       = 0u;
    s_align.fail       = 0u;
    s_align.zero_deg   = mid_enc_info()->zero_deg;
    s_align.err_deg    = 0.0f;
    s_align.n          = 0u;
    s_align.m          = 0.0f;
    s_align_t0         = HAL_GetTick();
    s_align_frames0    = mid_enc_info()->frames;

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

    /* S7.6: protections ignore the first milliseconds. */
    s_prot_start_ms = HAL_GetTick();
    s_prot_enc_cnt  = 0u;
    s_prot_enc_zero_cnt = 0u;
    s_prot_bad_seen = mid_enc_info()->frames;
    if (mid_current_get()->ready == 0u) {
        mid_current_restart_cal();
    }
    s_st.state = APP_ST_RUN;
    return 0u;
}

uint8_t app_stop(void)
{
    /* S10: a free-running PLL must not survive the transition. */
    mid_enc_pll_clear();

    alg_vf_stop(&s_vf);                  /* drop the V/f ramp first */
    alg_foc_reset(&s_foc);
    app_spd_off();
    s_step_phase       = 0u;             /* cancel any running step */
    s_sstep_phase      = 0u;
    /* Acknowledge the latched fault, keep the per-type counters. */
    s_fault.code       = (uint8_t)APP_FAULT_NONE;
    s_fault.value      = 0.0f;
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
void app_fault_raise(uint8_t code, float value)
{
    /* Verify the bridge really goes quiet before anything else. */
    bsp_pwm_disable();
    bsp_pwm_set_all_duty(PWM_START_DUTY);

    app_spd_off();
    s_foc_info.id_ref  = 0.0f;
    s_foc_info.iq_ref  = 0.0f;
    s_step_phase       = 0u;
    alg_foc_reset(&s_foc);

    /* S10: the bridge is about to go quiet, so the speed estimate must not
     * keep free-running (see mid_enc_pll_clear). */
    mid_enc_pll_clear();

    s_fault.code      = code;
    s_fault.value     = value;
    s_fault.uptime_ms = HAL_GetTick();
    if (code < (uint8_t)APP_FAULT_COUNT) {
        s_fault.count[code]++;
    }

    s_st.state = APP_ST_FAULT;

    switch (code) {
    case (uint8_t)APP_FAULT_OVERCURRENT:
        log_printf("ERR fault overcurrent I=%d mA", (int)(value * 1000.0f));
        break;
    case (uint8_t)APP_FAULT_OVERSPEED:
        log_printf("ERR fault overspeed %d rpm", (int)value);
        break;
    case (uint8_t)APP_FAULT_STALL:
        log_printf("ERR fault stall, iq %d mA at %d rpm",
                   (int)(value * 1000.0f), (int)mid_enc_info()->spd_rpm);
        break;
    case (uint8_t)APP_FAULT_UNDERVOLT:
        log_printf("ERR fault undervoltage %d mV", (int)(value * 1000.0f));
        break;
    case (uint8_t)APP_FAULT_OVERVOLT:
        log_printf("ERR fault overvoltage %d mV", (int)(value * 1000.0f));
        break;
    default:
        log_printf("ERR fault encoder invalid");
        break;
    }
    log_printf(" type stop to reset");
}

const app_fault_info_t *app_fault_info(void)
{
    return &s_fault;
}

const app_prot_t *app_prot_info(void)
{
    return &s_prot;
}

uint8_t app_prot_set(uint8_t which, float value)
{
    switch (which) {
    case 0u: if ((value < 100.0f)  || (value > 6000.0f)) { return 1u; }
             s_prot.overspeed_rpm = value; break;
    case 1u: if ((value < 1.0f)    || (value > 1000.0f)) { return 1u; }
             s_prot.stall_rpm = value; break;
    case 2u: if ((value < 0.05f)   || (value > 1.2f)) { return 1u; }
             s_prot.stall_iq_a = value; break;
    case 3u: if ((value < 5.0f)    || (value > 20.0f)) { return 1u; }
             s_prot.uv_v = value; break;
    case 4u: if ((value < 5.0f)    || (value > 24.0f)) { return 1u; }
             s_prot.ov_v = value; break;
    case 5u: /* the shunt saturates at +-1.32 A, so the trip must stay below
              * that; the lower bound keeps it above the noise floor. */
             if ((value < 0.20f) || (value > 1.30f)) { return 1u; }
             s_prot.oc_a = value; break;
    default: return 1u;
    }
    return 0u;
}

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

    /* The reference itself is ramped in the 1 kHz task; the kick window
     * starts now so the rotor breaks free before the ramp asks for speed. */
    s_spd_target   = rpm;
    s_spd.on       = 1u;
    s_spd.kick_on  = 1u;
    s_spd_kick_t0  = HAL_GetTick();
    s_spd_div     = (uint8_t)SPD_DIV;   /* run a step on the next frame */
    alg_pi_reset(&s_spd.pi);
    s_foc_info.spd_ref    = rpm;   /* display only; the loop uses the ramp */
    s_foc_info.spd_target = rpm;
    return 0u;
}

void app_spd_off(void)
{
    s_spd.on       = 0u;
    s_spd.spd_ref  = 0.0f;
    s_spd_target   = 0.0f;
    s_spd.out     = 0.0f;
    alg_pi_reset(&s_spd.pi);
    s_foc_info.spd_ref = 0.0f;
    s_foc_info.spd_target = 0.0f;
    s_foc_info.id_ref  = 0.0f;
    s_foc_info.iq_ref  = 0.0f;
}

uint8_t app_spd_set_ff(float base_a, float slope_a_per_rpm)
{
    if ((base_a < 0.0f) || (base_a > 0.5f)) {
        return 1u;
    }
    if ((slope_a_per_rpm < 0.0f) || (slope_a_per_rpm > 5e-4f)) {
        return 1u;
    }
    s_spd.ff_base  = base_a;
    s_spd.ff_slope = slope_a_per_rpm;
    return 0u;
}

void app_spd_get_ff(float *base_a, float *slope_a_per_rpm)
{
    if (base_a != 0) {
        *base_a = s_spd.ff_base;
    }
    if (slope_a_per_rpm != 0) {
        *slope_a_per_rpm = s_spd.ff_slope;
    }
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

void app_rip_start(void)
{
    s_rip.iq_min  =  1e9f;
    s_rip.iq_max  = -1e9f;
    s_rip.spd_min =  1e9f;
    s_rip.spd_max = -1e9f;
    s_rip.n       = 0u;
    s_rip_t0      = HAL_GetTick();
    s_rip_on      = 1u;
}

const app_rip_t *app_rip_info(void)
{
    return &s_rip;
}

uint8_t app_sstep_start(float rpm)
{
    if (s_st.state != APP_ST_RUN) { return 2u; }
    if (s_st.mode != CTRL_FOC)    { return 4u; }
    if ((rpm > SPD_RPM_MAX) || (rpm < -SPD_RPM_MAX)) { return 5u; }
    if ((rpm < 50.0f) && (rpm > -50.0f)) { return 5u; }

    s_sstep.target = rpm;
    s_sstep.valid  = 0u;
    s_sstep_phase  = 1u;
    s_sstep_t0     = HAL_GetTick();
    app_spd_off();
    return 0u;
}

const app_sstep_info_t *app_sstep_info(void)
{
    return &s_sstep;
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

    /* No encoder frame yet: hold the zero vector rather than invent an angle.
     * S10.4: unless I/f is driving, in which case the angle is ours. */
    if ((e->valid == 0u) && (s_if.run == 0u)) {
        bsp_pwm_set_all_duty(PWM_START_DUTY);
        return;
    }

    /* ---- encoder liveness: count frames without a NEW good one.
     * Using the bad-frame counter was wrong: while the encoder is healthy
     * that counter never changes, so the "stuck" timer fired 20 frames
     * after start (measured: false encoder fault at 1 ms). ---- */
    if (s_if.run == 0u) {
        if (e->frames != s_prot_bad_seen) {
            s_prot_bad_seen = e->frames;
            s_prot_enc_cnt  = 0u;
        } else if (s_prot_enc_cnt < 0xFFFFu) {
            s_prot_enc_cnt++;
            if (s_prot_enc_cnt >= PROT_ENC_FRAMES) {
                app_fault_raise((uint8_t)APP_FAULT_ENCODER, 0.0f);
                return;
            }
        }

        /* A disconnected MT6701 drives MISO to a constant level, so every
         * bit reads 0 and the CRC of an all-zero frame is itself zero: the
         * CRC check passes and the angle looks like a valid 0. Measured on
         * hardware with the cable unplugged: raw 0, crc ok, no fault.
         * A real sensor sitting exactly at raw 0 for PROT_ENC_ZERO_FRAMES
         * frames in a row is far less likely than a dead bus. */
        if (e->raw == 0u) {
            if (s_prot_enc_zero_cnt < 0xFFFFu) {
                s_prot_enc_zero_cnt++;
            }
            if (s_prot_enc_zero_cnt >= (uint16_t)PROT_ENC_ZERO_FRAMES) {
                app_fault_raise((uint8_t)APP_FAULT_ENCODER, 0.0f);
                return;
            }
        } else {
            s_prot_enc_zero_cnt = 0u;
        }
    }

    /* ---- current guard (S7 will replace this with a hardware trip) ---- */
    imax = fabsf(cs->ia);
    t = fabsf(cs->ib); if (t > imax) { imax = t; }
    t = fabsf(cs->ic); if (t > imax) { imax = t; }

    if (imax >= s_prot.oc_a) {
        if (++s_foc_trip_cnt >= FOC_TRIP_FRAMES) {
            s_foc_trip_cnt = 0u;
            s_foc_info.faults++;
            app_fault_raise((uint8_t)APP_FAULT_OVERCURRENT, imax);
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
            float spd_fb;

            /* S10.5b: in sensorless mode the observer's speed is the feedback;
             * the encoder is only there to verify it. */
            spd_fb = (s_sfoc != 0u)
                   ? (s_obs.omega_e * 9.54929658551372f / (float)ENC_POLE_PAIRS)
                   : e->spd_rpm;

            (void)alg_spd_step(&s_spd, spd_fb, SPD_TS_S);
            s_foc_info.iq_ref = s_spd.out;
            s_foc_info.id_ref = 0.0f;
            s_foc_info.spd_rpm = spd_fb;
        }
        if (s_sfoc == 0u) {
            s_foc_info.spd_rpm = e->spd_rpm;
        }
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

    /* ---- S10.4: open-loop I/f ----
     * The forced frame supplies both the angle and the current vector; the
     * speed loop must stay out of the way (no meaningful feedback yet). */
    if (s_if.run != 0u) {
        alg_if_step(&s_if, FOC_TS);

        /* ---- S10.5: hand-over evaluation ----
         * Only once the drag phase is running (the rotor is actually turning). */
        if ((s_ho_state == 1u) && (s_if.phase == 1u)) {
            float de = s_obs.theta_out - s_if.theta;

            float rpm_if;

            while (de >  3.14159265f) { de -= 6.283185307179586f; }
            while (de < -3.14159265f) { de += 6.283185307179586f; }
            s_ho_dth = (de > 0.0f) ? de : -de;     /* diagnostics only */

            /* Judge the observer, not its agreement with the forced frame:
             * the drag always runs at a load angle (measured ~74 deg), so the
             * two angles are never close. */
            rpm_if = s_if.f_hz * (60.0f / (float)ENC_POLE_PAIRS);

            if ((fabsf(s_obs.eq) > OBS_E_MIN_V) &&
                (fabsf(rpm_if) > HO_MIN_RPM) &&
                (s_obs.pll.locked != 0u)) {
                if (s_ho_cnt < 0xFFFFu) {
                    s_ho_cnt++;
                }
                if (s_ho_cnt >= (uint16_t)HO_HOLD_CNT) {
                    s_ho_state = 2u;
                    s_ho_w     = 0.0f;
                }
            } else {
                s_ho_cnt = 0u;
            }
        } else if (s_ho_state == 2u) {
            s_ho_w += FOC_TS / HO_BLEND_S;
            if (s_ho_w >= 1.0f) {
                float rpm_tgt;

                s_ho_w     = 1.0f;
                s_ho_state = 3u;

                /* S10.5b: from here the observer owns both angle and speed.
                 * Start the speed loop at the frequency the drag was running,
                 * then drop the forced ramp so nothing fights it. */
                rpm_tgt = s_if.f_hz * (60.0f / (float)ENC_POLE_PAIRS);
                s_sfoc  = 1u;
                alg_if_stop(&s_if);
                app_spd_set(rpm_tgt);
            }
        }
        /* Torque, not alignment: the current must sit on the q axis of the
         * forced frame. A d-axis vector only pulls the rotor INTO the frame
         * (that is what align ol does) and produces no torque, so the rotor
         * would just sit there. Measured with id = i_amp on #138: frequency
         * ramped to 35 Hz, current 0.40 A, encoder 0 rpm. */
        if (s_if.phase == 0u) {
            /* Pre-align: d-axis, torque free (this is what "align ol" does). */
            s_foc_info.id_ref = s_if.i_now;
            s_foc_info.iq_ref = 0.0f;
            id_ref = s_if.i_now;
            iq_ref = 0.0f;
        } else {
            /* Drag: q axis, torque. */
            s_foc_info.id_ref = 0.0f;
            s_foc_info.iq_ref = s_if.i_now;
            id_ref = 0.0f;
            iq_ref = s_if.i_now;
        }
        s_foc_info.spd_rpm = s_if.f_hz * (60.0f / (float)ENC_POLE_PAIRS);
    }

    /* ---- S10.5: which angle drives the Park transform? ----
     *   not in I/f        -> encoder
     *   I/f, before blend -> forced frame
     *   I/f, blending     -> forced + w * (obs - forced)
     *                        (blend the DIFFERENCE: blending the wrapped
     *                         angles directly jumps at the 2*pi seam)
     *   I/f, after blend  -> observer */
    {
        float th_use;
        float we_use;

        if (s_sfoc != 0u) {
            /* True sensorless: the observer supplies the angle. */
            th_use = s_obs.theta_out;
            we_use = s_obs.omega_e;
        } else if (s_if.run == 0u) {
            th_use = e->elec_deg * 0.0174532925199433f;
            we_use = e->spd_hz * 6.283185307179586f;
        } else {
            float de = s_obs.theta_out - s_if.theta;

            while (de >  3.14159265f) { de -= 6.283185307179586f; }
            while (de < -3.14159265f) { de += 6.283185307179586f; }

            th_use = s_if.theta + (s_ho_w * de);
            while (th_use >= 6.283185307179586f) { th_use -= 6.283185307179586f; }
            while (th_use <  0.0f)                { th_use += 6.283185307179586f; }

            we_use = (6.283185307179586f * s_if.f_hz * (1.0f - s_ho_w)) +
                     (s_obs.omega_e * s_ho_w);
        }

        /* S7.2: electrical angular speed for the feedforward, signed. */
        (void)alg_foc_step(&s_foc, cs->ia, cs->ib, cs->ic,
                           th_use, we_use,
                           id_ref, iq_ref, FOC_TS, duty);
    }

    /* ---- S10.1: shadow observer (measures only, never drives) ---- */
    if (s_obs_on != 0u) {
        alg_obs_step(&s_obs, s_foc.v_alpha, s_foc.v_beta,
                     s_foc.i_alpha, s_foc.i_beta, FOC_TS);
    }

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

    /* Also reset the encoder error counters: "clr" is documented as
     * "clear counters", and without this the CRC count could not be
     * measured over a window (it kept the historical total). */
    mid_enc_clear_errs();
}

/* ------------------------------------------------------------------ */
/* S10.1 back-EMF observer (shadow mode)                              */
/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/* S10.4 open-loop I/f                                                */
/* ------------------------------------------------------------------ */
uint8_t app_if_start(float rpm)
{
    float f_hz;

    if (s_st.state != APP_ST_RUN) {
        return 2u;
    }
    if (rpm >  IF_RPM_MAX) { rpm =  IF_RPM_MAX; }
    if (rpm < -IF_RPM_MAX) { rpm = -IF_RPM_MAX; }

    /* Mechanical rpm -> electrical Hz. */
    f_hz = (rpm / 60.0f) * (float)ENC_POLE_PAIRS;

    app_spd_off();                  /* no speed loop during a forced run */

    if (s_if.run != 0u) {
        /* Already dragging: only move the target. Resetting theta and the
         * frequency here yanks the forced frame back to 0 and throws the
         * rotor out of sync at once (measured: 200 rpm -> 0 rpm). */
        alg_if_set_target(&s_if, f_hz);
    } else {
        /* Do NOT re-init here: that used to wipe the amplitude set by
         * "if i" back to the default, which made a parameter sweep look
         * like "the amplitude does not matter". */
        alg_if_start(&s_if);
        alg_if_set_target(&s_if, f_hz);
    }
    return 0u;
}

void app_if_stop(void)
{
    alg_if_stop(&s_if);
    s_foc_info.id_ref = 0.0f;
    s_foc_info.iq_ref = 0.0f;
}

void app_if_set_amp(float a)
{
    if ((a > 0.05f) && (a <= FOC_I_MAX_A)) {
        s_if.i_amp = a;
    }
}

void app_if_set_acc(float hz_s)
{
    if ((hz_s >= 2.0f) && (hz_s <= 300.0f)) {
        s_if.acc_hz_s = hz_s;
    }
}

/* ---- S10.5 hand-over ---- */
uint8_t app_sfoc_on(void)   { return s_sfoc; }

uint8_t app_ho_state(void) { return s_ho_state; }
float   app_ho_w(void)     { return s_ho_w; }
float   app_ho_dth(void)   { return s_ho_dth; }
void    app_ho_reset(void) { s_ho_state = 0u; s_ho_cnt = 0u; s_ho_w = 0.0f; }
void    app_ho_arm(void)
{
    s_sfoc     = 0u;          /* leave sensorless mode while re-arming */
    s_ho_state = 1u;
    s_ho_cnt   = 0u;
    s_ho_w     = 0.0f;
}

uint8_t app_if_on(void)      { return s_if.run; }
float   app_if_theta(void)   { return s_if.theta; }
float   app_if_freq_hz(void) { return s_if.f_hz; }
float   app_if_amp(void)     { return s_if.i_now; }

uint8_t app_obs_set_on(uint8_t on)
{
    s_obs_on = (on != 0u) ? 1u : 0u;
    if (s_obs_on != 0u) {
        /* Start from the encoder angle when it is valid, otherwise 0:
         * the PLL converges either way, this only avoids a long swing. */
        float th0 = (mid_enc_info()->valid != 0u)
                  ? (mid_enc_info()->elec_deg * 0.0174532925199433f) : 0.0f;

        alg_obs_reset(&s_obs, th0);
    }
    return s_obs_on;
}

uint8_t app_obs_get_on(void)
{
    return s_obs_on;
}

uint8_t app_obs_set_bw(float wn_hz)
{
    if ((wn_hz < 5.0f) || (wn_hz > 500.0f)) {
        return 1u;
    }
    alg_pll_set_bw(&s_obs.pll, wn_hz, 0.707f);
    return 0u;
}

uint8_t app_obs_set_lpf(float hz)
{
    if ((hz < 0.0f) || (hz > 8000.0f)) {
        return 1u;
    }
    s_obs.lpf_hz = hz;
    s_obs.lpf_k  = (hz > 0.0f) ? (1.0f / (6.283185307179586f * hz)) : 0.0f;
    return 0u;
}

void app_obs_set_comp(uint8_t mode)
{
    s_obs.comp = (mode <= 2u) ? mode : 0u;
}

void app_obs_set_trim(float deg)
{
    s_obs.theta_off = deg * 0.0174532925199433f;
}

void app_obs_set_delay(float us)
{
    if ((us >= 0.0f) && (us <= 500.0f)) {
        s_obs.delay_s = us * 0.000001f;
    }
}

void app_obs_set_rl(float r_ohm, float l_h)
{
    if (r_ohm > 0.0f) { s_obs.r_ohm = r_ohm; }
    if (l_h   > 0.0f) { s_obs.l_h   = l_h;   }
}

void app_obs_reset(float theta)
{
    alg_obs_reset(&s_obs, theta);
}

const void *app_obs_info(void)
{
    return (const void *)&s_obs;
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
/* S7.4: speed step measurement, 1 kHz. Mirrors the current-loop "step". */
static void sstep_task(uint32_t now)
{
    float spd = mid_enc_info()->spd_rpm;
    float tgt = s_sstep.target;
    uint32_t el;
    float band;

    if (s_sstep_phase == 0u) {
        return;
    }

    if (s_sstep_phase == 1u) {          /* settle at zero speed */
        uint32_t st = (uint32_t)(now - s_sstep_t0);

        app_spd_off();
        /* Wait until the rotor has really stopped: only checking the
         * elapsed time left the previous run coasting, and a step that
         * starts from 1000 rpm looks like 67% overshoot. */
        if (((st >= (uint32_t)SSTEP_SETTLE_MS) &&
             (mid_enc_info()->spd_rpm < 20.0f) &&
             (mid_enc_info()->spd_rpm > -20.0f)) ||
            (st >= (uint32_t)SSTEP_SETTLE_MAX_MS)) {
            s_sstep_phase  = 2u;
            s_sstep_t0     = now;
            s_sstep_n      = 0u;
            s_sstep_t10    = 0u;
            s_sstep_t90    = 0u;
            s_sstep_settle = 0u;
            s_sstep_got10  = 0u;
            s_sstep_got90  = 0u;
            s_sstep_in_band = 0u;
            s_sstep_peak   = 0.0f;
            s_sstep_tail_min =  1e9f;
            s_sstep_tail_max = -1e9f;
            s_sstep_tail_sum = 0.0f;
            s_sstep_tail_n   = 0u;
            (void)app_spd_set(tgt);        /* the actual step */
        }
    } else if (s_sstep_phase == 2u) {   /* record the response */
        el = (uint32_t)(now - s_sstep_t0);
        s_sstep_n++;

        if (s_sstep_n > 2u) {
            float sgn = (tgt > 0.0f) ? 1.0f : -1.0f;
            float y   = spd * sgn;         /* signed towards the target */
            float a   = tgt * sgn;

            if ((s_sstep_got10 == 0u) && (y >= (0.1f * a))) {
                s_sstep_got10 = 1u;
                s_sstep_t10   = s_sstep_n;
            }
            if ((s_sstep_got90 == 0u) && (y >= (0.9f * a))) {
                s_sstep_got90 = 1u;
                s_sstep_t90   = s_sstep_n;
            }
            if (y > s_sstep_peak) { s_sstep_peak = y; }

            /* settle: inside +-5% of the target and staying there */
            band = 0.05f * a;
            if ((y > (a - band)) && (y < (a + band))) {
                if (s_sstep_in_band == 0u) {
                    s_sstep_in_band = 1u;
                    s_sstep_settle  = s_sstep_n;
                }
            } else {
                s_sstep_in_band = 0u;
                s_sstep_settle  = 0u;
            }

            /* tail window: last 100 ms */
            if ((uint32_t)el >= ((uint32_t)SSTEP_RECORD_MS - 100u)) {
                if (y < s_sstep_tail_min) { s_sstep_tail_min = y; }
                if (y > s_sstep_tail_max) { s_sstep_tail_max = y; }
                s_sstep_tail_sum += y;
                s_sstep_tail_n++;
            }
        }

        if (el >= (uint32_t)SSTEP_RECORD_MS) {
            float a = (tgt > 0.0f) ? tgt : -tgt;

            s_sstep_phase = 0u;
            s_sstep.valid = 1u;
            s_sstep.rise_ms = -1.0f;
            if ((s_sstep_got10 != 0u) && (s_sstep_got90 != 0u) &&
                (s_sstep_t90 >= s_sstep_t10)) {
                s_sstep.rise_ms = (float)(s_sstep_t90 - s_sstep_t10);
            }
            /* settle time from the step to entering the band, 1 kHz */
            s_sstep.settle_ms = (float)s_sstep_settle;

            s_sstep.overshoot = 0.0f;
            if (a > 5.0f) {
                s_sstep.overshoot = ((s_sstep_peak - a) / a) * 100.0f;
                if (s_sstep.overshoot < 0.0f) { s_sstep.overshoot = 0.0f; }
            }

            s_sstep.err  = 0.0f;
            s_sstep.ss   = 0.0f;
            s_sstep.ripple = 0.0f;
            if (s_sstep_tail_n != 0u) {
                float avg = s_sstep_tail_sum / (float)s_sstep_tail_n;
                s_sstep.ss = avg;
                if (a > 5.0f) {
                    s_sstep.err = ((avg - a) / a) * 100.0f;
                    if (s_sstep.err < 0.0f) { s_sstep.err = -s_sstep.err; }
                }
                s_sstep.ripple = s_sstep_tail_max - s_sstep_tail_min;
            }
            s_sstep.peak = s_sstep_peak;

            app_spd_off();
            log_printf("OK sstep %d rpm, rise %d ms, over %.1f%%, err %.1f%%",
                       (int)tgt, (int)s_sstep.rise_ms,
                       (double)s_sstep.overshoot, (double)s_sstep.err);
            log_printf(" settle %d ms, ss %d rpm, peak %d rpm, rip %d rpm",
                       (int)s_sstep.settle_ms, (int)s_sstep.ss,
                       (int)s_sstep.peak, (int)s_sstep.ripple);
        }
    }
}

/* S7.6: speed and bus-voltage protections, evaluated once per millisecond. */
static void prot_check_1ms(uint32_t now)
{
    const mid_enc_t *e = mid_enc_info();
    float spd;
    float vbus;

    if (s_st.state != APP_ST_RUN) {
        s_prot_ov_ms = 0u;
        s_prot_stall_ms = 0u;
        s_prot_uv_ms = 0u;
        s_prot_ovv_ms = 0u;
        return;
    }

    /* Grace window: the first milliseconds after start are not representative
     * (the bus sags while the gate drivers come up, the speed estimate is
     * still empty). */
    if ((uint32_t)(now - s_prot_start_ms) < (uint32_t)PROT_GRACE_MS) {
        s_prot_ov_ms = 0u;
        s_prot_stall_ms = 0u;
        s_prot_uv_ms = 0u;
        s_prot_ovv_ms = 0u;
        return;
    }

    spd  = e->spd_rpm;
    vbus = bsp_vbus_volts();

    /* ---- over-speed: any mode ---- */
    if ((spd > s_prot.overspeed_rpm) || (spd < -s_prot.overspeed_rpm)) {
        if (s_prot_ov_ms == 0u) {
            s_prot_ov_ms = now;
        } else if ((uint32_t)(now - s_prot_ov_ms) >= (uint32_t)s_prot.overspeed_ms) {
            app_fault_raise((uint8_t)APP_FAULT_OVERSPEED, spd);
            return;
        }
    } else {
        s_prot_ov_ms = 0u;
    }

    /* ---- stall: only meaningful while the speed loop owns the torque. In
     * torque mode holding the shaft still is a legitimate experiment. ---- */
    if ((s_spd.on != 0u) &&
        (spd < s_prot.stall_rpm) && (spd > -s_prot.stall_rpm) &&
        ((s_foc_info.iq_ref > s_prot.stall_iq_a) ||
         (s_foc_info.iq_ref < -s_prot.stall_iq_a))) {
        if (s_prot_stall_ms == 0u) {
            s_prot_stall_ms = now;
        } else if ((uint32_t)(now - s_prot_stall_ms) >= (uint32_t)s_prot.stall_ms) {
            app_fault_raise((uint8_t)APP_FAULT_STALL, s_foc_info.iq_ref);
            return;
        }
    } else {
        s_prot_stall_ms = 0u;
    }

    /* ---- bus voltage ---- */
    if (vbus < s_prot.uv_v) {
        if (s_prot_uv_ms == 0u) {
            s_prot_uv_ms = now;
        } else if ((uint32_t)(now - s_prot_uv_ms) >= (uint32_t)s_prot.vbus_ms) {
            app_fault_raise((uint8_t)APP_FAULT_UNDERVOLT, vbus);
            return;
        }
    } else {
        s_prot_uv_ms = 0u;
    }

    if (vbus > s_prot.ov_v) {
        if (s_prot_ovv_ms == 0u) {
            s_prot_ovv_ms = now;
        } else if ((uint32_t)(now - s_prot_ovv_ms) >= (uint32_t)s_prot.vbus_ms) {
            app_fault_raise((uint8_t)APP_FAULT_OVERVOLT, vbus);
            return;
        }
    } else {
        s_prot_ovv_ms = 0u;
    }
}

static void task_1ms(uint32_t now)
{
    uint32_t dt = (uint32_t)(now - s_t_1ms);
    uint32_t d1 = (uint32_t)(now - s_t_1k);

    /* ---- true 1 kHz work -------------------------------------------
     * NOTE: the dt >= 1000 block below is the once-per-SECOND stats
     * block, not a 1 ms one. Putting this work there made sstep advance
     * once per second (metrics empty) and made the bus-voltage filter
     * run at the loop rate. A dedicated timestamp is the simple fix. */
    if (d1 >= 1u) {
        s_t_1k = now;

        if (bsp_vbus_update() != 0u) {
            alg_foc_set_vdc(&s_foc, bsp_vbus_volts());
        }

        /* Ripple diagnostic, sampled at 1 kHz. */
        if (s_rip_on != 0u) {
            if ((uint32_t)(now - s_rip_t0) < (uint32_t)RIP_WINDOW_MS) {
                float iq = s_foc_info.iq;
                float sp = s_foc_info.spd_rpm;

                if (iq < s_rip.iq_min)  { s_rip.iq_min  = iq; }
                if (iq > s_rip.iq_max)  { s_rip.iq_max  = iq; }
                if (sp < s_rip.spd_min) { s_rip.spd_min = sp; }
                if (sp > s_rip.spd_max) { s_rip.spd_max = sp; }
                s_rip.n++;
            } else {
                s_rip_on = 0u;      /* done, keep the result */
            }
        }

        prot_check_1ms(now);
        sstep_task(now);

        /* S7.3: the start-up kick lasts SPD_KICK_MS after a target change only. */
        if ((s_spd.kick_on != 0u) &&
            ((uint32_t)(now - s_spd_kick_t0) >= (uint32_t)SPD_KICK_MS)) {
            s_spd.kick_on = 0u;
        }

        /* Acceleration limit: walk spd_ref towards the commanded target at
         * s_spd_ramp_hz rpm/s. 1 kHz, so one step is ramp/1000 rpm. */
        if (s_spd.on != 0u) {
            float step_rpm = s_spd_ramp_hz * 0.001f;
            float d = s_spd_target - s_spd.spd_ref;

            if (d > step_rpm) {
                s_spd.spd_ref += step_rpm;
            } else if (d < -step_rpm) {
                s_spd.spd_ref -= step_rpm;
            } else {
                s_spd.spd_ref = s_spd_target;
            }
        }
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
    ch[17] = mid_enc_win_rpm();          /* window estimator, rpm */
    ch[18] = mid_enc_pll_rpm();          /* PLL observer, rpm     */
    /* S10.1: back-EMF observer against the encoder. ch19 = estimated
     * electrical angle, ch20 = wrapped difference to the encoder (deg).
     * If ch20 stays small the observer is good enough to take over. */
    ch[19] = s_obs.theta_out * 57.29577951308232f;
    {
        float de = s_obs.theta_out - (mid_enc_info()->elec_deg * 0.0174532925199433f);

        while (de >  3.14159265f) { de -= 6.283185307179586f; }
        while (de < -3.14159265f) { de += 6.283185307179586f; }
        ch[20] = de * 57.29577951308232f;
    }

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
