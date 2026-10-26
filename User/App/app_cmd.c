/**
  ******************************************************************************
  * @file    app_cmd.c
  * @brief   Host command parsing and dispatch.
  *
  * All output is plain ASCII on purpose: serial terminals, VOFA+ and Keil
  * ARMCC V5 all handle ASCII identically. Chinese text lives in /docs only.
  ******************************************************************************
  */

#include "app_cmd.h"
#include "app_main.h"
#include "board_cfg.h"
#include "bsp_adc.h"
#include "bsp_cordic.h"
#include "bsp_gpio.h"
#include "bsp_prof.h"
#include "bsp_pwm.h"
#include "bsp_ssi.h"
#include "bsp_uart.h"
#include "mid_current.h"
#include "mid_enc.h"
#include "mid_log.h"
#include "mid_trig.h"
#include "mid_vofa.h"

#include <string.h>

#define CMD_LINE_MAX    64u

static char    s_line[CMD_LINE_MAX];
static uint8_t s_line_len;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
static uint8_t split_args(char *s, char **tok, uint8_t max_tok)
{
    uint8_t n = 0u;

    while ((*s != '\0') && (n < max_tok)) {
        while ((*s == ' ') || (*s == '\t')) { s++; }
        if (*s == '\0') { break; }
        tok[n++] = s;
        while ((*s != '\0') && (*s != ' ') && (*s != '\t')) { s++; }
        if (*s != '\0') { *s++ = '\0'; }
    }
    return n;
}

static uint8_t parse_u32(const char *s, uint32_t *out)
{
    uint32_t v = 0u;
    uint8_t  digits = 0u;

    if (s == NULL) { return 0u; }
    while ((*s >= '0') && (*s <= '9')) {
        v = v * 10u + (uint32_t)(*s - '0');
        s++;
        digits++;
        if (digits > 9u) { return 0u; }
    }
    if ((digits == 0u) || (*s != '\0')) { return 0u; }
    *out = v;
    return 1u;
}

/* Duty argument: "0.5", ".5", "1", "0.25". Values above 1 are accepted and
 * clamped by bsp_pwm (the applied value is echoed back). */
static uint8_t parse_duty(const char *s, float *out)
{
    float   v = 0.0f;
    float   scale = 0.1f;
    uint8_t digits = 0u;

    if (s == NULL) { return 0u; }
    while ((*s >= '0') && (*s <= '9')) {
        v = v * 10.0f + (float)(*s - '0');
        s++;
        digits++;
        /* 5 digits: the largest runtime value is ki = 50000. Duties and rates
         * stay far below that, so widening the limit changes nothing there. */
        if (digits > 5u) { return 0u; }
    }
    if (*s == '.') {
        s++;
        while ((*s >= '0') && (*s <= '9')) {
            v += (float)(*s - '0') * scale;
            scale *= 0.1f;
            s++;
            digits++;
            if (digits > 6u) { return 0u; }
        }
    }
    if ((digits == 0u) || (*s != '\0')) { return 0u; }
    *out = v;
    return 1u;
}

/* Signed variant for the FOC current references (the q axis goes negative
 * when the motor is braked or driven backwards). */
static uint8_t parse_num(const char *s, float *out)
{
    uint8_t neg = 0u;
    float   v;

    if ((s != 0) && (*s == (char)0x2D)) {   /* "-" */
        neg = 1u;
        s++;
    }
    if (!parse_duty(s, &v)) {
        return 0u;
    }
    *out = (neg != 0u) ? -v : v;
    return 1u;
}

/* ------------------------------------------------------------------ */
/* Information screens                                                 */
/* ------------------------------------------------------------------ */
static void print_help(void)
{
    log_printf("---- commands ----");
    log_printf(" help           show this list");
    log_printf(" ver            version and board info");
    log_printf(" stat           runtime status snapshot");
    log_printf(" start          start (select a mode first)");
    log_printf(" stop           stop and shut the bridge down");
    log_printf(" mode <0|1|2>   0=IDLE 1=OPENLOOP 2=FOC");
    log_printf(" pwm <d>        all three duties 0..1 (needs start)");
    log_printf(" pwm <a> <b> <c> three independent duties");
    log_printf(" cur            raw codes and phase currents");
    log_printf(" vf <hz>        open-loop V/f target, electrical Hz (needs start)");
    log_printf(" id <A>         FOC d-axis current reference (mode 2)");
    log_printf(" iq <A>         FOC q-axis current reference (mode 2)");
    log_printf(" foc            show the FOC currents and voltages");
    log_printf(" pi <kp> <ki>   FOC PI gains (runtime), pi def restores");
    log_printf(" step <A>       measure an iq step at 20 kHz, prints the result");
    log_printf(" vf off         stop modulating, back to the 50%% zero vector");
    log_printf(" acc <hz/s>     frequency ramp rate 1..200 Hz/s");
    log_printf(" dir <0|1>      phase sequence, 1 reverses the rotation");
    log_printf(" cal [erase]    zero-drift calibration (needs start), erase = wipe");
    log_printf(" prof [reset|on|off]  per-stage timing profile (DWT)");
    log_printf(" phase [reset]  where each step lands in the 50us PWM period");
    log_printf(" trig           benchmark the sin/cos back end (LUT or CORDIC)");
    log_printf(" cordic         dump raw CORDIC readback for 4 angles");
    log_printf(" enc [zero|dir|raw]  MT6701 angle, zero capture, direction");
    log_printf(" align          force-align the rotor and store the electrical zero");
    log_printf(" led <on|off|toggle>   drive the PF1 LED");
    log_printf(" wave <on|off>  VOFA+ waveform upload");
    log_printf(" rate <hz>      waveform rate 1..500 (FireWater)");
    log_printf(" clr            clear counters");
    log_printf(" echo <text>    loopback test");
    log_printf("------------------");
}

static void print_version(void)
{
    log_printf("board = %s", BOARD_NAME);
    log_printf("build = %s %s #%u", __DATE__, __TIME__, (unsigned)FW_BUILD_ID);
    log_printf("core  = %u Hz", (unsigned)SYS_CORE_CLK_HZ);
    log_printf("foc   = %u Hz, Ts = %u us",
               (unsigned)FOC_FREQ_HZ, (unsigned)FOC_TS_US);
    log_printf("pwm   = %u Hz, ARR = %u, DTG = %u",
               (unsigned)PWM_FREQ_HZ_CALC, (unsigned)PWM_ARR, (unsigned)PWM_DTG_REG);
    log_printf("cur   = Rs = %u mOhm, gain = %u, %u uA/LSB, range +-%u mA",
               (unsigned)(RSHUNT_OHM * 1000.0f + 0.5f), (unsigned)INA240_GAIN,
               (unsigned)(CUR_A_PER_LSB * 1000000.0f + 0.5f),
               (unsigned)(CUR_RANGE_A * 1000.0f + 0.5f));
    log_printf("vbus  = div = %u, %u uV/LSB, max = %u mV",
               (unsigned)(VBUS_DIV_RATIO + 0.5f),
               (unsigned)(VBUS_V_PER_LSB * 1000000.0f + 0.5f),
               (unsigned)(VBUS_MAX_MEASURABLE_V * 1000.0f + 0.5f));
    log_printf("uart  = %u baud (USART1 PB6/PB7), VOFA+ = FireWater",
               (unsigned)DBG_UART_BAUD);
}

/* Where the zero-drift offsets came from (see CUR_SRC_* in mid_current.h) */
static const char *cal_src_str(uint8_t src)
{
    if (src == CUR_SRC_FLASH) { return "flash"; }
    if (src == CUR_SRC_LIVE)  { return "live"; }
    return "none";
}

static void print_stat(void)
{
    const app_status_t *st = app_status();

    log_printf("---- status ----");
    log_printf(" uptime   = %u ms", (unsigned)st->uptime_ms);
    log_printf(" state    = %s", app_state_name(st->state));
    log_printf(" mode     = %s", app_mode_name(st->mode));
    {
        float a, b, c;
        bsp_pwm_get_duty(&a, &b, &c);
        log_printf(" pwm      = %s, dtg=%u, duty=%.2f/%.2f/%.2f",
                   bsp_pwm_is_enabled() ? "ON" : "OFF",
                   (unsigned)bsp_pwm_get_dtg(), a, b, c);
    }
    {
        const current_sample_t *cs = mid_current_get();
        log_printf(" adc      = frames=%u ovr=%u cal=%u sat=%u src=%s gen=%u st=%u",
                   (unsigned)bsp_adc_frames(), (unsigned)bsp_adc_ovr_count(),
                   (unsigned)cs->ready, (unsigned)cs->saturated,
                   cal_src_str(cs->src), (unsigned)cs->gen,
                   (unsigned)bsp_adc_status());
    }
    log_printf(" led      = %s", bsp_led_is_on() ? "ON" : "OFF");
    log_printf(" key      = down=%u total=%u",
               (unsigned)bsp_key_is_down(), (unsigned)st->key_cnt);
    {
        const app_vf_info_t *v = app_vf_info();
        log_printf(" vf       = %s %.1f/%.1f Hz, m=%.2f, I=%.2f A, dir=%u",
                   v->on ? "ON" : "OFF", v->f_hz, v->f_target, v->m,
                   v->i_max, (unsigned)v->dir);
    }
    log_printf(" loop     = %u kHz", (unsigned)st->loop_khz);
    log_printf(" wave     = %s @ %u Hz, frames=%u drop=%u",
               st->wave_on ? "ON" : "OFF",
               (unsigned)st->wave_rate_hz,
               (unsigned)st->wave_frames,
               (unsigned)st->wave_drop);
    log_printf(" uart     = rx_drop=%u tx_pending=%u tx_stuck=%u",
               (unsigned)bsp_uart_rx_drop_count(),
               (unsigned)bsp_uart_tx_pending(),
               (unsigned)bsp_uart_tx_stuck_count());
    log_printf("----------------");
}

/* ------------------------------------------------------------------ */
/* Dispatch                                                            */
/* ------------------------------------------------------------------ */
static void cmd_exec(char *line)
{
    char   *tok[4];
    uint8_t n;
    uint32_t val;

    n = split_args(line, tok, 4u);
    if (n == 0u) { return; }

    /* ---------------- help ---------------- */
    if (strcmp(tok[0], "help") == 0) {
        print_help();
    }
    /* ---------------- ver ----------------- */
    else if (strcmp(tok[0], "ver") == 0) {
        print_version();
    }
    /* ---------------- stat ---------------- */
    else if (strcmp(tok[0], "stat") == 0) {
        print_stat();
    }
    /* ---------------- start --------------- */
    else if (strcmp(tok[0], "start") == 0) {
        uint8_t r = app_start();
        if (r == 0u)      { log_printf("OK start, mode=%s", app_mode_name(app_status()->mode)); }
        else if (r == 1u) { log_printf("ERR state=FAULT, clear the fault first"); }
        else if (r == 3u) { log_printf("ERR start rejected"); }
        else              { log_printf("ERR mode=IDLE, send 'mode 1' or 'mode 2' first"); }
    }
    /* ---------------- stop ---------------- */
    else if (strcmp(tok[0], "stop") == 0) {
        (void)app_stop();
        log_printf("OK stop, PWM off");
    }
    /* ---------------- mode ---------------- */
    else if (strcmp(tok[0], "mode") == 0) {
        if (n < 2u)                        { log_printf("ERR usage=mode <0|1|2>"); }
        else if (!parse_u32(tok[1], &val)) { log_printf("ERR bad arg"); }
        else {
            uint8_t r = app_set_mode((uint8_t)val);
            if (r == 0u)      { log_printf("OK mode=%s", app_mode_name(app_status()->mode)); }
            else if (r == 2u) { log_printf("ERR stop the motor first"); }
            else              { log_printf("ERR mode must be 0/1/2"); }
        }
    }
    /* ---------------- pwm ----------------- */
    else if (strcmp(tok[0], "pwm") == 0) {
        float a, b, c;

        if ((n == 2u) || (n == 4u)) {
            if (app_status()->state != APP_ST_RUN) {
                log_printf("ERR start first");
            } else if (app_vf_info()->on != 0u) {
                log_printf("ERR vf active, send 'vf off' first");
            } else if (mid_current_get()->ready == 0u) {
                log_printf("ERR calibrating, wait");
            } else if (!parse_duty(tok[1], &a)) {
                log_printf("ERR bad duty, use 0..1");
            } else if ((n == 4u) && ((!parse_duty(tok[2], &b)) || (!parse_duty(tok[3], &c)))) {
                log_printf("ERR bad duty, use 0..1");
            } else {
                if (n == 2u) { b = a; c = a; }
                bsp_pwm_set_duty(a, b, c);
                bsp_pwm_get_duty(&a, &b, &c);       /* echo what was applied */
                log_printf("OK duty = %.2f/%.2f/%.2f", a, b, c);
            }
        } else if (n == 1u) {
            bsp_pwm_get_duty(&a, &b, &c);
            log_printf("pwm = %s, dtg=%u, duty=%.2f/%.2f/%.2f",
                       bsp_pwm_is_enabled() ? "ON" : "OFF",
                       (unsigned)bsp_pwm_get_dtg(), a, b, c);
        } else {
            log_printf("ERR usage=pwm <d> | pwm <a> <b> <c>");
        }
    }
    /* ---------------- id / iq (S6) ----------------- */
    else if ((strcmp(tok[0], "id") == 0) || (strcmp(tok[0], "iq") == 0)) {
        const app_foc_info_t *f = app_foc_info();

        if (n == 2u) {
            float amp;

            if (!parse_num(tok[1], &amp)) {
                log_printf("ERR usage=id|iq <A>, range -1.20 .. 1.20");
            } else {
                float id = f->id_ref;
                float iq = f->iq_ref;
                uint8_t rc;

                if (tok[0][1] == (char)0x64) {   /* "id": second char 'd' */
                    id = amp;
                } else {
                    iq = amp;
                }

                rc = app_foc_set(id, iq);
                switch (rc) {
                case 0u:
                    log_printf("OK id=%.2f A, iq=%.2f A", (double)id, (double)iq);
                    break;
                case 2u:
                    log_printf("ERR start first");
                    break;
                case 4u:
                    log_printf("ERR mode 2 first (FOC)");
                    break;
                case 5u:
                    log_printf("ERR current too high, |I| <= %.2f A",
                               (double)FOC_I_MAX_A);
                    break;
                default:
                    log_printf("ERR calibrating, wait");
                    break;
                }
            }
        } else {
            log_printf("id=%.2f iq=%.2f A, ref %.2f/%.2f, m=%.3f",
                       (double)f->id, (double)f->iq,
                       (double)f->id_ref, (double)f->iq_ref, (double)f->m);
        }
    }
    /* ---------------- pi (S6.4) ----------------- */
    else if (strcmp(tok[0], "pi") == 0) {
        float kp;
        float ki;

        app_foc_get_pi(&kp, &ki);

        if ((n == 2u) && (strcmp(tok[1], "def") == 0)) {
            switch (app_foc_set_pi(FOC_KP, FOC_KI)) {
            case 0u:
                log_printf("OK pi = %.2f/%.0f (default)", (double)FOC_KP, (double)FOC_KI);
                break;
            case 4u:
                log_printf("ERR mode 2 first (FOC)");
                break;
            default:
                log_printf("ERR pi rejected");
                break;
            }
        } else if (n == 3u) {
            float nkp;
            float nki;

            if (!parse_num(tok[1], &nkp) || !parse_num(tok[2], &nki)) {
                log_printf("ERR usage=pi <kp> <ki> | pi def | pi");
            } else {
                switch (app_foc_set_pi(nkp, nki)) {
                case 0u:
                    log_printf("OK pi = %.2f/%.0f", (double)nkp, (double)nki);
                    break;
                case 1u:
                    log_printf("ERR pi out of range, kp %.2f..%.2f",
                               (double)FOC_KP_MIN, (double)FOC_KP_MAX);
                    break;
                case 4u:
                    log_printf("ERR mode 2 first (FOC)");
                    break;
                default:
                    log_printf("ERR pi rejected");
                    break;
                }
            }
        } else {
            log_printf("pi = kp %.2f V/A, ki %.0f V/(A s)", (double)kp, (double)ki);
            log_printf(" min kp %.2f, max kp %.2f, ki %.0f..%.0f",
                       (double)FOC_KP_MIN, (double)FOC_KP_MAX,
                       (double)FOC_KI_MIN, (double)FOC_KI_MAX);
        }
    }
    /* ---------------- step (S6.4) ----------------- */
    else if (strcmp(tok[0], "step") == 0) {
        float amp;

        if (n != 2u) {
            log_printf("ERR usage=step <A>, e.g. step 0.30");
        } else if (!parse_num(tok[1], &amp)) {
            log_printf("ERR usage=step <A>, e.g. step 0.30");
        } else {
            switch (app_step_start(amp)) {
            case 0u:
                log_printf("OK step %.2f A, wait 0.3 s for the result", (double)amp);
                break;
            case 2u:
                log_printf("ERR start first");
                break;
            case 4u:
                log_printf("ERR mode 2 first (FOC)");
                break;
            default:
                log_printf("ERR current too high, |I| <= %.2f A",
                           (double)FOC_I_MAX_A);
                break;
            }
        }
    }

    /* ---------------- foc ----------------- */
    else if (strcmp(tok[0], "foc") == 0) {
        const app_foc_info_t *f = app_foc_info();
        if (f->ready == 0u) {
            log_printf("foc idle, run mode 2 + start + id/iq");
        } else {
            log_printf("foc id=%.3f iq=%.3f A, ref %.2f/%.2f",
                       (double)f->id, (double)f->iq,
                       (double)f->id_ref, (double)f->iq_ref);
            log_printf(" vd=%.2f vq=%.2f V, m=%.3f, faults %u",
                       (double)f->vd, (double)f->vq, (double)f->m,
                       (unsigned)f->faults);
        }
    }

    /* ---------------- vf ----------------- */
    else if (strcmp(tok[0], "vf") == 0) {
        if ((n >= 2u) && (strcmp(tok[1], "off") == 0)) {
            (void)app_vf_off();
            log_printf("OK vf off, duty 0.50");
        } else if (n == 2u) {
            float hz;

            if (!parse_duty(tok[1], &hz)) {
                log_printf("ERR usage=vf <hz> | vf off");
            } else if (hz <= 0.0f) {
                /* "vf 0" is the same as "vf off": a V/f drive at 0 Hz would
                 * only apply the low-speed boost as a DC vector, which
                 * heats the motor and holds the shaft without rotating. */
                (void)app_vf_off();
                log_printf("OK vf 0 = off, duty 0.50");
            } else {
                switch (app_vf_set(hz)) {
                case 0u:
                    log_printf("OK vf target = %.2f Hz", hz);
                    break;
                case 2u:
                    log_printf("ERR start first");
                    break;
                default:
                    log_printf("ERR calibrating, wait");
                    break;
                }
            }
        } else {
            const app_vf_info_t *v = app_vf_info();
            log_printf("vf = %s %.1f/%.1f Hz, m=%.2f, I=%.2f A, dir=%u",
                       v->on ? "ON" : "OFF", v->f_hz, v->f_target, v->m,
                       v->i_max, (unsigned)v->dir);
        }
    }
    /* ---------------- acc ----------------- */
    else if (strcmp(tok[0], "acc") == 0) {
        float a;

        if ((n != 2u) || (!parse_duty(tok[1], &a))) {
            log_printf("ERR usage=acc <hz/s> 1..200");
        } else {
            app_vf_set_acc(a);
            log_printf("OK acc = %.2f Hz/s", a);
        }
    }
    /* ---------------- dir ----------------- */
    else if (strcmp(tok[0], "dir") == 0) {
        float a;

        if ((n != 2u) || (!parse_duty(tok[1], &a))) {
            log_printf("ERR usage=dir <0|1>");
        } else if (app_vf_set_dir((uint8_t)(a + 0.5f)) != 0u) {
            log_printf("ERR dir must be 0 or 1");
        } else {
            log_printf("OK dir = %u", (unsigned)app_vf_info()->dir);
        }
    }
    /* ---------------- phase ----------------- */
    else if (strcmp(tok[0], "phase") == 0) {
        const app_phase_info_t *ph = app_phase_info();

        if ((n >= 2u) && (strcmp(tok[1], "reset") == 0)) {
            app_phase_reset();
            log_printf("OK phase reset");
        } else {
            /* isr: where in the 50us period the ADC interrupt fired (TIM1 CNT,
             * 1 count = 11.76ns, peak = PWM_ARR). The other three are stage
             * durations in tenths of a microsecond (170 cycles = 1us). */
            uint32_t p0 = (ph->pick.min_cyc * 10u) / PHASE_CYC_PER_US;
            uint32_t p1 = (ph->pick.max_cyc * 10u) / PHASE_CYC_PER_US;
            uint32_t c0 = (ph->comp.min_cyc * 10u) / PHASE_CYC_PER_US;
            uint32_t c1 = (ph->comp.max_cyc * 10u) / PHASE_CYC_PER_US;
            uint32_t w0 = (ph->wr.min_cyc   * 10u) / PHASE_CYC_PER_US;
            uint32_t w1 = (ph->wr.max_cyc   * 10u) / PHASE_CYC_PER_US;

            log_printf("---- phase, stage us (isr cnt = sample point) ----");
            log_printf(" isr cnt = %u..%u",
                       (unsigned)ph->isr.cnt_min, (unsigned)ph->isr.cnt_max);
            log_printf(" pick %u.%u..%u.%u  comp %u.%u..%u.%u  wr %u.%u..%u.%u",
                       (unsigned)(p0 / 10u), (unsigned)(p0 % 10u),
                       (unsigned)(p1 / 10u), (unsigned)(p1 % 10u),
                       (unsigned)(c0 / 10u), (unsigned)(c0 % 10u),
                       (unsigned)(c1 / 10u), (unsigned)(c1 % 10u),
                       (unsigned)(w0 / 10u), (unsigned)(w0 % 10u),
                       (unsigned)(w1 / 10u), (unsigned)(w1 % 10u));
        }
    }
    /* ---------------- trig ----------------- */
    else if (strcmp(tok[0], "trig") == 0) {
        const uint32_t n_iter = 1000u;
        uint32_t t0;
        uint32_t cyc;
        uint32_t per;
        uint32_t ns;
        float    th = 0.0f;
        float    c = 0.0f;
        float    s = 0.0f;
        uint32_t i;

        t0 = bsp_prof_now();
        for (i = 0u; i < n_iter; i++) {
            mid_trig_cos_sin(th, &c, &s);
            th += 0.01f;
        }
        cyc = bsp_prof_now() - t0;
        per = cyc / n_iter;
        ns  = (per * 100u) / 17u;              /* 170 cycles = 1us */

        log_printf("trig = %s, %u cycles/call, %u ns, cos1000=%d sin1000=%d",
                   mid_trig_name(), (unsigned)per, (unsigned)ns,
                   (int)(c * 1000.0f), (int)(s * 1000.0f));
    }
    /* ---------------- cordic ----------------- */
    else if (strcmp(tok[0], "cordic") == 0) {
        static const float  ang[4] = { 0.0f, 1.5707963f, 3.1415927f, -1.5707963f };
        static const char  *nm[4]  = { "0", "pi/2", "pi", "-pi/2" };
        uint8_t i;

        log_printf("---- cordic raw readback ----");
        for (i = 0u; i < 4u; i++) {
            int32_t c32 = 0;
            int32_t s32 = 0;
            float   c = 0.0f;
            float   s = 0.0f;

            bsp_cordic_cos_sin_raw(ang[i], &c, &s, &c32, &s32);
            log_printf(" th=%-5s c=%08x s=%08x", nm[i],
                       (unsigned)c32, (unsigned)s32);
        }
        log_printf("-----------------------------");
    }
    /* ---------------- align ----------------- */
    else if (strcmp(tok[0], "align") == 0) {
        switch (app_align_start()) {
        case 0u:
            log_printf("OK align started, hold theta=0");
            break;
        case 2u:
            log_printf("ERR start first");
            break;
        default:
            log_printf("ERR already aligning");
            break;
        }
    }
    /* ---------------- enc ----------------- */
    else if (strcmp(tok[0], "enc") == 0) {
        const mid_enc_t *e = mid_enc_info();

        if ((n >= 2u) && (strcmp(tok[1], "zero") == 0)) {
            if (mid_enc_zero() == 0u) {
                log_printf("OK enc zero, off %.2f deg (saved)",
                           (double)mid_enc_info()->zero_deg);
            } else {
                log_printf("ERR enc read failed");
            }
        } else if ((n >= 3u) && (strcmp(tok[1], "dir") == 0)) {
            uint32_t dv = 0u;

            if ((parse_u32(tok[2], &dv) == 0u) || (dv > 1u)) {
                log_printf("ERR usage=enc dir <0|1>");
            } else if (mid_enc_set_dir((uint8_t)dv) == 0u) {
                log_printf("OK enc dir = %u", (unsigned)dv);
            } else {
                log_printf("ERR enc dir");
            }
        } else if ((n >= 2u) && (strcmp(tok[1], "bench") == 0)) {
            uint8_t  f[3];
            uint32_t k;
            uint32_t t0;
            uint32_t cyc;
            uint32_t cfg1 = 0u;
            uint32_t ccipr2 = 0u;

            /* SPI only: no decode, no speed estimator */
            t0 = bsp_prof_now();
            for (k = 0u; k < 200u; k++) {
                (void)bsp_ssi_read(f);
            }
            cyc = (bsp_prof_now() - t0) / 200u;
            bsp_ssi_diag(&cfg1, &ccipr2);

            log_printf("enc bench = %u cycles/read, %u ns",
                       (unsigned)cyc, (unsigned)((cyc * 100u) / 17u));
            log_printf(" spi cr1=%08x ccipr2=%08x",
                       (unsigned)cfg1, (unsigned)ccipr2);
        } else if ((n >= 2u) && (strcmp(tok[1], "raw") == 0)) {
            uint8_t k;

            for (k = 0u; k < 5u; k++) {
                uint8_t f[3];

                if (bsp_ssi_read(f) == 0u) {
                    log_printf(" enc raw %02x %02x %02x",
                               (unsigned)f[0], (unsigned)f[1], (unsigned)f[2]);
                } else {
                    log_printf(" enc raw ERR");
                }
            }
        } else if (e->valid == 0u) {
            log_printf("enc = no frame yet (frames=%u io_err=%u)",
                       (unsigned)e->frames, (unsigned)e->io_errs);
        } else {
            log_printf("enc = mech %.2f elec %.2f deg, raw %u, mg %u, crc %s",
                       (double)e->mech_deg, (double)e->elec_deg,
                       (unsigned)e->raw, (unsigned)e->mg,
                       (e->crc_ok != 0u) ? "ok" : "BAD");
            log_printf(" spd = %.2f Hz elec, %.1f rpm mech",
                       (double)e->spd_hz, (double)e->spd_rpm);
            log_printf(" err = %u/%u crc, %u ovr, %u late, zero %.1f deg, dir %u, src %s",
                       (unsigned)e->crc_errs, (unsigned)e->frames,
                       (unsigned)e->ovr, (unsigned)e->late,
                       (double)e->zero_deg,
                       (unsigned)e->dir,
                       (e->src == ENC_SRC_FLASH) ? "flash" : "default");
        }
    }
    /* ---------------- cur ----------------- */
    else if (strcmp(tok[0], "cur") == 0) {
        const current_sample_t *cs = mid_current_get();
        if (cs->ready == 0u) {
            /* No calibration record and no live calibration yet: the nominal
             * 2048 in offset[] is not a measurement, so do not show it as one. */
            log_printf("cur = raw ----/----/----, off ----/----/---- (not calibrated)");
        } else if (bsp_adc_frames() == 0u) {
            /* Nothing has been converted yet: the TIM1_CC4 trigger only reaches
             * the ADC while the bridge is enabled. Show that instead of 0/0/0,
             * which looks like a broken reading. */
            log_printf("cur = raw ----/----/----, off %u/%u/%u",
                       (unsigned)cs->offset[0], (unsigned)cs->offset[1],
                       (unsigned)cs->offset[2]);
        } else {
            log_printf("cur = raw %u/%u/%u, off %u/%u/%u",
                       (unsigned)cs->raw[0], (unsigned)cs->raw[1],
                       (unsigned)cs->raw[2],
                       (unsigned)cs->offset[0], (unsigned)cs->offset[1],
                       (unsigned)cs->offset[2]);
        }
        log_printf("cur = %d/%d/%d mA, sat=%u",
                   (int)(cs->ia * 1000.0f), (int)(cs->ib * 1000.0f),
                   (int)(cs->ic * 1000.0f), (unsigned)cs->saturated);
        log_printf("adc = frames=%u, ovr=%u, cal=%u, src=%s, gen=%u, st=%u",
                   (unsigned)bsp_adc_frames(), (unsigned)bsp_adc_ovr_count(),
                   (unsigned)cs->ready, cal_src_str(cs->src),
                   (unsigned)cs->gen, (unsigned)bsp_adc_status());
        {
            uint32_t cfg[3];
            bsp_adc_get_cfg(cfg);
            log_printf("adc = cr=%08x cfgr=%08x jsqr=%08x",
                       (unsigned)cfg[0], (unsigned)cfg[1], (unsigned)cfg[2]);
        }
    }
    /* ---------------- cal ----------------- */
    else if (strcmp(tok[0], "cal") == 0) {
        if ((n >= 2u) && (strcmp(tok[1], "erase") == 0)) {
            /* Erasing restarts the averaging window, so the next 256 frames
             * become the new offset. If the bridge is running, force the zero
             * vector first: otherwise the load current would be averaged in
             * and baked into the stored offset. */
            if (app_status()->state == APP_ST_RUN) {
                bsp_pwm_set_all_duty(PWM_START_DUTY);
            }
            mid_current_erase_cal();
            log_printf("OK cal erased, will calibrate at next start");
        } else if (app_status()->state != APP_ST_RUN) {
            log_printf("ERR start first (ADC trigger needs MOE=1)");
        } else {
            /* Zero vector during calibration: no phase-to-phase voltage, so the
             * offset really is the zero-current reading. */
            bsp_pwm_set_all_duty(PWM_START_DUTY);
            mid_current_restart_cal();
            log_printf("OK cal started, %u frames at duty 0.50, saved to flash",
                       (unsigned)CUR_CAL_SAMPLES);
        }
    }
    /* ---------------- prof ---------------- */
    else if (strcmp(tok[0], "prof") == 0) {
        if ((n >= 2u) && (strcmp(tok[1], "reset") == 0)) {
            bsp_prof_reset();
            log_printf("OK prof reset");
        } else if ((n >= 2u) && (strcmp(tok[1], "off") == 0)) {
            bsp_prof_enable(0u);
            log_printf("OK prof off");
        } else if ((n >= 2u) && (strcmp(tok[1], "on") == 0)) {
            bsp_prof_enable(1u);
            log_printf("OK prof on");
        } else {
            static const char *const names[PROF_SLOT_COUNT] = {
                "isr", "isr_period", "adc_read", "conv",
                "wave", "cmd", "txpump", "loop", "enc"
            };
            uint8_t i;

            log_printf("---- profile (ns) ----");
            for (i = 0u; i < (uint8_t)PROF_SLOT_COUNT; i++) {
                log_printf(" %-10s n=%u min=%u avg=%u max=%u",
                           names[i],
                           (unsigned)bsp_prof_count((prof_slot_t)i),
                           (unsigned)bsp_prof_min_ns((prof_slot_t)i),
                           (unsigned)bsp_prof_avg_ns((prof_slot_t)i),
                           (unsigned)bsp_prof_max_ns((prof_slot_t)i));
            }
            log_printf("----------------------");
        }
    }
    /* ---------------- led ----------------- */
    else if (strcmp(tok[0], "led") == 0) {
        if (n < 2u)                             { log_printf("ERR usage=led <on|off|toggle>"); }
        else if (strcmp(tok[1], "on") == 0)     { bsp_led_write(1u); log_printf("OK led ON"); }
        else if (strcmp(tok[1], "off") == 0)    { bsp_led_write(0u); log_printf("OK led OFF"); }
        else if (strcmp(tok[1], "toggle") == 0) { bsp_led_toggle();   log_printf("OK led %s", bsp_led_is_on() ? "ON" : "OFF"); }
        else                                    { log_printf("ERR usage=led <on|off|toggle>"); }
    }
    /* ---------------- wave ---------------- */
    else if (strcmp(tok[0], "wave") == 0) {
        if (n < 2u) { log_printf("ERR usage=wave <on|off>"); }
        else if (strcmp(tok[1], "on") == 0) {
            app_wave_set(1u, app_status()->wave_rate_hz);
            log_printf("OK wave ON @ %u Hz, select FireWater in VOFA+",
                       (unsigned)app_status()->wave_rate_hz);
        } else if (strcmp(tok[1], "off") == 0) {
            app_wave_set(0u, app_status()->wave_rate_hz);
            log_printf("OK wave OFF");
        } else {
            log_printf("ERR usage=wave <on|off>");
        }
    }
    /* ---------------- rate ---------------- */
    else if (strcmp(tok[0], "rate") == 0) {
        if (n < 2u)                        { log_printf("ERR usage=rate <hz>"); }
        else if (!parse_u32(tok[1], &val)) { log_printf("ERR bad arg"); }
        else {
            app_wave_set(app_status()->wave_on, (uint16_t)val);
            log_printf("OK rate = %u Hz", (unsigned)app_status()->wave_rate_hz);
        }
    }
    /* ---------------- clr ----------------- */
    else if (strcmp(tok[0], "clr") == 0) {
        app_counters_clear();
        log_printf("OK counters cleared");
    }
    /* ---------------- echo ---------------- */
    else if (strcmp(tok[0], "echo") == 0) {
        if (n < 2u) { log_printf("(empty)"); }
        else        { log_printf("%s", tok[1]); }
    }
    /* ---------------- unknown ------------- */
    else {
        log_printf("ERR unknown cmd '%s', try 'help'", tok[0]);
    }
}

/* ------------------------------------------------------------------ */
/* Init / poll                                                         */
/* ------------------------------------------------------------------ */
void app_cmd_init(void)
{
    s_line_len = 0u;
    s_line[0]  = '\0';
}

void app_cmd_poll(void)
{
    uint8_t  buf[32];
    uint16_t n;
    uint16_t i;

    n = bsp_uart_rx_read(buf, (uint16_t)sizeof(buf));
    for (i = 0u; i < n; i++) {
        char c = (char)buf[i];

        if ((c == '\r') || (c == '\n')) {
            if (s_line_len > 0u) {
                s_line[s_line_len] = '\0';
                cmd_exec(s_line);
                s_line_len = 0u;
            }
        } else if ((c == 0x08) || (c == 0x7F)) {          /* backspace */
            if (s_line_len > 0u) { s_line_len--; }
        } else if ((c >= 0x20) && (c <= 0x7E)) {          /* printable ASCII only */
            if (s_line_len < (CMD_LINE_MAX - 1u)) {
                s_line[s_line_len++] = c;
            } else {
                s_line_len = 0u;
                log_printf("ERR line too long");
            }
        } else {
            /* ignore other control characters */
        }
    }
}
