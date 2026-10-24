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
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "mid_log.h"
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

static void print_stat(void)
{
    const app_status_t *st = app_status();

    log_printf("---- status ----");
    log_printf(" uptime   = %u ms", (unsigned)st->uptime_ms);
    log_printf(" state    = %s", app_state_name(st->state));
    log_printf(" mode     = %s", app_mode_name(st->mode));
    log_printf(" led      = %s", bsp_led_is_on() ? "ON" : "OFF");
    log_printf(" key      = down=%u total=%u",
               (unsigned)bsp_key_is_down(), (unsigned)st->key_cnt);
    log_printf(" loop     = %u kHz", (unsigned)st->loop_khz);
    log_printf(" wave     = %s @ %u Hz, frames=%u drop=%u",
               st->wave_on ? "ON" : "OFF",
               (unsigned)st->wave_rate_hz,
               (unsigned)st->wave_frames,
               (unsigned)st->wave_drop);
    log_printf(" uart     = rx_drop=%u tx_pending=%u",
               (unsigned)bsp_uart_rx_drop_count(),
               (unsigned)bsp_uart_tx_pending());
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
