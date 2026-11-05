/**
  ******************************************************************************
  * @file    test_main.c
  * @brief   Host unit tests for the pure-logic firmware modules.
  *
  *   Covered:
  *     - mid_ringbuf : basic, wrap-around, random fuzz vs reference model
  *     - mid_log     : all format specifiers, width/padding, truncation
  *     - mid_vofa    : byte-exact FireWater CSV frames
  *     - bsp_uart    : TX chunking, retry on HAL_BUSY, RX inject/wrap/overflow
  *     - bsp_gpio    : LED polarity, key debounce, press edges
  *     - app_main    : state machine, mode switching, wave rate clamping
  *     - app_cmd     : command parsing, error paths, backspace, overflow
  *
  *   Build & run:  test/host/build.sh
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "fake_api.h"
#include "board_cfg.h"
#include "mid_ringbuf.h"
#include "mid_log.h"
#include "mid_vofa.h"
#include "mid_current.h"
#include "bsp_gpio.h"
#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "bsp_prof.h"
#include "alg_svpwm.h"
#include "alg_vf.h"
#include "alg_pi.h"
#include "alg_foc.h"
#include "alg_spd.h"
#include "alg_pll.h"
#include "alg_obs.h"
#include "alg_if.h"
#include "alg_ident.h"
#include "mid_enc.h"
#include "mid_trig.h"
#include "bsp_uart.h"
#include "app_main.h"
#include "app_cmd.h"
#include "tim.h"

/* Extra hooks implemented in hal_stub.c */
extern int      fake_led_level(void);
extern int      fake_tx_call_count(void);
extern uint16_t fake_rx_pos(void);
extern int      fake_rx_started(void);

/* ------------------------------------------------------------------ */
/* Tiny test framework                                                 */
/* ------------------------------------------------------------------ */
static int g_checks;
static int g_fails;
static int g_fail_mark;    /* g_fails value at the start of the current section */

#define T(cond)                                                             \
    do {                                                                    \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            g_fails++;                                                      \
            printf("    FAIL  %s:%d  %s\n", __FILE__, __LINE__, #cond);     \
        }                                                                   \
    } while (0)

static void section(const char *name)
{
    printf("  %-42s", name);
    fflush(stdout);
    g_fail_mark = g_fails;
}

static void section_end(void)
{
    printf("ok\n");
}

static void pump_all(void);

/* Captured TX helpers. They first flush the firmware TX ring into the fake
 * UART, exactly like app_loop() does with bsp_uart_tx_pump(). */
static int tx_is(const char *expect)
{
    size_t n;
    pump_all();
    n = strlen(expect);
    if (fake_tx_len() != (uint32_t)n) { return 0; }
    return (memcmp(fake_tx_data(), expect, n) == 0) ? 1 : 0;
}

static int tx_contains(const char *needle)
{
    uint32_t n;
    size_t   k = strlen(needle);
    uint32_t i;

    pump_all();
    n = fake_tx_len();
    if (k == 0u || n < (uint32_t)k) { return 0; }
    for (i = 0u; i + k <= n; i++) {
        if (memcmp(&fake_tx_data()[i], needle, k) == 0) { return 1; }
    }
    return 0;
}

static void tx_dump_on_fail(void)
{
    uint32_t i;
    uint32_t n = fake_tx_len();
    printf("    TX(%u) = \"", (unsigned)n);
    for (i = 0u; (i < n) && (i < 200u); i++) {
        uint8_t c = fake_tx_data()[i];
        if (c == '\r')      { printf("\\r"); }
        else if (c == '\n') { printf("\\n"); }
        else if ((c >= 32) && (c < 127)) { printf("%c", c); }
        else                { printf("\\x%02X", c); }
    }
    printf("\"\n");
}

/* Deterministic PRNG (xorshift32) */
static uint32_t s_rng = 0x12345678u;
static uint32_t rnd(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return s_rng;
}

/* Drain the TX ring into the fake UART (simulates the main loop pumping) */
static void pump_all(void)
{
    int guard = 0;
    while ((bsp_uart_tx_pending() > 0u) && (guard++ < 10000)) {
        bsp_uart_tx_pump();
    }
}

/* Flush pending TX into the capture buffer, then start from a clean slate */
static void tx_clear(void)
{
    pump_all();
    fake_tx_reset();
}

/* Throw away anything left in the RX ring (previous test leftovers) */
static void rx_clear(void){
    uint8_t sink[64];
    int guard = 0;
    while ((bsp_uart_rx_avail() > 0u) && (guard++ < 100000)) {
        (void)bsp_uart_rx_read(sink, (uint16_t)sizeof(sink));
    }
}

/* mid_current becomes ready only after CUR_CAL_SAMPLES frames. The host has no
 * ADC, so feed the calibration directly (zero current -> offset 2048). */
static void force_current_ready(void)
{
    uint16_t r[3] = { 2048u, 2048u, 2048u };
    int i;
    for (i = 0; i < (int)CUR_CAL_SAMPLES; i++) {
        mid_current_update(r);
    }
}

/* Feed a command line and let the parser consume every byte */
static void cmd_feed(const char *s)
{
    fake_rx_feed_str(s);
    while (bsp_uart_rx_avail() > 0u) {
        app_cmd_poll();
    }
}

/* ================================================================== */
/* ring buffer                                                         */
/* ================================================================== */
static void test_ringbuf(void)
{
    ringbuf_t r;
    uint8_t   mem[16];
    uint8_t   out[16];
    uint8_t   in[16];
    int       i;

    section("ringbuf: basic push/pop");
    ring_init(&r, mem, 16u);
    T(r.size == 16u);
    T(ring_avail(&r) == 0u);
    T(ring_space(&r) == 15u);          /* one slot reserved */

    for (i = 0; i < 10; i++) { in[i] = (uint8_t)(0xA0 + i); }
    T(ring_push(&r, in, 10u) == 10u);
    T(ring_avail(&r) == 10u);
    T(ring_space(&r) == 5u);
    T(ring_pop(&r, out, 16u) == 10u);
    T(memcmp(out, in, 10u) == 0);
    T(ring_avail(&r) == 0u);
    section_end();

    section("ringbuf: full -> truncate");
    ring_init(&r, mem, 16u);
    for (i = 0; i < 16; i++) { in[i] = (uint8_t)i; }
    T(ring_push(&r, in, 16u) == 15u);  /* only size-1 fit */
    T(ring_avail(&r) == 15u);
    T(ring_push(&r, in, 4u) == 0u);    /* already full */
    section_end();

    section("ringbuf: wrap-around");
    ring_init(&r, mem, 16u);
    for (i = 0; i < 15; i++) { in[i] = (uint8_t)i; }
    T(ring_push(&r, in, 15u) == 15u);
    T(ring_pop(&r, out, 12u) == 12u);  /* tail now at 12 */
    in[0] = 0x55; in[1] = 0x66; in[2] = 0x77;
    T(ring_push(&r, in, 3u) == 3u);    /* 12,13,14 then should wrap */
    T(ring_pop(&r, out, 16u) == 6u);   /* 3 left + 3 new */
    T(out[0] == 12 && out[1] == 13 && out[2] == 14);
    T(out[3] == 0x55 && out[4] == 0x66 && out[5] == 0x77);
    section_end();

    section("ringbuf: peek stops at wrap boundary");
    ring_init(&r, mem, 16u);
    for (i = 0; i < 15; i++) { in[i] = (uint8_t)i; }
    (void)ring_push(&r, in, 15u);
    (void)ring_pop(&r, out, 12u);      /* tail = 12, 3 bytes left: 12,13,14 */
    in[0] = 0x99; in[1] = 0x66; in[2] = 0x77;
    (void)ring_push(&r, in, 3u);       /* lands at 15,0,1 -> head wraps to 2 */
    T(ring_avail(&r) == 6u);
    /* contiguous segment from tail(12) to the end of the buffer is 4 bytes */
    T(ring_peek(&r, out, 8u) == 4u);
    T(out[0] == 12 && out[1] == 13 && out[2] == 14 && out[3] == 0x99);
    /* after dropping those 4, the 2 wrapped bytes remain */
    ring_drop(&r, 4u);
    T(ring_avail(&r) == 2u);
    T(ring_peek(&r, out, 8u) == 2u);
    T(out[0] == 0x66 && out[1] == 0x77);
    section_end();

    section("ringbuf: non power-of-two rejected");
    ring_init(&r, mem, 15u);
    T(r.size == 0u);
    T(ring_space(&r) == 0u);
    T(ring_push(&r, in, 1u) == 0u);
    section_end();

    section("ringbuf: 200k random ops vs reference model");
    {
        ringbuf_t rr;
        uint8_t   rmem[64];
        uint8_t   model[63];           /* reference queue: capacity 63 */
        int       mhead = 0, mcount = 0;  /* model ring */
        uint32_t  k;
        int       bad = 0;

        ring_init(&rr, rmem, 64u);
        for (k = 0u; k < 200000u; k++) {
            uint32_t op = rnd() % 3u;
            if (op == 0u) {
                uint8_t  tmp[20];
                uint16_t len = (uint16_t)(rnd() % 20u);
                uint16_t got;
                uint16_t j;
                for (j = 0u; j < len; j++) { tmp[j] = (uint8_t)(rnd() & 0xFFu); }
                got = ring_push(&rr, tmp, len);
                /* model */
                {
                    uint16_t space = (uint16_t)(63 - mcount);
                    uint16_t take = (len > space) ? space : len;
                    for (j = 0u; j < take; j++) {
                        model[(mhead + mcount) % 63] = tmp[j];
                        mcount++;
                    }
                    if (got != take) { bad++; }
                }
            } else if (op == 1u) {
                uint8_t  tmp[20];
                uint16_t want = (uint16_t)(rnd() % 20u);
                uint16_t got  = ring_pop(&rr, tmp, want);
                uint16_t take = (want > (uint16_t)mcount) ? (uint16_t)mcount : want;
                uint16_t j;
                if (got != take) { bad++; }
                for (j = 0u; j < take; j++) {
                    if (tmp[j] != model[(mhead + j) % 63]) { bad++; }
                }
                mhead = (mhead + take) % 63;
                mcount -= take;
            } else {
                if (ring_avail(&rr) != (uint16_t)mcount) { bad++; }
            }
        }
        T(bad == 0);
        T(ring_avail(&rr) == (uint16_t)mcount);
    }
    section_end();
}

/* ================================================================== */
/* logging                                                             */
/* ================================================================== */
static void test_log(void)
{
    section("log: integer / string / char / percent");
    tx_clear();
    log_printf("a=%d b=%u c=%x d=%X s=%s c=%c p=100%%",
               42, 4000000000u, 0xABCDu, 0xABCDu, "hi", 'Z');
    T(tx_is("[log] a=42 b=4000000000 c=abcd d=ABCD s=hi c=Z p=100%\r\n"));
    if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    section_end();

    section("log: negative and INT32_MIN");
    tx_clear();
    log_printf("%d %d %d", -1, -2147483647 - 1, 0);
    T(tx_is("[log] -1 -2147483648 0\r\n"));
    if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    section_end();

    section("log: width / zero pad / left align");
    tx_clear();
    log_printf("[%5d][%05u][%-5d][%08x][%4s]", 42, 42u, 42, 0xABCDu, "ab");
    T(tx_is("[log] [   42][00042][42   ][0000abcd][  ab]\r\n"));
    if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    section_end();

    section("log: float 2 decimals");
    tx_clear();
    log_printf("%f|%f|%f|%f", 1.25f, -0.5f, 1234.5678f, 0.0f);
    T(tx_is("[log] 1.25|-0.50|1234.57|0.00\r\n"));
    if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    section_end();

    section("log: precision modifier consumed");
    tx_clear();
    log_printf("%.3f %.1f", 1.23456f, 2.345f);
    T(tx_is("[log] 1.23 2.35\r\n"));
    if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    section_end();

    section("log: colon is sanitised to semicolon");
    tx_clear();
    log_printf("t = %02d:%02d:%02d", 2, 31, 35);
    T(tx_is("[log] t = 02;31;35\r\n"));
    if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    section_end();

    section("log: unknown specifier passes through");
    tx_clear();
    log_printf("%q ok");
    T(tx_is("[log] %q ok\r\n"));
    if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    section_end();

    section("log: truncation at 128 bytes");
    {
        char big[300];
        memset(big, 'A', sizeof(big) - 1u);
        big[sizeof(big) - 1u] = '\0';
        tx_clear();
        log_printf("%s", big);
        pump_all();
        T(fake_tx_len() == 128u);
        T(memcmp(fake_tx_data(), "[log] ", 6u) == 0);
        T(fake_tx_data()[125] == 'A');
        T(fake_tx_data()[126] == '\r');
        T(fake_tx_data()[127] == '\n');
    }
    section_end();

    section("log: empty format gives bare CRLF");
    tx_clear();
    log_printf("");
    T(tx_is("\r\n"));
    section_end();
}

/* ================================================================== */
/* VOFA+ FireWater (CSV)                                                */
/* ================================================================== */
static void test_vofa(void)
{
    section("vofa: FireWater single channel");
    tx_clear();
    {
        float ch[1];
        ch[0] = 1.5f;
        T(vofa_send(ch, 1u) == 1u);
        T(tx_is("1.500\n"));
        if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    }
    section_end();

    section("vofa: FireWater 3 channels, negative value");
    tx_clear();
    {
        float ch[3];
        ch[0] =  1.0f;
        ch[1] = -1.0f;
        ch[2] =  0.0f;
        T(vofa_send(ch, 3u) == 1u);
        T(tx_is("1.000,-1.000,0.000\n"));
        if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    }
    section_end();

    section("vofa: FireWater 6 channels, bounded and newline terminated");
    tx_clear();
    {
        float ch[6];
        int i;
        for (i = 0; i < 6; i++) { ch[i] = (float)i; }
        T(vofa_send(ch, 6u) == 1u);
        pump_all();
        T(fake_tx_len() < 110u);                       /* must stay bounded */
        T(fake_tx_data()[fake_tx_len() - 1u] == '\n');
        T(tx_contains("3.000"));
        T(tx_contains("5.000"));
        if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    }
    section_end();

    section("vofa: rejects 0 and above the channel limit");
    tx_clear();
    {
        float ch[20];
        T(vofa_send(ch, 0u) == 0u);
        T(vofa_send(ch, (uint8_t)(VOFA_MAX_CHANNELS + 1u)) == 0u);
    T(vofa_send(ch, 17u) == 1u);          /* vbus made 17 legal */
        T(vofa_send(NULL, 3u) == 0u);
        T(fake_tx_len() == 0u);
    }
    section_end();
}

/* ================================================================== */
/* UART driver                                                         */
/* ================================================================== */
static void test_uart(void)
{
    section("uart: TX chunking (64B per DMA)");
    tx_clear();
    {
        uint8_t buf[200];
        int i;
        uint32_t before;
        for (i = 0; i < 200; i++) { buf[i] = (uint8_t)('a' + (i % 26)); }
        T(bsp_uart_tx_write(buf, 200u) == 200u);
        before = fake_tx_call_count();
        pump_all();
        T(fake_tx_len() == 200u);
        T(memcmp(fake_tx_data(), buf, 200u) == 0);
        /* 200 bytes / 64 = 4 transfers (64,64,64,8) */
        T((fake_tx_call_count() - before) == 4);
    }
    section_end();

    section("uart: TX ring capacity limit");
    tx_clear();
    {
        uint8_t buf[5000];
        memset(buf, 'x', sizeof(buf));
        /* ring capacity is size-1 = 4095 */
        T(bsp_uart_tx_write(buf, 5000u) == (uint16_t)(DBG_TX_RING_SIZE - 1u));
        T(bsp_uart_tx_pending() == (uint16_t)(DBG_TX_RING_SIZE - 1u));
        pump_all();
        T(bsp_uart_tx_pending() == 0u);
        T(fake_tx_len() == (uint32_t)(DBG_TX_RING_SIZE - 1u));
    }
    section_end();

    section("uart: retry when HAL returns BUSY");
    tx_clear();
    {
        uint8_t buf[10];
        memset(buf, 'k', sizeof(buf));
        T(bsp_uart_tx_write(buf, 10u) == 10u);
        fake_tx_force_busy(1);
        bsp_uart_tx_pump();                 /* first attempt fails */
        T(fake_tx_len() == 0u);
        T(bsp_uart_tx_pending() == 10u);    /* data kept */
        bsp_uart_tx_pump();                 /* retry succeeds */
        T(fake_tx_len() == 10u);
        T(bsp_uart_tx_pending() == 0u);
    }
    section_end();

    section("uart: TX watchdog recovers a lost completion event");
    tx_clear();
    fake_tick_set(1000u);
    {
        uint8_t a[5] = { 'h', 'e', 'l', 'l', 'o' };
        uint32_t stuck0 = bsp_uart_tx_stuck_count();

        /* DMA accepts the chunk but TxCplt never fires -> s_tx_busy stays set */
        T(bsp_uart_tx_write(a, 5u) == 5u);
        fake_tx_no_complete(1);
        bsp_uart_tx_pump();
        T(fake_tx_len() == 5u);
        T(bsp_uart_tx_pending() == 0u);
        T(bsp_uart_tx_idle() == 0u);            /* waiting for the callback */

        /* one tick short of the timeout: must not abort anything yet */
        fake_tick_add((uint32_t)DBG_TX_STUCK_MS - 1u);
        bsp_uart_tx_pump();
        T(fake_tx_abort_count() == 0);
        T(bsp_uart_tx_idle() == 0u);

        /* link healthy again, new data queued, timeout now expires */
        fake_tx_no_complete(0);
        T(bsp_uart_tx_write(a, 3u) == 3u);
        fake_tick_add(2u);
        bsp_uart_tx_pump();                     /* abort + release + resend */

        T(fake_tx_abort_count() == 1);
        T(bsp_uart_tx_stuck_count() == stuck0 + 1u);
        T(bsp_uart_tx_pending() == 0u);
        T(bsp_uart_tx_idle() == 1u);
        T(fake_tx_len() == 8u);                 /* the 3-byte chunk went out */
    }
    section_end();

    section("uart: RX inject simple");
    {
        uint8_t out[32];
        const char *msg = "hello";
        T(fake_rx_started() == 1);
        T(bsp_uart_rx_avail() == 0u);
        fake_rx_feed_str(msg);
        T(bsp_uart_rx_avail() == 5u);
        T(bsp_uart_rx_read(out, sizeof(out)) == 5u);
        T(memcmp(out, msg, 5u) == 0);
    }
    section_end();

    section("uart: RX wrap-around (160 bytes through a 128B buffer)");
    {
        uint8_t out[200];
        uint8_t expect[160];
        int i;
        uint8_t a[100];
        uint8_t b[60];

        for (i = 0; i < 100; i++) { a[i] = (uint8_t)i; }
        for (i = 0; i < 60;  i++) { b[i] = (uint8_t)(0x80 + i); }
        memcpy(expect, a, 100u);
        memcpy(&expect[100], b, 60u);

        fake_rx_feed(a, 100u);
        fake_rx_feed(b, 60u);
        T(bsp_uart_rx_avail() == 160u);
        T(bsp_uart_rx_read(out, sizeof(out)) == 160u);
        T(memcmp(out, expect, 160u) == 0);
    }
    section_end();

    section("uart: RX ring overflow counts drops");
    {
        uint8_t big[2000];
        int i;
        for (i = 0; i < 2000; i++) { big[i] = (uint8_t)i; }
        fake_rx_feed(big, 1000u);
        fake_rx_feed(&big[1000], 1000u);
        T(bsp_uart_rx_drop_count() > 0u);
        T(bsp_uart_rx_avail() <= DBG_RX_RING_SIZE);
        /* leave the parser and the RX ring clean for the next test */
        rx_clear();
        app_cmd_init();
    }
    section_end();
}

/* ================================================================== */
/* GPIO: LED polarity and key debounce                                 */
/* ================================================================== */
static void test_gpio(void)
{
    uint32_t t;

    section("gpio: LED polarity (PF1 low = ON)");
    bsp_led_write(0u);
    T(bsp_led_is_on() == 0u);
    T(fake_led_level() == 1);          /* PF1 high */
    bsp_led_write(1u);
    T(bsp_led_is_on() == 1u);
    T(fake_led_level() == 0);          /* PF1 low = lit */
    bsp_led_toggle();
    T(bsp_led_is_on() == 0u);
    T(fake_led_level() == 1);
    section_end();

    section("gpio: key debounce and press edge");
    /* clear any pending event */
    (void)bsp_key_take_press();
    (void)bsp_key_take_release();
    t = fake_tick_get();

    /* press, but shorter than the debounce window -> no event */
    fake_key_set(1);
    fake_tick_set(t + 1u);  bsp_key_poll(fake_tick_get());
    fake_tick_set(t + 10u); bsp_key_poll(fake_tick_get());   /* only 9ms */
    T(bsp_key_take_press() == 0u);
    T(bsp_key_is_down() == 0u);

    /* keep it pressed past 20ms -> event fires exactly once */
    fake_tick_set(t + 30u); bsp_key_poll(fake_tick_get());
    T(bsp_key_is_down() == 1u);
    T(bsp_key_take_press() == 1u);
    T(bsp_key_take_press() == 0u);           /* consumed */

    /* holding longer must not re-fire */
    fake_tick_set(t + 500u); bsp_key_poll(fake_tick_get());
    T(bsp_key_take_press() == 0u);

    /* release */
    fake_key_set(0);
    fake_tick_set(t + 501u); bsp_key_poll(fake_tick_get());
    fake_tick_set(t + 530u); bsp_key_poll(fake_tick_get());
    T(bsp_key_is_down() == 0u);
    T(bsp_key_take_release() == 1u);
    section_end();
}

/* ================================================================== */
/* PWM: pure conversion + register sequencing                          */
/* ================================================================== */
static void test_pwm(void)
{
    section("pwm: duty to CCR mapping (PWM2: high at the counter peak)");
    T(bsp_pwm_duty_to_ccr(0.0f) == PWM_CCR_MAX);
    T(bsp_pwm_duty_to_ccr(0.1f) == 3825u);
    T(bsp_pwm_duty_to_ccr(0.5f) == 2125u);
    T(bsp_pwm_duty_to_ccr(0.9f) == 425u);
    T(bsp_pwm_duty_to_ccr(1.0f) == PWM_CCR_MIN);
    section_end();

    section("pwm: out-of-range and NaN duty clamp to the safe window");
    T(bsp_pwm_duty_to_ccr(-0.5f) == PWM_CCR_MAX);
    T(bsp_pwm_duty_to_ccr(1.5f) == PWM_CCR_MIN);
    {
        volatile float z = 0.0f;
        float nan_v = z / z;
        T(bsp_pwm_duty_to_ccr(nan_v) == PWM_CCR_MAX);   /* NaN -> duty 0 */
    }
    section_end();

    section("pwm: ccr to duty round trip");
    T(fabsf(bsp_pwm_ccr_to_duty((uint16_t)2125u) - 0.5f) < 1e-6f);
    T(fabsf(bsp_pwm_ccr_to_duty(0u) - 1.0f) < 1e-6f);
    T(fabsf(bsp_pwm_ccr_to_duty((uint16_t)PWM_CCR_PERIOD) - 0.0f) < 1e-6f);
    section_end();

    section("pwm: init leaves the bridge off with 50% preloaded");
    memset(&fake_tim1_regs, 0, sizeof(fake_tim1_regs));
    fake_tim1_regs.BDTR = 4u;                       /* DTG as in tim.c */
    fake_tim1_regs.CCR4 = 4000u;                    /* CH4 as configured in CubeMX */
    bsp_pwm_init();
    T((fake_tim1_regs.BDTR & TIM_BDTR_MOE) == 0u);
    T((fake_tim1_regs.CCER & TIM_CCER_CC1E) == 0u);  /* main outputs off */
    T((fake_tim1_regs.CCER & TIM_CCER_CC4E) != 0u); /* CH4 stays on for ADC */
    /* CCR preload must be on so duty updates land at the update event */
    T((fake_tim1_regs.CCMR1 & TIM_CCMR1_OC1PE) != 0u);
    T((fake_tim1_regs.CCMR1 & TIM_CCMR1_OC2PE) != 0u);
    T((fake_tim1_regs.CCMR2 & TIM_CCMR2_OC3PE) != 0u);
    T(fake_tim1_regs.CCR1 == 2125u);
    T(fake_tim1_regs.CCR2 == 2125u);
    T(fake_tim1_regs.CCR3 == 2125u);
    T(fake_tim1_regs.CCR4 == 4000u);                /* firmware must not override CubeMX */
    T((fake_tim1_regs.CR1 & TIM_CR1_CEN) != 0u);    /* counter running */
    T((fake_tim1_regs.EGR & TIM_EGR_UG) != 0u);     /* preload latched */
    T(bsp_pwm_is_enabled() == 0u);
    T(bsp_pwm_get_dtg() == 4u);
    section_end();

    section("pwm: enable sets CCxE and MOE, disable clears them");
    bsp_pwm_enable();
    T(bsp_pwm_is_enabled() == 1u);
    T((fake_tim1_regs.CCER & TIM_CCER_CC1E) != 0u);
    T((fake_tim1_regs.CCER & TIM_CCER_CC1NE) != 0u);
    T((fake_tim1_regs.CCER & TIM_CCER_CC2E) != 0u);
    T((fake_tim1_regs.CCER & TIM_CCER_CC3NE) != 0u);
    T((fake_tim1_regs.CCER & TIM_CCER_CC4E) != 0u);
    bsp_pwm_disable();
    T(bsp_pwm_is_enabled() == 0u);
    T((fake_tim1_regs.CCER & TIM_CCER_CC1E) == 0u);  /* main outputs off */
    T((fake_tim1_regs.CCER & TIM_CCER_CC4E) != 0u); /* CH4 kept for ADC */
    section_end();

    section("pwm: per-phase duty reaches the right CCR");
    bsp_pwm_enable();
    bsp_pwm_set_duty(0.25f, 0.5f, 0.75f);
    T(fake_tim1_regs.CCR1 == 3188u);                /* CCR = (1 - duty) * 4250 */
    T(fake_tim1_regs.CCR2 == 2125u);
    T(fake_tim1_regs.CCR3 == 1063u);
    {
        float a, b, c;
        bsp_pwm_get_duty(&a, &b, &c);
        T(fabsf(a - 0.25f) < 2e-3f);          /* readback is the applied CCR */
        T(fabsf(b - 0.50f) < 2e-3f);
        T(fabsf(c - 0.75f) < 2e-3f);
    }
    section_end();

    section("pwm: readback reports the applied duty, not the request");
    bsp_pwm_set_duty(0.0f, 0.5f, 1.0f);
    T(fake_tim1_regs.CCR1 == PWM_CCR_MAX);
    T(fake_tim1_regs.CCR3 == PWM_CCR_MIN);
    {
        float a, b, c;
        bsp_pwm_get_duty(&a, &b, &c);
        T(fabsf(a - (1.0f - 4209.0f / 4250.0f)) < 5e-4f);   /* 0.94% floor */
        T(fabsf(b - 0.5f) < 1e-6f);
        T(fabsf(c - (1.0f - 40.0f / 4250.0f)) < 5e-4f);     /* 99.06% ceiling */
    }
    section_end();

    section("pwm: disable keeps duty, set_all_duty drives three phases");
    bsp_pwm_set_duty(0.25f, 0.5f, 0.75f);
    bsp_pwm_disable();
    T(fake_tim1_regs.CCR1 == 3188u);                 /* duty survives stop */
    bsp_pwm_set_all_duty(0.2f);
    T(fake_tim1_regs.CCR1 == 3400u);
    T(fake_tim1_regs.CCR2 == 3400u);
    T(fake_tim1_regs.CCR3 == 3400u);
    section_end();
}

/* ================================================================== */
/* Current conversion (Mid)                                            */
/* ================================================================== */
static void test_current(void)
{
    uint16_t r[3];
    uint16_t i;

    section("current: 256-frame zero-drift calibration");
    mid_current_init();
    T(mid_current_get()->ready == 0u);
    T(mid_current_get()->offset[0] == (uint16_t)CUR_ZERO_CODE);
    for (i = 0u; i < (uint8_t)(CUR_CAL_SAMPLES - 1u); i++) {
        r[0] = 2050u; r[1] = 2049u; r[2] = 2051u;
        mid_current_update(r);
    }
    T(mid_current_get()->ready == 0u);          /* one frame short */
    T(mid_current_get()->ia == 0.0f);           /* reports 0 while calibrating */
    r[0] = 2050u; r[1] = 2049u; r[2] = 2051u;
    mid_current_update(r);
    T(mid_current_get()->ready == 1u);
    T(mid_current_get()->offset[0] == 2050u);
    T(mid_current_get()->offset[1] == 2049u);
    T(mid_current_get()->offset[2] == 2051u);
    section_end();

    section("current: conversion = (raw - offset) * A/LSB * CUR_SIGN");
    r[0] = 2050u + 155u; r[1] = 2049u; r[2] = 2051u;
    mid_current_update(r);
    T(fabsf(mid_current_get()->ia - 0.099975f * CUR_SIGN) < 1e-3f);
    T(fabsf(mid_current_get()->ib) < 1e-6f);
    r[0] = 2050u - 155u; r[1] = 2049u; r[2] = 2051u;
    mid_current_update(r);
    T(fabsf(mid_current_get()->ia + 0.099975f * CUR_SIGN) < 1e-3f);   /* opposite direction */
    section_end();

    section("current: saturation flag at the rails");
    r[0] = 2050u; r[1] = 2049u; r[2] = 2051u; mid_current_update(r);
    T(mid_current_get()->saturated == 0u);
    r[0] = 2u;    r[1] = 2049u; r[2] = 2051u; mid_current_update(r);
    T(mid_current_get()->saturated == 1u);
    r[0] = 2050u; r[1] = 4094u; r[2] = 2051u; mid_current_update(r);
    T(mid_current_get()->saturated == 1u);
    section_end();

    section("current: restart returns to calibration");
    mid_current_restart_cal();
    T(mid_current_get()->ready == 0u);
    T(mid_current_get()->offset[0] == (uint16_t)CUR_ZERO_CODE);
    section_end();

    section("current: calibration is persisted to flash and reloaded");
    fake_flash_reset();
    mid_current_init();
    for (i = 0u; i < (uint16_t)CUR_CAL_SAMPLES; i++) {
        r[0] = 2050u; r[1] = 2049u; r[2] = 2051u;
        mid_current_update(r);
    }
    T(mid_current_get()->ready == 1u);
    T(mid_current_get()->offset[0] == 2050u);
    T(mid_current_get()->src == CUR_SRC_LIVE);
    T(mid_current_get()->gen == 1u);          /* first save */
    /* a second calibration must bump the generation (proves a new write) */
    mid_current_restart_cal();
    for (i = 0u; i < (uint16_t)CUR_CAL_SAMPLES; i++) {
        r[0] = 2050u; r[1] = 2049u; r[2] = 2051u;
        mid_current_update(r);
    }
    T(mid_current_get()->gen == 2u);
    /* simulate a power cycle: fresh init, then load from flash */
    mid_current_init();
    T(mid_current_get()->ready == 0u);
    T(mid_current_get()->src == CUR_SRC_NONE);
    T(mid_current_get()->offset[0] == (uint16_t)CUR_ZERO_CODE);
    T(mid_current_load() == 1u);
    T(mid_current_get()->ready == 1u);
    T(mid_current_get()->src == CUR_SRC_FLASH);
    T(mid_current_get()->offset[0] == 2050u);
    T(mid_current_get()->offset[1] == 2049u);
    T(mid_current_get()->offset[2] == 2051u);
    section_end();

    section("current: corrupt flash record is rejected");
    fake_flash_page[3] ^= 0x5Au;              /* break the CRC */
    mid_current_init();
    T(mid_current_load() == 0u);
    T(mid_current_get()->ready == 0u);
    section_end();

    section("current: erase returns to uncalibrated");
    fake_flash_reset();
    mid_current_init();
    for (i = 0u; i < (uint16_t)CUR_CAL_SAMPLES; i++) {
        r[0] = 2050u; r[1] = 2049u; r[2] = 2051u;
        mid_current_update(r);
    }
    T(mid_current_get()->ready == 1u);
    mid_current_erase_cal();
    T(mid_current_get()->ready == 0u);
    T(mid_current_load() == 0u);
    section_end();
}

/* ================================================================== */
/* DWT profiler (Bsp)                                                  */
/* ================================================================== */
static void test_prof(void)
{
    section("prof: min/avg/max from explicit cycle counts");
    bsp_prof_init();
    bsp_prof_reset();
    bsp_prof_add_cycles(PROF_ISR, 170u);        /* 1000 ns @170MHz */
    bsp_prof_add_cycles(PROF_ISR, 340u);        /* 2000 ns */
    T(bsp_prof_count(PROF_ISR) == 2u);
    T(bsp_prof_min_ns(PROF_ISR) == 1000u);
    T(bsp_prof_max_ns(PROF_ISR) == 2000u);
    T(bsp_prof_avg_ns(PROF_ISR) == 1500u);
    T(bsp_prof_count(PROF_WAVE) == 0u);
    T(bsp_prof_min_ns(PROF_WAVE) == 0u);        /* empty slot reads 0, not 0xFFFF */
    section_end();

    section("prof: enable / disable and reset");
    bsp_prof_enable(0u);
    bsp_prof_add_cycles(PROF_WAVE, 170u);
    T(bsp_prof_count(PROF_WAVE) == 0u);
    bsp_prof_enable(1u);
    bsp_prof_add_cycles(PROF_WAVE, 170u);
    T(bsp_prof_count(PROF_WAVE) == 1u);
    bsp_prof_reset();
    T(bsp_prof_count(PROF_WAVE) == 0u);
    section_end();

    section("prof: now()/add() bracket the fake DWT");
    fake_dwt.CYCCNT = 1000u;
    {
        uint32_t t0 = bsp_prof_now();
        fake_dwt.CYCCNT = 1170u;
        bsp_prof_add(PROF_CMD, t0);
    }
    T(bsp_prof_count(PROF_CMD) == 1u);
    T(bsp_prof_min_ns(PROF_CMD) == 1000u);
    section_end();
}

/* ================================================================== */
/* ADC driver (Bsp)                                                    */
/* ================================================================== */
static void test_adc(void)
{
    section("adc: JEOC latches JDR1..3 and counts frames");
    memset(&fake_adc2_regs, 0, sizeof(fake_adc2_regs));
    bsp_prof_enable(0u);                        /* keep timings out of the way */
    bsp_adc_init();
    T(bsp_adc_started() == 1u);
    T(bsp_adc_frames() == 0u);
    fake_adc2_regs.JDR1 = 2100u;
    fake_adc2_regs.JDR2 = 2200u;
    fake_adc2_regs.JDR3 = 2300u;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc2);
    T(bsp_adc_frames() == 1u);
    {
        uint16_t r[3];
        bsp_adc_get_raw(r);
        T(r[0] == 2100u);
        T(r[1] == 2200u);
        T(r[2] == 2300u);
    }
    section_end();

    section("adc: overrun is counted and cleared");
    fake_adc2_regs.ISR = ADC_ISR_OVR;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc2);
    T(bsp_adc_ovr_count() == 1u);
    T((fake_adc2_regs.ISR & ADC_ISR_OVR) == 0u);
    section_end();

    /* Reset the frame counter: app_loop() consumes any new frame, and a stale
     * one (JDR = 2100/2200/2300 above) would show up as a fake current. */
    bsp_adc_init();
    T(bsp_adc_frames() == 0u);

    section("phase probe: ISR cnt + stage durations");
    bsp_prof_enable(1u);                          /* the probe follows it */
    app_phase_reset();
    fake_tim1_regs.CNT = 1234u;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc2);   /* ISR latches CNT=1234 */
    T(bsp_adc_frames() == 1u);
    app_loop();                                   /* main loop consumes it */
    T(app_phase_info()->isr.cnt == 1234u);
    T(app_phase_info()->isr.valid == 1u);
    T(app_phase_info()->pick.valid == 1u);        /* pickup latency measured */
    app_phase_reset();
    T(app_phase_info()->isr.valid == 0u);
    T(app_phase_info()->pick.valid == 0u);
    section_end();

    section("phase probe: disabled together with the profiler");
    app_phase_reset();
    bsp_prof_enable(0u);
    fake_tim1_regs.CNT = 2000u;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc2);   /* probe must stay silent */
    app_loop();
    T(app_phase_info()->isr.valid == 0u);
    T(app_phase_info()->pick.valid == 0u);
    bsp_prof_enable(1u);
    app_phase_reset();
    section_end();

    bsp_prof_enable(1u);
}

/* ================================================================== */
/* Encoder (Mid + Bsp fake)                                            */
/* ================================================================== */
extern uint8_t fake_ssi_frame[3];
extern uint8_t fake_ssi_fail;
extern uint8_t fake_ssi_busy;

static void enc_build_frame(uint16_t d, uint8_t mg)
{
    uint32_t bits = (((uint32_t)d & 0x3FFFu) << 4) | ((uint32_t)mg & 0x0Fu);
    uint8_t  crc  = mid_enc_crc6(bits);
    uint32_t w    = (bits << 6) | (uint32_t)(crc & 0x3Fu);

    fake_ssi_frame[0] = (uint8_t)((w >> 16) & 0xFFu);
    fake_ssi_frame[1] = (uint8_t)((w >> 8) & 0xFFu);
    fake_ssi_frame[2] = (uint8_t)(w & 0xFFu);
}

static void test_enc(void)
{
    const mid_enc_t *e;

    fake_ssi_fail = 0u;
    mid_enc_init();

    section("enc: no frame yet");
    e = mid_enc_info();
    T(e->valid == 0u);
    T(e->frames == 0u);
    section_end();

    section("enc: raw zero maps to 0 deg, CRC accepted");
    enc_build_frame(0u, 0u);
    mid_enc_update();
    e = mid_enc_info();
    T(e->valid == 1u);
    T(e->crc_ok == 1u);
    T(e->raw == 0u);
    T(fabsf(e->mech_deg) < 0.01f);
    T(fabsf(e->elec_deg) < 0.01f);
    section_end();

    section("enc: quarter and half turn");
    enc_build_frame(4096u, 0u);                 /* 90 deg mechanical */
    mid_enc_update();
    e = mid_enc_info();
    T(fabsf(e->mech_deg - 90.0f) < 0.02f);
    T(fabsf(e->elec_deg - 270.0f) < 0.05f);     /* 90*7 = 630 -> 270 */

    enc_build_frame(8192u, 0u);                 /* 180 deg */
    mid_enc_update();
    T(fabsf(mid_enc_info()->mech_deg - 180.0f) < 0.02f);
    section_end();

    section("enc: magnetic status is decoded");
    enc_build_frame(100u, 2u);                  /* field too weak */
    mid_enc_update();
    T(mid_enc_info()->mg == 2u);
    section_end();

    section("enc: a corrupted CRC is counted but the angle is kept");
    {
        uint32_t before = mid_enc_info()->crc_errs;

        enc_build_frame(1000u, 0u);
        fake_ssi_frame[2] ^= 0x01u;             /* flip a CRC bit */
        mid_enc_update();
        e = mid_enc_info();
        T(e->crc_ok == 0u);
        T(e->crc_errs == before + 1u);
        T(e->raw == 1000u);
    }
    section_end();

    section("enc: late only counts an armed-but-unfinished frame");
    {
        uint32_t before = mid_enc_info()->late;

        /* Nothing armed (bridge off): this call just arms the next frame, so
         * it must NOT be counted as late. */
        fake_ssi_fail = 1u;
        fake_ssi_busy = 0u;
        mid_enc_update();
        T(mid_enc_info()->late == before);

        /* Armed but the DMA has not finished: that is a real late frame. */
        fake_ssi_busy = 1u;
        mid_enc_update();
        T(mid_enc_info()->late == before + 1u);

        fake_ssi_fail = 0u;
        fake_ssi_busy = 0u;
        mid_enc_update();          /* recover so later tests see fresh data */
    }
    section_end();

    section("enc: 15 real frames captured on hardware decode with a good CRC");
    {
        /* Raw SSI frames logged from the board (see docs/14 section 9).
         * They validate the table-driven CRC against the chip, not against
         * our own encoder function. */
        static const uint8_t raw[15][3] = {
            { 0x4Eu, 0xCAu, 0x05u }, { 0x4Eu, 0xC8u, 0x1Du },
            { 0xAEu, 0x7Cu, 0x2Du }, { 0xAEu, 0x7Eu, 0x35u },
            { 0xB3u, 0x60u, 0x05u }, { 0xB3u, 0x62u, 0x1Du },
            { 0xBDu, 0xE0u, 0x1Au }, { 0xBDu, 0xE4u, 0x2Au },
            { 0xC8u, 0x2Eu, 0x03u }, { 0xC8u, 0x2Cu, 0x1Bu },
            { 0xCFu, 0x0Eu, 0x33u }, { 0xCFu, 0x0Cu, 0x2Bu },
            { 0xD4u, 0x76u, 0x1Cu }, { 0xD4u, 0x78u, 0x17u },
            { 0xD4u, 0x7Au, 0x0Fu },
        };
        uint8_t i;
        uint8_t bad = 0u;

        for (i = 0u; i < 15u; i++) {
            fake_ssi_frame[0] = raw[i][0];
            fake_ssi_frame[1] = raw[i][1];
            fake_ssi_frame[2] = raw[i][2];
            mid_enc_update();
            if (mid_enc_info()->crc_ok == 0u) {
                bad++;
            }
        }
        T(bad == 0u);
    }
    section_end();
    section("enc: zero capture makes the current position 0 deg");
    enc_build_frame(2048u, 0u);                 /* 45 deg */
    mid_enc_update();
    T(mid_enc_zero() == 0u);
    T(fabsf(mid_enc_info()->zero_deg - 45.0f) < 0.05f);
    mid_enc_update();
    T(fabsf(mid_enc_info()->mech_deg) < 0.02f);
    section_end();

    section("enc: direction reverses the electrical angle");
    enc_build_frame(2048u + 468u, 0u);          /* zero + ~10.3 deg */
    mid_enc_update();
    {
        float fwd = mid_enc_info()->elec_deg;

        T(mid_enc_set_dir(1u) == 0u);
        mid_enc_update();
        T(fabsf(mid_enc_info()->elec_deg - (360.0f - fwd)) < 0.05f);
        T(mid_enc_set_dir(0u) == 0u);
    }
    section_end();

    section("enc: speed estimate over a 10ms window");
    {
        uint8_t k;
        extern DWT_TypeDef fake_dwt;

        mid_enc_set_spd_src(0u);   /* S8.1: the PLL is default, this tests the window */
        fake_dwt.CYCCNT = 1000000u;
        enc_build_frame(0u, 0u);
        mid_enc_update();                 /* first sample: no delta yet */

        /* 3.6 deg mechanical per 1ms = 10 rev/s = 70 Hz electrical */
        for (k = 1u; k <= 12u; k++) {
            uint16_t d = (uint16_t)((uint32_t)k * 164u);   /* 3.6 deg = 163.84 LSB */

            fake_dwt.CYCCNT += (SYS_CORE_CLK_HZ / 1000u);
            enc_build_frame(d, 0u);
            mid_enc_update();
        }
        T(fabsf(mid_enc_info()->spd_hz - 70.0f) < 3.0f);
        T(fabsf(mid_enc_info()->spd_rpm - 600.0f) < 30.0f);
    }
    section_end();

    section("enc: speed drops to zero when the rotor stops");
    {
        uint8_t k;
        extern DWT_TypeDef fake_dwt;

        /* 25 frames so a second, fully static 10ms window completes even when
         * the first one still carried deltas from the previous test */
        for (k = 0u; k < 25u; k++) {
            fake_dwt.CYCCNT += (SYS_CORE_CLK_HZ / 1000u);
            enc_build_frame(1968u, 0u);
            mid_enc_update();
        }
        T(fabsf(mid_enc_info()->spd_hz) < 1.0f);
    }
    section_end();
    section("enc: zero and direction survive a re-init (flash)");
    mid_enc_init();
    e = mid_enc_info();
    T(e->src == ENC_SRC_FLASH);
    T(fabsf(e->zero_deg - 45.0f) < 0.05f);
    T(e->dir == 0u);
    section_end();
}

/* ================================================================== */
/* SVPWM (Alg)                                                         */
/* ================================================================== */
/* ---- sin/cos back end (Mid) ---- */
static void test_trig(void)
{
    int   i;
    float worst = 0.0f;

    section("trig: back end exposes a name");
    T(strcmp(mid_trig_name(), "LUT") == 0 || strcmp(mid_trig_name(), "CORDIC") == 0);
    T(mid_trig_init() == 0u);
    section_end();

    section("trig: cos/sin match the C library");
    for (i = 0; i < 720; i++) {
        float th = -3.14159265f + (float)i * 6.2831853f / 720.0f;
        float c, s;

        mid_trig_cos_sin(th, &c, &s);
        {
            float dc = fabsf(c - cosf(th));
            float ds = fabsf(s - sinf(th));

            if (dc > worst) { worst = dc; }
            if (ds > worst) { worst = ds; }
        }
    }
    /* LUT interpolation error is 7.5e-5 of amplitude; 1e-3 is generous. */
    T(worst < 1e-3f);
    section_end();

    section("trig: angles outside [0, 2pi) are wrapped");
    {
        float c1, s1, c2, s2;

        mid_trig_cos_sin(0.5f, &c1, &s1);
        mid_trig_cos_sin(0.5f + 6.2831853f * 3.0f, &c2, &s2);
        T(fabsf(c1 - c2) < 1e-4f);
        T(fabsf(s1 - s2) < 1e-4f);
        mid_trig_cos_sin(0.5f - 6.2831853f * 2.0f, &c2, &s2);
        T(fabsf(c1 - c2) < 1e-4f);
    }
    section_end();
}

/* cosf-based reference, used only to cross-check the lookup table. */
static void ref_svpwm(float theta, float m, float duty[3])
{
    float hi, lo, off;
    int   i;

    duty[0] = 0.5f + 0.5f * m * cosf(theta);
    duty[1] = 0.5f + 0.5f * m * cosf(theta - 2.0943951f);
    duty[2] = 0.5f + 0.5f * m * cosf(theta + 2.0943951f);

    hi = duty[0];
    lo = duty[0];
    for (i = 1; i < 3; i++) {
        if (duty[i] > hi) { hi = duty[i]; }
        if (duty[i] < lo) { lo = duty[i]; }
    }
    off = 0.5f * (hi + lo) - 0.5f;
    for (i = 0; i < 3; i++) { duty[i] -= off; }
}

static void test_svpwm(void)
{
    float d[3];
    int   i;

    section("svpwm: zero modulation is the 50% zero vector");
    alg_svpwm(0.0f, 0.0f, d);
    T(fabsf(d[0] - 0.5f) < 1e-6f);
    T(fabsf(d[1] - 0.5f) < 1e-6f);
    T(fabsf(d[2] - 0.5f) < 1e-6f);
    alg_svpwm(1.234f, 0.0f, d);
    T(fabsf(d[0] - 0.5f) < 1e-6f && fabsf(d[1] - 0.5f) < 1e-6f);
    section_end();

    section("svpwm: duties stay in [0,1] and are centred (max+min == 1)");
    {
        int bad_range = 0;
        int bad_centre = 0;

        for (i = 0; i < 720; i++) {
            float th = (float)i * 3.14159265f / 360.0f;
            float hi, lo;
            int k;

            alg_svpwm(th, 1.0f, d);
            hi = d[0]; lo = d[0];
            for (k = 0; k < 3; k++) {
                if (d[k] < -1e-5f || d[k] > 1.00001f) { bad_range++; }
                if (d[k] > hi) { hi = d[k]; }
                if (d[k] < lo) { lo = d[k]; }
            }
            if (fabsf(hi + lo - 1.0f) > 1e-5f) { bad_centre++; }
        }
        T(bad_range == 0);
        T(bad_centre == 0);
    }
    section_end();

    section("svpwm: line-to-line amplitude is 0.866*m");
    {
        float max_ll = 0.0f;

        for (i = 0; i < 720; i++) {
            float th = (float)i * 3.14159265f / 360.0f;
            float ll;

            alg_svpwm(th, 1.0f, d);
            ll = fabsf(d[0] - d[1]);
            if (ll > max_ll) { max_ll = ll; }
        }
        /* (sqrt3/2) * m with m = 1 */
        T(fabsf(max_ll - 0.8660254f) < 0.005f);
    }
    section_end();

    section("svpwm: theta = 0 puts phase A on top, B == C");
    alg_svpwm(0.0f, 1.0f, d);
    T(d[0] > d[1] && d[0] > d[2]);
    T(fabsf(d[1] - d[2]) < 1e-6f);
    T(d[1] > 0.0f && d[0] < 1.0f);         /* never hits the rails */
    section_end();

    section("svpwm: negating theta swaps B and C (dir == 1)");
    {
        float fwd[3], rev[3];

        alg_svpwm(0.7f, 0.8f, fwd);
        alg_svpwm(-0.7f, 0.8f, rev);
        T(fabsf(fwd[0] - rev[0]) < 1e-6f);
        T(fabsf(fwd[1] - rev[2]) < 1e-6f);
        T(fabsf(fwd[2] - rev[1]) < 1e-6f);
    }
    section_end();

    section("svpwm: lookup table matches cosf within 1 PWM count");
    {
        float a[3], b[3];
        float worst = 0.0f;

        for (i = 0; i < 720; i++) {
            float th = -3.14159265f + (float)i * 6.2831853f / 720.0f;
            int   k;

            alg_svpwm(th, 0.95f, a);
            ref_svpwm(th, 0.95f, b);
            for (k = 0; k < 3; k++) {
                float d = fabsf(a[k] - b[k]);
                if (d > worst) { worst = d; }
            }
        }
        /* one PWM count = 1/4250 = 2.35e-4 */
        T(worst < (1.0f / 4250.0f));
    }
    section_end();
}

/* ================================================================== */
/* V/f ramp (Alg)                                                      */
/* ================================================================== */
static void test_vf(void)
{
    alg_vf_t vf;
    int      i;

    section("vf: ramp runs at the configured rate");
    alg_vf_init(&vf);
    alg_vf_start(&vf, 100.0f);
    for (i = 0; i < 20000; i++) {          /* 1s at 20kHz */
        (void)alg_vf_step(&vf, 0.00005f);
    }
    T(fabsf(vf.f_hz - VF_ACC_HZ_PER_S) < 0.05f);
    section_end();

    section("vf: ramp stops exactly at the target");
    for (i = 0; i < 200000; i++) {
        (void)alg_vf_step(&vf, 0.00005f);
    }
    T(fabsf(vf.f_hz - 100.0f) < 1e-3f);
    section_end();

    section("vf: angle advances by 2*pi*f*ts and stays wrapped");
    alg_vf_init(&vf);
    alg_vf_start(&vf, 50.0f);
    vf.f_hz = 50.0f;
    {
        float th0 = vf.theta;
        (void)alg_vf_step(&vf, 0.00005f);
        T(fabsf((vf.theta - th0) - 6.2831853f * 50.0f * 0.00005f) < 1e-6f);
    }
    for (i = 0; i < 40000; i++) {          /* 2s = 100 electrical cycles */
        (void)alg_vf_step(&vf, 0.00005f);
    }
    T(vf.theta >= 0.0f && vf.theta < 6.2831854f);
    section_end();

    section("vf: modulation index follows the V/f curve");
    alg_vf_init(&vf);
    vf.f_hz = 10.0f;                       /* 1.5 + 0.26  = 1.76V -> 0.2395 */
    T(fabsf(alg_vf_mod_index(&vf) - 0.2395f) < 1e-3f);
    vf.f_hz = 200.0f;                      /* 1.5 + 5.2   = 6.7V  -> 0.9118 */
    T(fabsf(alg_vf_mod_index(&vf) - 0.9118f) < 1e-3f);
    vf.f_hz = 500.0f;                      /* clamps at VF_M_MAX */
    T(fabsf(alg_vf_mod_index(&vf) - VF_M_MAX) < 1e-6f);
    section_end();

    section("vf: foldback scales the modulation index");
    alg_vf_init(&vf);
    vf.f_hz = 10.0f;
    vf.m_scale = 0.5f;
    T(fabsf(alg_vf_mod_index(&vf) - 0.1198f) < 1e-3f);
    vf.m_scale = 1.0f;
    section_end();

    section("vf: dir negates the angle handed to the SVPWM");
    alg_vf_init(&vf);
    alg_vf_start(&vf, 10.0f);
    vf.f_hz = 10.0f;
    vf.dir = 0u;
    {
        float a0 = alg_vf_step(&vf, 0.00005f);
        T(a0 >= 0.0f && a0 < 6.2831854f);
    }
    vf.dir = 1u;
    {
        float a1 = alg_vf_step(&vf, 0.00005f);
        T(a1 <= 0.0f && a1 > -6.2831854f);
    }
    section_end();

    section("vf: stop zeroes the frequency for the next ramp");
    alg_vf_start(&vf, 80.0f);
    alg_vf_stop(&vf);
    T(vf.on == 0u);
    T(vf.f_hz == 0.0f);
    T(vf.f_target == 0.0f);
    section_end();
}

/* ================================================================== */
/* Application state machine                                           */
/* ================================================================== */
static void test_app(void)
{
    force_current_ready();

    section("sys: ART accelerator is enabled at boot");
    T((fake_flash_regs.ACR & FLASH_ACR_PRFTEN) != 0u);
    T((fake_flash_regs.ACR & FLASH_ACR_ICEN) != 0u);
    T((fake_flash_regs.ACR & FLASH_ACR_DCEN) == 0u);   /* left off on purpose */
    section_end();

    section("app: align needs the bridge running");
    {
        app_stop();
        T(app_align_start() == 2u);          /* IDLE: rejected */
        T(app_status()->state != APP_ST_ALIGN);
    }
    section_end();

    section("app: align measures the offset by sweeping the field");
    {
        uint16_t k;
        uint32_t t_ms = 0u;
        const float off_elec = 49.043f;   /* encoder zero offset, elec deg */

        mid_enc_zero_set(0.0f);          /* start from a known zero */
        app_set_mode(1u);
        app_start();

        fake_adc2_regs.JDR1 = 2048u;
        fake_adc2_regs.JDR2 = 2048u;
        fake_adc2_regs.JDR3 = 2048u;
        fake_tim1_regs.CNT = 3700u;

        enc_build_frame(0u, 0u);
        HAL_ADCEx_InjectedConvCpltCallback(&hadc2);
        app_loop();

        T(app_align_start() == 0u);
        T(app_status()->state == APP_ST_ALIGN);

        /* Simulate a rotor that follows the field with the encoder zero
         * sitting off_elec degrees away from the true d-axis. */
        for (k = 0u; k < 1440u; k++) {
            float field = 0.0f;
            float enc_elec;
            float mech;
            uint16_t d;

            t_ms += 10u;
            fake_tick_add(10u);

            if (t_ms < 200u) {
                field = 0.0f;
            } else if (t_ms < 7200u) {
                /* forward: 2 Hz for 7 s = 14 electrical revs */
                field = 6.2831853f * 2.0f * ((float)(t_ms - 200u) / 1000.0f);
            } else if (t_ms < 14200u) {
                float ts = (float)(t_ms - 7200u) / 1000.0f;
                field = 6.2831853f * (14.0f - (2.0f * ts));
            } else {
                field = 0.0f;
            }

            enc_elec = field + (off_elec * 0.0174532925199433f);
            mech = enc_elec * (180.0f / 3.14159265358979f) / 7.0f;
            while (mech >= 360.0f) { mech -= 360.0f; }
            while (mech < 0.0f)    { mech += 360.0f; }
            d = (uint16_t)(mech / 360.0f * 16384.0f);

            enc_build_frame(d, 0u);
            HAL_ADCEx_InjectedConvCpltCallback(&hadc2);
            app_loop();
        }

        T(app_align_info()->done == 1u);
        T(app_align_info()->fail == 0u);
        T(app_status()->state == APP_ST_IDLE);
        T(app_align_info()->n > 400u);
        /* the offset (49.04 deg elec) must become a 7.0 deg mechanical shift */
        T(fabsf(app_align_info()->err_deg - off_elec) < 2.0f);
        T(fabsf(mid_enc_info()->zero_deg - 7.006f) < 0.5f);
    }
    section_end();

    section("app: align aborts when the encoder never answers");
    {
        uint16_t k;

        app_set_mode(1u);
        app_start();
        fake_ssi_fail = 1u;
        enc_build_frame(0u, 0u);
        HAL_ADCEx_InjectedConvCpltCallback(&hadc2);
        app_loop();

        T(app_align_start() == 0u);
        for (k = 0u; k < 60u; k++) {
            fake_tick_add(10u);
            HAL_ADCEx_InjectedConvCpltCallback(&hadc2);
            app_loop();
        }
        T(app_align_info()->done == 1u);
        T(app_align_info()->fail == 3u);
        T(app_status()->state == APP_ST_IDLE);
        fake_ssi_fail = 0u;
    }
    section_end();
    section("app: mode switch gating");
    (void)app_stop();
    T(app_set_mode(0u) == 0u);
    T(app_set_mode(1u) == 0u);
    T(app_set_mode(2u) == 0u);
    T(app_set_mode(3u) == 1u);                 /* out of range */
    T(app_status()->mode == CTRL_FOC);
    section_end();

    section("app: start requires a usable mode");
    (void)app_stop();
    (void)app_set_mode(0u);                    /* CTRL_IDLE */
    T(app_start() == 2u);                      /* rejected: no mode */
    T(app_status()->state == APP_ST_IDLE);
    (void)app_set_mode(2u);                    /* CTRL_FOC: S6 is in */
    T(app_start() == 0u);
    T(app_status()->state == APP_ST_RUN);
    T(bsp_pwm_is_enabled() == 1u);
    T(app_foc_info()->id_ref == 0.0f);         /* starts with zero current */
    T(app_foc_info()->iq_ref == 0.0f);
    (void)app_stop();
    T(bsp_pwm_is_enabled() == 0u);
    (void)app_set_mode(1u);                    /* CTRL_OPENLOOP */
    T(app_start() == 0u);
    T(app_status()->state == APP_ST_RUN);
    T(bsp_pwm_is_enabled() == 1u);             /* bridge enabled on start */
    section_end();

    section("app: no mode change while running");
    T(app_status()->state == APP_ST_RUN);
    T(app_set_mode(1u) == 2u);                 /* rejected */
    T(app_status()->mode == CTRL_OPENLOOP);
    T(app_stop() == 0u);
    T(bsp_pwm_is_enabled() == 0u);             /* bridge off after stop */
    T(app_status()->state == APP_ST_IDLE);
    T(app_set_mode(1u) == 0u);                 /* now allowed */
    T(app_status()->mode == CTRL_OPENLOOP);
    section_end();

    section("app: wave rate clamping (FireWater max)");
    app_wave_set(1u, 3000u);
    T(app_status()->wave_rate_hz == DBG_WAVE_RATE_MAX);   /* clamped to max */
    app_wave_set(1u, 0u);
    T(app_status()->wave_rate_hz == 1u);                  /* clamped to min */
    app_wave_set(0u, 250u);
    T(app_status()->wave_rate_hz == 250u);
    T(app_status()->wave_on == 0u);
    section_end();

    section("app: frame pacing at max rate");
    {
        uint32_t t0;
        int i;
        rx_clear();
        app_cmd_init();
        tx_clear();
        app_wave_set(1u, DBG_WAVE_RATE_MAX);
        app_counters_clear();
        t0 = fake_tick_get();
        for (i = 0; i < 1000; i++) {
            fake_tick_set(t0 + (uint32_t)(i + 1));
            app_loop();
        }
        pump_all();              /* flush the last queued frame */
        app_wave_set(0u, DBG_WAVE_RATE_MAX);
        /* period = 1000/MAX ms -> frames in 1000 ms = 1000/period */
        T(app_status()->wave_frames == (1000u / (1000u / DBG_WAVE_RATE_MAX)));
        T(app_status()->wave_drop == 0u);
        T(tx_contains("0.000,"));
        T(tx_contains(","));
        if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    }
    section_end();
}

/* ================================================================== */
/* Command parser                                                      */
/* ================================================================== */
static void test_cmd(void)
{
    rx_clear();
    app_cmd_init();
    force_current_ready();

    section("cmd: help");
    tx_clear();
    cmd_feed("help\r\n");
    T(tx_contains("---- commands ----"));
    T(tx_contains("mode <0|1|2>"));
    T(tx_contains("pwm <d>"));
    T(tx_contains("wave <on [hz]|off>"));
    if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    section_end();

    section("cmd: ver reports the real hardware numbers");
    tx_clear();
    cmd_feed("ver\n");
    T(tx_contains("Rs = 25 mOhm"));
    T(tx_contains("645 uA/LSB"));
    T(tx_contains("+-1320 mA"));
    T(tx_contains("div = 7"));
    T(tx_contains("5640 uV/LSB"));
    T(tx_contains("max = 23100 mV"));
    T(tx_contains("Ts = 50 us"));
    T(tx_contains("ARR = 4249"));
    {   /* build id must be reported so the acceptance test can prove the
           board image matches the source */
        char want[24];
        snprintf(want, sizeof(want), "#%u", (unsigned)FW_BUILD_ID);
        T(tx_contains(want));
    }
    if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    section_end();

    section("cmd: stat");
    tx_clear();
    cmd_feed("stat\r\n");
    T(tx_contains("state    = "));
    T(tx_contains("mode     = "));
    T(tx_contains("uart     = rx_drop="));
    if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    section_end();

    section("cmd: mode / start / stop flow");
    tx_clear();
    cmd_feed("mode 0\r\n");
    T(tx_contains("OK mode=IDLE"));
    tx_clear();
    cmd_feed("start\r\n");
    T(tx_contains("ERR mode=IDLE"));
    tx_clear();
    cmd_feed("mode 2\r\n");
    T(tx_contains("OK mode=FOC"));
    tx_clear();
    cmd_feed("start\r\n");
    T(tx_contains("OK start, mode=FOC"));
    T(app_status()->state == APP_ST_RUN);
    tx_clear();
    cmd_feed("stop\r\n");
    tx_clear();
    cmd_feed("mode 1\r\n");
    T(tx_contains("OK mode=OPENLOOP"));
    tx_clear();
    cmd_feed("start\r\n");
    T(tx_contains("OK start, mode=OPENLOOP"));
    T(app_status()->state == APP_ST_RUN);
    tx_clear();
    cmd_feed("mode 2\r\n");
    T(tx_contains("ERR stop the motor first"));
    tx_clear();
    cmd_feed("stop\r\n");
    T(tx_contains("OK stop, PWM off"));
    T(app_status()->state == APP_ST_IDLE);
    section_end();

    section("cmd: pwm needs a running bridge");
    tx_clear();
    cmd_feed("pwm 0.5\r\n");
    T(tx_contains("ERR start first"));
    tx_clear();
    cmd_feed("pwm\r\n");
    T(tx_contains("pwm = OFF"));
    section_end();

    section("cmd: pwm refused while calibrating");
    mid_current_restart_cal();                 /* ready = 0 */
    cmd_feed("mode 1\r\n");
    cmd_feed("start\r\n");
    tx_clear();
    cmd_feed("pwm 0.5\r\n");
    T(tx_contains("ERR calibrating"));
    force_current_ready();
    tx_clear();
    cmd_feed("pwm 0.5\r\n");
    T(tx_contains("OK duty = 0.50/0.50/0.50"));
    cmd_feed("stop\r\n");
    section_end();

    section("cmd: pwm sets and clamps duty");
    cmd_feed("mode 1\r\n");
    cmd_feed("start\r\n");
    tx_clear();
    cmd_feed("pwm 0.25\r\n");
    T(tx_contains("OK duty = 0.25/0.25/0.25"));
    T(fake_tim1_regs.CCR1 == 3188u);
    T(fake_tim1_regs.CCR2 == 3188u);
    T(fake_tim1_regs.CCR3 == 3188u);
    tx_clear();
    cmd_feed("pwm 0.1 0.5 0.9\r\n");
    T(tx_contains("OK duty = 0.10/0.50/0.90"));
    T(fake_tim1_regs.CCR1 == 3825u);
    T(fake_tim1_regs.CCR2 == 2125u);
    T(fake_tim1_regs.CCR3 == 425u);
    tx_clear();
    cmd_feed("pwm 2\r\n");
    T(tx_contains("OK duty = 0.99/0.99/0.99"));     /* clamped, echoes applied */
    T(fake_tim1_regs.CCR1 == PWM_CCR_MIN);
    tx_clear();
    cmd_feed("pwm abc\r\n");
    T(tx_contains("ERR bad duty"));
    tx_clear();
    cmd_feed("pwm 0.1 0.2\r\n");
    T(tx_contains("ERR usage=pwm"));
    tx_clear();
    cmd_feed("stop\r\n");
    section_end();

    section("cmd: bad arguments");
    tx_clear();
    cmd_feed("mode\r\n");
    T(tx_contains("ERR usage=mode <0|1|2>"));
    tx_clear();
    cmd_feed("mode 9\r\n");
    T(tx_contains("ERR mode must be 0/1/2"));
    tx_clear();
    cmd_feed("mode abc\r\n");
    T(tx_contains("ERR bad arg"));
    tx_clear();
    cmd_feed("rate\r\n");
    T(tx_contains("ERR usage=rate <hz>"));
    tx_clear();
    cmd_feed("led\r\n");
    T(tx_contains("ERR usage=led"));
    section_end();

    section("cmd: vf / acc / dir flow");
    cmd_feed("stop\r\n");
    tx_clear();
    cmd_feed("vf 50\r\n");
    T(tx_contains("ERR start first"));
    cmd_feed("mode 1\r\n");
    cmd_feed("start\r\n");
    tx_clear();
    cmd_feed("vf 50\r\n");
    T(tx_contains("OK vf target = 50.00 Hz"));
    tx_clear();
    cmd_feed("acc 30\r\n");
    T(tx_contains("OK acc = 30.00 Hz/s"));
    tx_clear();
    cmd_feed("dir 1\r\n");
    T(tx_contains("OK dir = 1"));
    tx_clear();
    cmd_feed("dir 5\r\n");
    T(tx_contains("ERR dir must be 0 or 1"));
    tx_clear();
    cmd_feed("pwm 0.5\r\n");
    T(tx_contains("ERR vf active"));
    tx_clear();
    cmd_feed("vf off\r\n");
    T(tx_contains("OK vf off"));
    tx_clear();
    cmd_feed("pwm 0.5\r\n");
    T(tx_contains("OK duty = 0.50/0.50/0.50"));
    cmd_feed("dir 0\r\n");
    cmd_feed("stop\r\n");
    section_end();

    section("cmd: cur and prof");
    tx_clear();
    cmd_feed("cur\r\n");
    T(tx_contains("cur = raw "));
    T(tx_contains("adc = frames="));
    tx_clear();
    cmd_feed("prof\r\n");
    T(tx_contains("isr"));
    T(tx_contains("adc_read"));
    T(tx_contains("loop"));
    tx_clear();
    cmd_feed("prof reset\r\n");
    T(tx_contains("OK prof reset"));
    section_end();

    section("cmd: vf 0 behaves like vf off");
    cmd_feed("mode 1\r\nstart\r\n");
    tx_clear();
    cmd_feed("vf 0\r\n");
    T(tx_contains("OK vf 0 = off"));
    T(app_vf_info()->on == 0u);
    T(mid_current_get()->ready == 1u);
    section_end();
    section("cmd: phase report");
    tx_clear();
    cmd_feed("phase\r\n");
    T(tx_contains("phase, stage us"));
    T(tx_contains("isr cnt = "));
    T(tx_contains(" wr "));
    if (g_fails != g_fail_mark) { tx_dump_on_fail(); }
    tx_clear();
    cmd_feed("phase reset\r\n");
    T(tx_contains("OK phase reset"));
    section_end();

    section("cmd: trig benchmark");
    tx_clear();
    cmd_feed("trig\r\n");
    T(tx_contains("trig = "));
    T(tx_contains("cycles/call"));
    section_end();

    section("pi: proportional, integral and anti-windup");
    {
        alg_pi_t pi;
        float out;
        int16_t k;

        alg_pi_init(&pi, 2.0f, 100.0f, -5.0f, 5.0f);
        out = alg_pi_step(&pi, 0.1f, 0.00005f);
        T(fabsf(out - 0.2f) < 0.02f);

        alg_pi_reset(&pi);
        for (k = 0; k < 5000; k++) {
            out = alg_pi_step(&pi, 1.0f, 0.00005f);
        }
        T(out <= 5.0f);
        T(out > 4.9f);
        T(pi.integ <= 5.001f);

        for (k = 0; k < 200; k++) {
            out = alg_pi_step(&pi, -1.0f, 0.00005f);
        }
        T(out < 5.0f);
    }
    section_end();

    section("foc: clarke and park match the textbook formulas");
    {
        float a;
        float b;
        float d;
        float q;

        alg_clarke(1.0f, -0.5f, &a, &b);
        T(fabsf(a - 1.0f) < 1e-5f);
        T(fabsf(b) < 1e-5f);

        alg_park(a, b, 0.0f, 1.0f, &d, &q);
        T(fabsf(d - 1.0f) < 1e-5f);
        T(fabsf(q) < 1e-5f);

        alg_park(1.0f, 0.0f, 1.0f, 0.0f, &d, &q);
        T(fabsf(d) < 1e-5f);
        T(fabsf(q + 1.0f) < 1e-5f);

        {
            float c = 0.6f;
            float sn = 0.8f;
            float a2;
            float b2;

            alg_park(0.3f, -0.7f, sn, c, &d, &q);
            alg_inv_park(d, q, sn, c, &a2, &b2);
            T(fabsf(a2 - 0.3f) < 1e-5f);
            T(fabsf(b2 + 0.7f) < 1e-5f);
        }
    }
    section_end();

    section("foc: svpwm alpha-beta form equals the polar form");
    {
        uint16_t k;
        float d1[3];
        float d2[3];
        float worst = 0.0f;

        for (k = 0u; k < 360u; k++) {
            float th = (float)k * 0.0174532925199433f;
            float c;
            float sn;
            uint8_t i;

            mid_trig_cos_sin(th, &c, &sn);
            alg_svpwm(th, 0.7f, d1);
            alg_svpwm_ab(0.7f * c, 0.7f * sn, d2);

            for (i = 0u; i < 3u; i++) {
                float e = fabsf(d1[i] - d2[i]);
                if (e > worst) { worst = e; }
            }
        }
        T(worst < 1e-4f);
    }
    section_end();

    section("foc: loop saturates inside the voltage circle");
    {
        alg_foc_t f;
        float duty[3];
        float mag;
        uint16_t k;

        alg_foc_init(&f, 12.0f, 0.95f, 1.57f, 7227.0f, 5.7f);

        for (k = 0u; k < 20000u; k++) {
            (void)alg_foc_step(&f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                               0.5f, 0.0f, 0.00005f, duty);
        }

        mag = sqrtf((f.vd * f.vd) + (f.vq * f.vq));
        T(mag <= 5.71f);
        T(mag > 5.0f);                   /* it did saturate */
        /* the reported modulation index must also respect the limit */
        T(f.m <= 0.951f);
        T(duty[0] >= 0.0f && duty[0] <= 1.0f);
        T(duty[1] >= 0.0f && duty[1] <= 1.0f);
        T(duty[2] >= 0.0f && duty[2] <= 1.0f);
        T(fabsf(f.vq) < 0.5f);           /* iq ref was 0 */
    }
    section_end();

    section("foc: circle limit keeps the vector angle");
    {
        alg_foc_t f;
        float duty[3];
        float mag;

        alg_foc_init(&f, 12.0f, 0.95f, 50.0f, 0.0f, 5.7f);
        (void)alg_foc_step(&f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f,
                           0.00005f, duty);
        mag = sqrtf((f.vd * f.vd) + (f.vq * f.vq));
        T(mag <= 5.71f);
        T(fabsf(f.vd - f.vq) < 0.01f);
    }
    section_end();

    section("cmd: id/iq need FOC mode and start");
    {
        tx_clear();
        cmd_feed("mode 1\r\n");
        cmd_feed("start\r\n");
        tx_clear();
        cmd_feed("id 0.3\r\n");
        T(tx_contains("ERR mode 2 first"));

        cmd_feed("stop\r\n");
        cmd_feed("mode 2\r\n");
        tx_clear();
        cmd_feed("id 0.3\r\n");
        T(tx_contains("ERR start first"));

        cmd_feed("start\r\n");
        tx_clear();
        cmd_feed("id 0.30\r\n");
        T(tx_contains("OK id=0.30 A, iq=0.00 A"));
        T(fabsf(app_foc_info()->id_ref - 0.30f) < 1e-4f);

        tx_clear();
        cmd_feed("iq -0.50\r\n");
        T(tx_contains("OK id=0.30 A, iq=-0.50 A"));
        T(fabsf(app_foc_info()->iq_ref + 0.50f) < 1e-4f);

        /* the reference vector must stay inside the ADC range */
        tx_clear();
        cmd_feed("id 1.30\r\n");
        T(tx_contains("ERR current too high"));

        tx_clear();
        cmd_feed("foc\r\n");
        T(tx_contains("foc idle") || tx_contains("foc id="));

        cmd_feed("stop\r\n");
        cmd_feed("mode 0\r\n");
    }
    section_end();

    section("cmd: pi survives start/stop and rejects out of range");
    {
        float kp;
        float ki;

        tx_clear();
        cmd_feed("mode 2\r\n");
        tx_clear();
        cmd_feed("pi 2.50 9000\r\n");
        T(tx_contains("OK pi = 2.50/9000"));

        /* starting must NOT throw the tuned gains away */
        cmd_feed("start\r\n");
        app_foc_get_pi(&kp, &ki);
        T(fabsf(kp - 2.50f) < 1e-3f);
        T(fabsf(ki - 9000.0f) < 1.0f);

        /* and stop must not either */
        cmd_feed("stop\r\n");
        app_foc_get_pi(&kp, &ki);
        T(fabsf(kp - 2.50f) < 1e-3f);

        tx_clear();
        cmd_feed("pi 99 100\r\n");
        T(tx_contains("ERR pi out of range"));

        tx_clear();
        cmd_feed("pi def\r\n");
        T(tx_contains("default"));
        app_foc_get_pi(&kp, &ki);
        T(fabsf(kp - FOC_KP) < 1e-3f);
        T(fabsf(ki - FOC_KI) < 1.0f);

        cmd_feed("mode 0\r\n");
    }
    section_end();

    section("spd: feedforward and kick shape the output");
    {
        alg_spd_t sp;
        float out;
        uint16_t k;

        alg_spd_init(&sp, 0.05f, 0.5f, 1.2f, 0.015f, 8e-5f, 0.15f, 50.0f);
        sp.on = 1u;

        /* target zero -> zero output, no kick */
        sp.spd_ref = 0.0f;
        T(alg_spd_step(&sp, 0.0f, 0.001f) == 0.0f);

        /* rotor stopped with a non-zero target -> at least the breakaway
         * current, otherwise the cogging detent would hold it forever */
        sp.spd_ref = 300.0f;
        out = alg_spd_step(&sp, 0.0f, 0.001f);
        T(out >= 0.15f);

        /* once it turns (above kick_rpm) the kick is gone: PI + feedforward */
        alg_spd_reset(&sp);
        sp.on = 1u;
        sp.spd_ref = 300.0f;
        out = alg_spd_step(&sp, 300.0f, 0.001f);
        T(out > 0.0f);
        T(out < 0.15f);

        /* negative target with the rotor stopped -> negative kick */
        alg_spd_reset(&sp);
        sp.on = 1u;
        sp.spd_ref = -300.0f;
        out = alg_spd_step(&sp, 0.0f, 0.001f);
        T(out <= -0.15f);

        /* the output never exceeds the current limit */
        alg_spd_reset(&sp);
        sp.on = 1u;
        sp.spd_ref = 2000.0f;
        {
            uint8_t inside = 1u;

            for (k = 0u; k < 2000u; k++) {
                out = alg_spd_step(&sp, 0.0f, 0.001f);
                if ((out > 1.201f) || (out < -1.201f)) { inside = 0u; }
            }
            T(inside == 1u);           /* never left the current limit */
        }
        T(out > 1.0f);                 /* it did saturate high */
    }
    section_end();

    section("spd: closed loop on an inertia + friction model");
    {
        alg_spd_t sp;
        float w = 0.0f;                /* rad/s mechanical          */
        float iq;
        float rpm;
        uint16_t n;

        /* Kt = 0.0289 Nm/A; J = 8e-5 kg m2 back-calculated from the S7.0
         * measurement (0.12 A ramps the rotor to ~400 rpm in ~1 s);
         * viscous 2.8e-5 Nm/(rad/s); coulomb 8.7e-4 Nm (~0.03 A). */
        const float kt = 0.0289f;
        const float j  = 8e-5f;
        const float bv = 2.8e-5f;
        const float tc = 8.7e-4f;

        alg_spd_init(&sp, 0.05f, 0.5f, 1.2f, 0.015f, 8e-5f, 0.15f, 50.0f);
        sp.on = 1u;
        sp.spd_ref = 300.0f;

        for (n = 0u; n < 8000u; n++) {          /* 8 s at 1 kHz */
            rpm = w * 9.5492966f;               /* rad/s -> rpm  */
            iq  = alg_spd_step(&sp, rpm, 0.001f);

            {
                float tq = kt * iq;
                float fr = bv * w;

                if ((w > 1e-4f) || (tq > tc)) {
                    fr += tc;                   /* kinetic friction */
                } else if ((w < -1e-4f) || (tq < -tc)) {
                    fr -= tc;
                }
                if (tq > fr) {
                    w += (tq - fr) / j * 0.001f;
                } else if (tq < fr) {
                    w += (tq + fr) / j * 0.001f;
                    if (w < 0.0f) { w = 0.0f; }
                }
                if (w < 0.0f) { w = 0.0f; }
            }

            if (w > 200.0f) { w = 200.0f; }     /* keep the test bounded */
        }

        /* It must have accelerated from standstill and the error must be
         * small at the end of the run. */
        printf("      [spd] final rpm=%.1f iq=%.3f\n", (double)(w * 9.5492966f), (double)iq);
        T(w > 5.0f);
        T(fabsf((w * 9.5492966f) - 300.0f) < 60.0f);
    }
    section_end();

    section("foc: feedforward injects the back-EMF term");
    {
        alg_foc_t f;
        float duty[3];

        /* no PI at all: the output is the feedforward only */
        alg_foc_init(&f, 12.0f, 0.95f, 0.0f, 0.0f, 5.7f);
        alg_foc_set_ff(&f, 0.004f, 0.0005f, 1.0f);

        /* stopped: no feedforward */
        (void)alg_foc_step(&f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.00005f, duty);
        T(fabsf(f.vq) < 1e-6f);

        /* 1000 rad/s electrical -> vq = 0.004 * 1000 = 4.0 V */
        (void)alg_foc_step(&f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, 0.0f, 0.00005f, duty);
        T(fabsf(f.vq - 4.0f) < 0.02f);
        T(fabsf(f.vd) < 0.02f);

        /* reversed rotation flips the sign */
        (void)alg_foc_step(&f, 0.0f, 0.0f, 0.0f, 0.0f, -1000.0f, 0.0f, 0.0f, 0.00005f, duty);
        T(fabsf(f.vq + 4.0f) < 0.02f);

        /* gain 0 disables everything */
        alg_foc_set_ff(&f, 0.004f, 0.0005f, 0.0f);
        (void)alg_foc_step(&f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, 0.0f, 0.00005f, duty);
        T(fabsf(f.vq) < 1e-6f);
    }
    section_end();

    section("foc: feedforward cancels the coupling term");
    {
        alg_foc_t f;
        float duty[3];

        alg_foc_init(&f, 12.0f, 0.95f, 0.0f, 0.0f, 5.7f);
        alg_foc_set_ff(&f, 0.0f, 0.001f, 1.0f);   /* only the L coupling */

        /* theta = 90 deg, so ia = 0.3 gives iq = -0.3 and id = 0 */
        (void)alg_foc_step(&f, 0.3f, -0.15f, -0.15f, 1.5707963f,
                           1000.0f, 0.0f, 0.0f, 0.00005f, duty);
        T(fabsf(f.iq + 0.3f) < 0.02f);
        /* vd = -w*L*iq = -1000*0.001*(-0.3) = +0.3 V */
        T(fabsf(f.vd - 0.3f) < 0.02f);
    }
    section_end();

    section("cmd: spd needs FOC + start, and spi tunes the gains");
    {
        tx_clear();
        cmd_feed("mode 1\r\n");
        cmd_feed("start\r\n");
        tx_clear();
        cmd_feed("spd 300\r\n");
        T(tx_contains("ERR mode 2 first"));

        cmd_feed("stop\r\n");
        cmd_feed("mode 2\r\n");
        tx_clear();
        cmd_feed("spd 300\r\n");
        T(tx_contains("ERR start first"));

        cmd_feed("start\r\n");
        tx_clear();
        cmd_feed("spd 300\r\n");
        T(tx_contains("OK spd target = 300 rpm"));

        tx_clear();
        cmd_feed("spd\r\n");
        /* the ramp has not moved yet in the host test, so only the target is set */
        T(tx_contains("(target 300)"));

        tx_clear();
        cmd_feed("spd 9999\r\n");
        T(tx_contains("ERR speed too high"));

        tx_clear();
        cmd_feed("spd off\r\n");
        T(tx_contains("OK spd off"));

        tx_clear();
        cmd_feed("spi 0.10 1.0\r\n");
        T(tx_contains("OK spi = 0.10/1.00"));

        tx_clear();
        cmd_feed("spi\r\n");
        T(tx_contains("spi = kp 100.00 mA/rpm"));

        tx_clear();
        cmd_feed("spi 99 1\r\n");
        T(tx_contains("ERR spi out of range"));

        cmd_feed("stop\r\n");
        cmd_feed("mode 0\r\n");
    }
    section_end();

    section("cmd: prot and fault report the protection state");
    {
        tx_clear();
        cmd_feed("prot\r\n");
        T(tx_contains("prot os"));

        tx_clear();
        cmd_feed("prot os 800\r\n");
        T(tx_contains("OK prot os = 800.00"));
        T(fabsf(app_prot_info()->overspeed_rpm - 800.0f) < 0.01f);

        tx_clear();
        cmd_feed("prot os 99999\r\n");
        T(tx_contains("ERR prot value out of range"));

        tx_clear();
        cmd_feed("prot zz 5\r\n");
        T(tx_contains("ERR usage=prot"));

        /* restore the default so later tests are unaffected */
        cmd_feed("prot os 2000\r\n");

        tx_clear();
        cmd_feed("fault\r\n");
        T(tx_contains("fault none"));
    }
    section_end();

    section("app: a fault shuts the bridge down and can be cleared");
    {
        uint8_t i;

        cmd_feed("mode 2\r\n");
        cmd_feed("start\r\n");
        T(app_status()->state == APP_ST_RUN);

        app_fault_raise((uint8_t)APP_FAULT_OVERSPEED, 2100.0f);
        T(app_status()->state == APP_ST_FAULT);
        T(bsp_pwm_is_enabled() == 0u);
        T(app_fault_info()->code == (uint8_t)APP_FAULT_OVERSPEED);
        T(fabsf(app_fault_info()->value - 2100.0f) < 0.5f);
        T(app_fault_info()->count[APP_FAULT_OVERSPEED] >= 1u);

        /* start is refused while the fault is latched */
        T(app_start() == 1u);

        /* stop clears it */
        cmd_feed("stop\r\n");
        T(app_status()->state == APP_ST_IDLE);
        T(app_start() == 0u);
        cmd_feed("stop\r\n");
        cmd_feed("mode 0\r\n");
        for (i = 0u; i < 1u; i++) { }
    }
    section_end();

    section("cmd: unknown command");
    tx_clear();
    cmd_feed("foobar\r\n");
    T(tx_contains("ERR unknown cmd 'foobar'"));
    T(tx_contains("try 'help'"));
    section_end();

    section("cmd: led control and echo");
    tx_clear();
    cmd_feed("led on\r\n");
    T(tx_contains("OK led ON"));
    T(bsp_led_is_on() == 1u);
    tx_clear();
    cmd_feed("led off\r\n");
    T(tx_contains("OK led OFF"));
    T(bsp_led_is_on() == 0u);
    tx_clear();
    cmd_feed("led toggle\r\n");
    T(tx_contains("OK led ON"));
    tx_clear();
    cmd_feed("echo hello\r\n");
    T(tx_is("[log] hello\r\n"));
    section_end();

    section("cmd: backspace editing");
    tx_clear();
    cmd_feed("stax\x08t\r\n");        /* backspace turns stax into stat */
    T(tx_contains("---- status ----"));
    section_end();

    section("cmd: over-long line rejected");
    {
        char big[100];
        int i;
        for (i = 0; i < 90; i++) { big[i] = 'x'; }
        big[90] = '\0';
        tx_clear();
        cmd_feed(big);
        cmd_feed("\r\n");
        T(tx_contains("ERR line too long"));
    }
    section_end();

    section("alg_obs: tracks a synthetic constant-speed motor");
    {
        /* Pretend the plant is v = e (no current), with
         *   e = w*psi*(-sin th, cos th)
         * and check that the observer recovers theta and w. */
        alg_obs_t ob;
        const float psi = 0.00413f;
        const float ts  = 50.0e-6f;
        const float we  = 440.0f;      /* elec rad/s ~= 600 rpm */
        float th = 0.5f;               /* deliberately not 0 */
        int k;
        float de;

        alg_obs_init(&ob, 2.3f, 0.0005f, 1500.0f, 80.0f, 4000.0f);
        for (k = 0; k < 4000; k++) {   /* 200 ms */
            float ea = -(we * psi) * sinf(th);
            float eb =  (we * psi) * cosf(th);

            alg_obs_step(&ob, ea, eb, 0.0f, 0.0f, ts);
            th += we * ts;
            while (th >= 6.2831853f) { th -= 6.2831853f; }
        }
        de = ob.theta - th;
        while (de >  3.14159265f) { de -= 6.283185307179586f; }
        while (de < -3.14159265f) { de += 6.283185307179586f; }
        T(fabsf(de) < 0.05f);          /* < 2.9 deg */
        T(fabsf(ob.omega_e - we) < (0.05f * we));
        /* eq carries the amplitude, ~= w*psi */
        T(fabsf(ob.eq - (we * psi)) < (0.15f * we * psi));
    }
    section_end();

    section("alg_obs: converges from a 1 rad initial error");
    {
        alg_obs_t ob;
        const float psi = 0.00413f;
        const float ts  = 50.0e-6f;
        const float we  = 220.0f;
        float th = 1.0f;
        int k;
        float de;

        alg_obs_init(&ob, 2.3f, 0.0005f, 1500.0f, 80.0f, 4000.0f);
        alg_obs_reset(&ob, 0.0f);
        for (k = 0; k < 8000; k++) {   /* 400 ms */
            float ea = -(we * psi) * sinf(th);
            float eb =  (we * psi) * cosf(th);

            alg_obs_step(&ob, ea, eb, 0.0f, 0.0f, ts);
            th += we * ts;
        }
        de = ob.theta - th;
        while (de >  3.14159265f) { de -= 6.283185307179586f; }
        while (de < -3.14159265f) { de += 6.283185307179586f; }
        T(fabsf(de) < 0.10f);          /* < 5.7 deg after 400 ms */
    }
    section_end();

    section("app: align ol arms the open-loop pre-position");
    {
        T(app_align_ol_start() == 2u);        /* not running yet */
        cmd_feed("mode 2\r\n");
        cmd_feed("start\r\n");
        T(app_status()->state == APP_ST_RUN);
        T(app_align_ol_start() == 0u);
        T(app_status()->state == APP_ST_ALIGN);
        T(app_align_ol_start() == 3u);        /* already aligning */
        cmd_feed("stop\r\n");
        T(app_status()->state == APP_ST_IDLE);
        cmd_feed("mode 0\r\n");
    }
    section_end();
    section("enc: pll_clear drops a free-running speed estimate");
    {
        /* A type-2 PLL cannot pull back once the true speed is outside its
         * capture range: it keeps reporting the old speed (measured on
         * hardware: 2000 rpm clamp for ~6 s after "stop"). The app calls
         * mid_enc_pll_clear() whenever the bridge is disabled, so the stale
         * estimate cannot survive the transition. */
        extern DWT_TypeDef fake_dwt;
        uint32_t t  = 1000000u;
        uint16_t raw = 0u;
        int i;

        fake_ssi_fail = 0u;
        mid_enc_init();
        mid_enc_set_spd_src(1u);          /* PLL as the source */

        /* spin at ~1460 rpm: 20 codes per 50 us update */
        for (i = 0; i < 3000; i++) {
            t += 8500u;                   /* 50 us at 170 MHz */
            fake_dwt.CYCCNT = t;
            raw = (uint16_t)(raw + 20u);
            enc_build_frame(raw, 0u);
            mid_enc_update();
        }
        T(fabsf(mid_enc_pll_rpm()) > 800.0f);

        mid_enc_pll_clear();              /* what app_stop() does */
        T(fabsf(mid_enc_info()->spd_rpm) < 1.0f);
        T(fabsf(mid_enc_pll_rpm()) < 1.0f);

        /* the rotor is stationary and the estimate must stay at zero */
        for (i = 0; i < 3000; i++) {
            t += 8500u;
            fake_dwt.CYCCNT = t;
            enc_build_frame(raw, 0u);
            mid_enc_update();
        }
        T(fabsf(mid_enc_pll_rpm()) < 50.0f);
    }
    section_end();

    section("enc: tracking is untouched while the motor turns");
    {
        extern DWT_TypeDef fake_dwt;
        uint32_t t  = 2000000u;
        uint16_t raw = 0u;
        int i;

        fake_ssi_fail = 0u;
        mid_enc_init();
        mid_enc_set_spd_src(1u);

        for (i = 0; i < 4000; i++) {      /* ~200 ms of real motion */
            t += 8500u;
            fake_dwt.CYCCNT = t;
            raw = (uint16_t)(raw + 20u);
            enc_build_frame(raw, 0u);
            mid_enc_update();
        }
        T(fabsf(mid_enc_pll_rpm()) > 800.0f);
        T(fabsf(mid_enc_info()->spd_rpm - mid_enc_pll_rpm()) < 1.0f);
    }
    section_end();
    section("alg_if: frequency and amplitude ramps");
    {
        alg_if_t v;
        float ts = 1.0f / 20000.0f;
        int i;

        alg_if_init(&v, 30.0f, 0.4f, 2.0f);
        alg_if_start(&v);
        alg_if_set_target(&v, 35.0f);        /* ~300 rpm at 7 pole pairs */

        for (i = 0; i < 2000; i++) {          /* 100 ms */
            alg_if_step(&v, ts);
        }
        /* 30 Hz/s for 0.1 s = +3 Hz */
        T(fabsf(v.f_hz - 5.0f) < 0.2f);
        /* amplitude 0 -> 0.4 A in 100 ms */
        T(fabsf(v.i_now - 0.4f) < 0.05f);

        for (i = 0; i < 20000; i++) {         /* 1 s more */
            alg_if_step(&v, ts);
        }
        T(fabsf(v.f_hz - 35.0f) < 0.1f);      /* reached the target */
        T(v.theta >= 0.0f && v.theta < 6.2832f);

        alg_if_stop(&v);
        T(v.run == 0u);
        T(fabsf(v.i_now) < 0.01f);
    }
    section_end();

    section("alg_if: the angle advances at the commanded frequency");
    {
        alg_if_t v;
        float ts = 1.0f / 20000.0f;
        float th0;
        int i;

        alg_if_init(&v, 1000.0f, 0.4f, 10.0f);  /* fast ramp */
        alg_if_start(&v);
        alg_if_set_target(&v, 10.0f);           /* already there */
        for (i = 0; i < 200; i++) { alg_if_step(&v, ts); }

        th0 = v.theta;
        for (i = 0; i < 2000; i++) { alg_if_step(&v, ts); }   /* 100 ms */
        /* 10 Hz for 0.1 s = 1 electrical revolution */
        {
            float d = v.theta - th0;

            while (d < -3.14159265f) { d += 6.283185307179586f; }
            while (d >  3.14159265f) { d -= 6.283185307179586f; }
            T(fabsf(d) < 0.2f);
        }
    }
    section_end();

    section("alg_if: soft start (slow amplitude ramp + d -> q crossfade)");
    {
        alg_if_t v;
        float ts = 1.0f / 20000.0f;
        float id;
        float iq;
        int i;

        /* S10.8 / #166: the App's settings. Before this the amplitude reached
         * 0.6 A in 100 ms and then hopped 90 deg onto q in one 50 us frame,
         * which is 0 -> 17 mN.m instantly = the visible kick. */
        alg_if_init(&v, 30.0f, 0.6f, 2.0f);
        alg_if_set_ramp_time(&v, 0.30f);
        alg_if_set_xfade(&v, 0.08f);
        alg_if_set_align_time(&v, 0.60f);
        alg_if_start(&v);
        T(v.phase == 0u);

        /* Half way into the amplitude ramp the current is still moderate. */
        for (i = 0; i < 3000; i++) { alg_if_step(&v, ts); }     /* 150 ms */
        T(v.i_now > 0.22f);
        T(v.i_now < 0.38f);
        alg_if_apply(&v, &id, &iq);
        T(fabsf(iq) < 1e-6f);                    /* still purely on d */
        T(fabsf(id - v.i_now) < 1e-6f);

        /* End of the pre-align: full amplitude, crossfade under way. */
        for (i = 0; i < 10000; i++) { alg_if_step(&v, ts); }    /* 650 ms */
        T(v.phase == 1u);
        T(fabsf(v.i_now - 0.6f) < 0.01f);
        T(v.xq > 0.0f);
        T(v.xq < 1.0f);
        alg_if_apply(&v, &id, &iq);
        T(iq > 0.0f);
        T(iq < v.i_now);           /* torque ramps: no step */
        T(fabsf((id + iq) - v.i_now) < 1e-6f);

        /* Crossfade finished: everything on q. */
        for (i = 0; i < 2000; i++) { alg_if_step(&v, ts); }     /* +100 ms */
        alg_if_apply(&v, &id, &iq);
        T(fabsf(v.xq - 1.0f) < 1e-6f);
        T(fabsf(iq - v.i_now) < 1e-6f);
        T(fabsf(id) < 1e-6f);

        /* The old behaviour (xfade off) must still snap to q immediately, so
         * the change is opt-in for callers that want it. */
        alg_if_init(&v, 30.0f, 0.6f, 2.0f);
        alg_if_start(&v);
        for (i = 0; i < 200; i++) { alg_if_step(&v, ts); }
        T(fabsf(v.xq - 1.0f) < 1e-6f);
    }
    section_end();

    section("spd: preload makes the hand-over transfer bumpless");
    {
        alg_spd_t sp;
        float u;
        int i;
        uint8_t ok = 1u;

        alg_spd_init(&sp, 0.05f, 0.5f, 1.2f, 0.015f, 8e-5f, 0.15f, 50.0f);
        sp.on = 1u;

        /* Hand-over: rotor already at the drag speed, the drag was making
         * 0.17 A of torque current. */
        sp.spd_ref = 310.0f;
        alg_spd_preload(&sp, 0.17f);

        for (i = 0; i < 20; i++) {
            u = alg_spd_step(&sp, 310.0f, 0.001f);
            if ((u < 0.16f) || (u > 0.18f)) { ok = 0u; }
        }
        T(ok == 1u);               /* stays at the drag torque, no jump */

        /* Without the preload the same situation brakes hard: reference just
         * left zero (the App's 600 rpm/s ramp), rotor already at 310 rpm. */
        alg_spd_reset(&sp);
        sp.on      = 1u;
        sp.spd_ref = 10.0f;
        sp.spd_f   = 310.0f;
        u = alg_spd_step(&sp, 310.0f, 0.001f);
        T(u < -1.0f);              /* that is the kick the preload removes */
    }
    section_end();

    section("spd: accel feedforward is clamped to the ramp rate");
    {
        alg_spd_t sp;
        float u1, u2;

        alg_spd_init(&sp, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 30.0f);
        sp.ff_acc   = 3.11e-3f;
        sp.acc_max  = 62.83f;      /* 600 rpm/s in rad/s^2 */
        sp.on       = 1u;
        sp.spd_ref  = 200.0f;
        sp.ref_prev = 200.0f;

        /* One ramp step per call: 0.6 rpm in 1 ms. */
        sp.spd_ref = 200.6f;
        u1 = alg_spd_step(&sp, 200.6f, 0.001f);

        /* A call that catches two steps must not double the feedforward
         * (the on-board +0.20 A spike every 40 ms). */
        sp.spd_ref = 201.8f;
        u2 = alg_spd_step(&sp, 201.8f, 0.001f);

        T(u1 > 0.15f && u1 < 0.25f);
        T(fabsf(u2 - u1) < 0.02f);

        /* Without the clamp the same call does double. */
        sp.acc_max  = 0.0f;
        sp.ref_prev = 200.6f;
        sp.spd_ref  = 201.8f;
        T(alg_spd_step(&sp, 201.8f, 0.001f) > (1.5f * u1));
    }
    section_end();

    section("foc: the PI keeps head-room for the feed-forward");
    {
        alg_foc_t f;
        float duty[3];
        float mag;
        uint16_t k;

        alg_foc_init(&f, 12.0f, 0.95f, 3.0f, 11000.0f, 5.7f);
        alg_foc_set_ff(&f, 0.00413f, 0.0005f, 1.0f);

        /* 709 rad/s electrical backwards: the ff is about -2.93 V. #169 kept
         * the PI clamped at +-5.7 V and added the ff afterwards, so the most
         * the loop could ever apply was 2.77 V - the machine free-ran. */
        (void)alg_foc_step(&f, 0.0f, 0.0f, 0.0f, 0.0f, -709.0f,
                           0.0f, 1.16f, 0.00005f, duty);
        T(f.pi_q.out_max > 8.0f);            /* 5.7 + 2.93 */
        T(f.pi_q.out_max < 9.0f);
        T(f.pi_q.out_min < -8.0f);
        T(f.pi_d.out_max < 6.0f);            /* id does not couple at iq = 0 */

        /* the vector circle is still the hard limit */
        for (k = 0u; k < 20000u; k++) {
            (void)alg_foc_step(&f, 0.0f, 0.0f, 0.0f, 0.0f, -709.0f,
                               0.0f, 1.16f, 0.00005f, duty);
        }
        mag = sqrtf((f.vd * f.vd) + (f.vq * f.vq));
        T(mag <= 5.71f);
        T(mag > 5.0f);                       /* it really did saturate now */
    }
    section_end();

    section("alg_if: a reverse request starts in reverse");
    {
        alg_if_t v;

        alg_if_init(&v, 30.0f, 0.6f, 2.0f);
        alg_if_set_align_time(&v, 0.0f);
        alg_if_start(&v);
        T(v.f_hz > 1.9f);
        alg_if_set_target(&v, -46.7f);       /* 400 rpm backwards */
        T(v.f_hz < -1.9f);                   /* #169: it stayed +2 Hz */
        T(v.f_target < -46.0f);

        /* once it is ramping, a new target must not snap anything */
        alg_if_step(&v, 0.05f);
        alg_if_step(&v, 0.05f);
        {
            float before = v.f_hz;

            alg_if_set_target(&v, -20.0f);
            T(v.f_hz == before);
        }
    }
    section_end();

    section("alg_ident: R from two points cancels the offset");
{
    /* A single point carries the inverter drop: 0.3 A -> 2.60 ohm and 0.6 A
     * -> 2.45 ohm on the reference motor. The slope must give the 2.3 ohm
     * the magnet wire actually has. */
    float r1 = alg_ident_r(0.30f * 2.60f, 0.30f, 0.60f * 2.45f, 0.60f);
    float r2 = alg_ident_r(0.780f, 0.30f, 1.470f, 0.60f);

    T(fabsf(r1 - 2.30f) < 0.01f);
    T(fabsf(r2 - 2.30f) < 0.01f);
    /* identical currents: refuse instead of dividing by ~0 */
    T(alg_ident_r(1.0f, 0.50f, 1.0f, 0.51f) == 0.0f);
}

section("alg_ident: L from the vd - R*id = L*di/dt fit");
{
    alg_ident_l_t acc;
    float l = 0.0f;
    float id = 0.0f;
    int   k;

    /* Synthesize the exact law for L = 0.5 mH and R = 2.3 ohm at 20 kHz,
     * then make sure the fit recovers the inductance. */
    alg_ident_l_reset(&acc);
    for (k = 0; k < 20; k++) {
        /* Keep vd well inside the 5.7 V budget: the fit now ignores frames
         * that sit against the limit (that is the whole point of #175). */
        float didt = 300.0f;               /* A/s */
        float vd   = (2.3f * id) + (0.0005f * didt);
        alg_ident_l_add(&acc, vd, id, didt, 2.3f, 5.7f);
        id += didt * 0.00005f;
    }
    T(acc.n == 20u);
    T(alg_ident_l_result(&acc, &l) == 0u);
    T(fabsf(l - 0.0005f) < 1e-6f);

    /* A frame where the current did not move contributes nothing. */
    alg_ident_l_reset(&acc);
    alg_ident_l_add(&acc, 1.0f, 0.0f, 0.0f, 2.3f, 5.7f);
    T(alg_ident_l_result(&acc, &l) == 1u);

    /* A negative slope can only be noise: say so rather than returning a
     * negative inductance. */
    alg_ident_l_reset(&acc);
    for (k = 0; k < 5; k++) {
        alg_ident_l_add(&acc, 0.0f, 0.0f, -3000.0f, 2.3f, 5.7f);
    }
    T(alg_ident_l_result(&acc, &l) == 2u);

    /* A frame against the voltage limit says nothing about L: the drive was
     * not able to apply what the loop asked for, so it must not be counted. */
    alg_ident_l_reset(&acc);
    for (k = 0; k < 20; k++) {
        alg_ident_l_add(&acc, 5.65f, 0.10f, 3000.0f, 2.3f, 5.7f);
    }
    T(acc.n == 0u);
    T(alg_ident_l_result(&acc, &l) == 1u);

    /* Settled frames (current not moving) add nothing either. */
    alg_ident_l_reset(&acc);
    for (k = 0; k < 20; k++) {
        alg_ident_l_add(&acc, 0.46f, 0.20f, 5.0f, 2.3f, 5.7f);
    }
    T(acc.n == 0u);
}

section("alg_ident: KE from the drag");
{
    /* 0.00413 V per elec rad/s is what the motor does: at 60 Hz electrical
     * (377 rad/s) the back-EMF is 1.56 V. */
    T(fabsf(alg_ident_ke(1.557f, 377.0f) - 0.00413f) < 1e-5f);
    T(fabsf(alg_ident_ke(1.557f, -377.0f) - 0.00413f) < 1e-5f);  /* signed w */
    T(alg_ident_ke(1.557f, 0.5f) == 0.0f);                        /* too slow */
}

section("alg_ident: Z = R + jwL out of the injection coefficients");
{
    const float w = 6.2831853f * 2000.0f;
    const float a = 0.1f;                 /* amps at the injection */
    const float vc = a * w * 0.0005f;     /* L part, on cosine     */
    const float vs = a * 2.3f;            /* R part, on sine       */
    float l_h = 0.0f;
    float r_o = 0.0f;

    T(alg_ident_z(vc, vs, 0.0f, a, w, &l_h, &r_o) == 0u);
    T(fabsf(l_h - 0.0005f) < 1e-6f);
    T(fabsf(r_o - 2.3f) < 1e-3f);
    /* No current: nothing can be divided out. */
    T(alg_ident_z(vc, vs, 0.0f, 0.0f, w, &l_h, &r_o) == 1u);
    /* A negative inductance means the phase convention broke. */
    T(alg_ident_z(-vc, vs, 0.0f, a, w, &l_h, &r_o) == 3u);
}

section("cfg: motor parameter golden values");
    {
        /* Deliberate snapshots. If a refactor changes one of these the
         * test fails, so the change has to be reviewed on purpose: the
         * S10.0 refactor silently broke the acceleration feedforward
         * (used J/KE_elec instead of J/Kt, i.e. 7x too large) while the
         * firmware still built with 0 errors and 0 warnings. */
        T(ENC_POLE_PAIRS == 7u);
        T(FOC_K_EMF > 0.00269f && FOC_K_EMF < 0.00271f);
        T(FOC_L_H > 0.00092f && FOC_L_H < 0.00094f);
        T(FOC_KP > 2.99f && FOC_KP < 3.01f);
        T(FOC_KI > 10999.0f && FOC_KI < 11001.0f);
        T(SPD_KP > 0.0059f && SPD_KP < 0.0061f);
        T(SPD_KI > 0.079f && SPD_KI < 0.081f);
        T(FOC_I_MAX_A > 1.19f && FOC_I_MAX_A < 1.21f);
        T(FOC_I_TRIP_A > 1.27f && FOC_I_TRIP_A < 1.29f);
        T(SPD_RPM_MAX > 1999.0f && SPD_RPM_MAX < 2001.0f);
        /* Kt(SI) = KE per mechanical rad/s = KE_elec * pole pairs */
        T(MOTOR_KT_NM_A > 0.0188f && MOTOR_KT_NM_A < 0.0190f);
        /* the value that bit us: must stay near J/Kt, never J/KE_elec */
        T(SPD_FF_ACC > 0.0047f && SPD_FF_ACC < 0.0048f);
    }
    section_end();

    section("alg_pll: locks onto a constant speed");
    {
        alg_pll_t p;
        float th = 0.0f;
        float w  = 2.0f * 3.14159265f * 10.0f;   /* 10 Hz elec */
        int i;

        alg_pll_init(&p, 100.0f, 0.707f, 2000.0f);
        alg_pll_reset(&p, 0.0f);
        for (i = 0; i < 4000; i++) {             /* 200 ms at 20 kHz */
            th += w * (1.0f / 20000.0f);
            while (th >= 6.2831853f) { th -= 6.2831853f; }
            alg_pll_step(&p, th, 1.0f / 20000.0f);
        }
        T(fabsf(p.omega - w) < (0.02f * w));
        T(p.locked == 1u);
    }
    section_end();

    section("alg_pll: crosses the 2pi wrap without a glitch");
    {
        alg_pll_t p;
        float th = 6.0f;
        float w  = 2.0f * 3.14159265f * 20.0f;
        float worst = 0.0f;
        int i;

        alg_pll_init(&p, 100.0f, 0.707f, 2000.0f);
        alg_pll_reset(&p, th);
        for (i = 0; i < 4000; i++) {
            float e;

            th += w * (1.0f / 20000.0f);
            while (th >= 6.2831853f) { th -= 6.2831853f; }
            alg_pll_step(&p, th, 1.0f / 20000.0f);
            e = fabsf(p.omega - w);
            if (i > 2000) { if (e > worst) { worst = e; } }
        }
        T(worst < (0.03f * w));
    }
    section_end();

    section("alg_pll: clamps a runaway speed");
    {
        alg_pll_t p;
        float th = 0.0f;
        int i;

        alg_pll_init(&p, 100.0f, 0.707f, 50.0f);
        alg_pll_reset(&p, 0.0f);
        for (i = 0; i < 2000; i++) {
            th += 3.0f * (1.0f / 20000.0f);   /* very fast angle ramp */
            while (th >= 6.2831853f) { th -= 6.2831853f; }
            alg_pll_step(&p, th, 1.0f / 20000.0f);
        }
        T(p.omega <= 50.01f);
    }
    section_end();

    section("pll: command switches the speed source");
    {
        tx_clear();
        cmd_feed("pll on\r\n");
        T(tx_contains("OK pll speed source"));
        T(mid_enc_spd_src() == 1u);

        tx_clear();
        cmd_feed("pll off\r\n");
        T(tx_contains("OK window speed source"));
        T(mid_enc_spd_src() == 0u);

        tx_clear();
        cmd_feed("pll wn 200\r\n");
        T(tx_contains("OK pll wn = 200.00 Hz"));

        tx_clear();
        cmd_feed("pll zeta 1.0\r\n");
        T(tx_contains("OK pll zeta = 1.00"));

        tx_clear();
        cmd_feed("pll\r\n");
        T(tx_contains("pll src window"));

        /* restore defaults for the sections that follow */
        cmd_feed("pll wn 100\r\n");
        cmd_feed("pll zeta 0.707\r\n");
        cmd_feed("pll off\r\n");
    }
    section_end();

    section("cmd: empty line ignored");
    tx_clear();
    cmd_feed("\r\n");
    T(fake_tx_len() == 0u);
    section_end();

    section("cmd: several commands in one burst");
    tx_clear();
    cmd_feed("echo a\r\necho b\r\necho c\r\n");
    T(tx_contains("a\r\n"));
    T(tx_contains("b\r\n"));
    T(tx_contains("c\r\n"));
    section_end();
}

/* ================================================================== */
static void banner_reset(void)
{
    /* app_init() prints a boot banner that is still sitting in the firmware
     * TX ring. Flush it into the capture buffer first, then clear both. */
    pump_all();
    fake_tx_reset();
}

int main(void)
{
    printf("\n===== myfoc host tests =====\n");

    fake_tick_reset();
    app_init();                /* inits gpio/uart/cmd and prints the banner */
    banner_reset();

    printf("\n[ring buffer]\n");     test_ringbuf();
    printf("\n[log formatter]\n");   test_log();
    printf("\n[vofa firewater]\n");  test_vofa();
    printf("\n[uart driver]\n");     test_uart();
    printf("\n[gpio led/key]\n");    test_gpio();
    printf("\n[pwm]\n");             test_pwm();
    printf("\n[current]\n");         test_current();
    printf("\n[profiler]\n");        test_prof();
    printf("\n[adc]\n");             test_adc();
    printf("\n[svpwm]\n");           test_svpwm();
    printf("\n[trig]\n");            test_trig();
    printf("\n[enc]\n");             test_enc();
    printf("\n[v/f]\n");             test_vf();
    printf("\n[app state machine]\n"); test_app();
    printf("\n[command parser]\n");  test_cmd();

    printf("\n============================\n");
    printf("checks: %d   failed: %d\n", g_checks, g_fails);
    printf("RESULT: %s\n", (g_fails == 0) ? "PASS" : "FAIL");
    return (g_fails == 0) ? 0 : 1;
}
