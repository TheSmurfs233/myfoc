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

    section("vofa: rejects 0 and >16 channels");
    tx_clear();
    {
        float ch[20];
        T(vofa_send(ch, 0u) == 0u);
        T(vofa_send(ch, 17u) == 0u);
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

    bsp_prof_enable(1u);
}

/* ================================================================== */
/* Application state machine                                           */
/* ================================================================== */
static void test_app(void)
{
    force_current_ready();

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
    (void)app_set_mode(2u);                    /* CTRL_FOC: no loop yet */
    T(app_start() == 3u);                      /* rejected until S6 */
    T(app_status()->state == APP_ST_IDLE);
    T(bsp_pwm_is_enabled() == 0u);             /* bridge still off */
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
    T(tx_contains("wave <on|off>"));
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
    T(tx_contains("ERR mode=FOC not ready"));
    T(app_status()->state == APP_ST_IDLE);
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
    printf("\n[app state machine]\n"); test_app();
    printf("\n[command parser]\n");  test_cmd();

    printf("\n============================\n");
    printf("checks: %d   failed: %d\n", g_checks, g_fails);
    printf("RESULT: %s\n", (g_fails == 0) ? "PASS" : "FAIL");
    return (g_fails == 0) ? 0 : 1;
}
