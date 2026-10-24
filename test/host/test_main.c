/**
  ******************************************************************************
  * @file    test_main.c
  * @brief   Host unit tests for the pure-logic firmware modules.
  *
  *   Covered:
  *     - mid_ringbuf : basic, wrap-around, random fuzz vs reference model
  *     - mid_log     : all format specifiers, width/padding, truncation
  *     - mid_vofa    : byte-exact JustFloat frames
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

#include "fake_api.h"
#include "board_cfg.h"
#include "mid_ringbuf.h"
#include "mid_log.h"
#include "mid_vofa.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "app_main.h"
#include "app_cmd.h"

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
static void rx_clear(void)
{
    uint8_t sink[64];
    int guard = 0;
    while ((bsp_uart_rx_avail() > 0u) && (guard++ < 100000)) {
        (void)bsp_uart_rx_read(sink, (uint16_t)sizeof(sink));
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
/* VOFA+ JustFloat                                                     */
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

    section("vofa: FireWater 6 channels, named and newline terminated");
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
/* Application state machine                                           */
/* ================================================================== */
static void test_app(void)
{
    section("app: mode switch gating");
    (void)app_stop();
    T(app_set_mode(0u) == 0u);
    T(app_set_mode(1u) == 0u);
    T(app_set_mode(2u) == 0u);
    T(app_set_mode(3u) == 1u);                 /* out of range */
    T(app_status()->mode == CTRL_FOC);
    section_end();

    section("app: start requires a mode");
    (void)app_stop();
    (void)app_set_mode(0u);                    /* CTRL_IDLE */
    T(app_start() == 2u);                      /* rejected */
    T(app_status()->state == APP_ST_IDLE);
    (void)app_set_mode(2u);
    T(app_start() == 0u);
    T(app_status()->state == APP_ST_RUN);
    section_end();

    section("app: no mode change while running");
    T(app_status()->state == APP_ST_RUN);
    T(app_set_mode(1u) == 2u);                 /* rejected */
    T(app_status()->mode == CTRL_FOC);
    T(app_stop() == 0u);
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

    section("cmd: help");
    tx_clear();
    cmd_feed("help\r\n");
    T(tx_contains("---- commands ----"));
    T(tx_contains("mode <0|1|2>"));
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
    T(tx_contains("OK start, mode=FOC"));
    T(app_status()->state == APP_ST_RUN);
    tx_clear();
    cmd_feed("mode 1\r\n");
    T(tx_contains("ERR stop the motor first"));
    tx_clear();
    cmd_feed("stop\r\n");
    T(tx_contains("OK stop, PWM off"));
    T(app_status()->state == APP_ST_IDLE);
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
    printf("\n[vofa justfloat]\n");  test_vofa();
    printf("\n[uart driver]\n");     test_uart();
    printf("\n[gpio led/key]\n");    test_gpio();
    printf("\n[app state machine]\n"); test_app();
    printf("\n[command parser]\n");  test_cmd();

    printf("\n============================\n");
    printf("checks: %d   failed: %d\n", g_checks, g_fails);
    printf("RESULT: %s\n", (g_fails == 0) ? "PASS" : "FAIL");
    return (g_fails == 0) ? 0 : 1;
}
