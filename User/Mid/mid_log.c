/**
  ******************************************************************************
  * @file    mid_log.c
  * @brief   Lightweight formatted output implementation.
  *
  * Every non-empty line is emitted with the "[log] " marker in front:
  *
  *     [log] uptime = 12345 ms
  *
  * Why: the waveform rides on the same UART in FireWater CSV format
  * (e.g. "169.347,1.000,0.000"). VOFA+ consumes any line that parses as CSV
  * into the plot and never shows it as text. The "[log] " prefix guarantees a
  * log line can never be mistaken for data, so the receive pane only ever
  * shows real log output - no waveform rows flooding it.
  ******************************************************************************
  */

#include "mid_log.h"
#include "bsp_uart.h"
#include <stdarg.h>
#include <string.h>

#define LOG_BUF_SIZE    128u    /* max length of one line, truncated if longer */

#define LOG_PREFIX      "[log] "
#define LOG_PREFIX_LEN  6u

/* Static buffer: no stack usage, safe because this is main-loop only */
static char s_log_buf[LOG_BUF_SIZE];

/* ------------------------------------------------------------------ */
/* unsigned integer to string (returns length, no terminator)          */
/* ------------------------------------------------------------------ */
static uint16_t fmt_uint(char *dst, uint32_t v, uint8_t base, uint8_t upper)
{
    static const char s_lower[] = "0123456789abcdef";
    static const char s_upper[] = "0123456789ABCDEF";
    const char *dig = upper ? s_upper : s_lower;
    char tmp[12];
    uint8_t i = 0u;
    uint8_t j;

    if (v == 0u) {
        tmp[i++] = '0';
    } else {
        while ((v != 0u) && (i < (uint8_t)sizeof(tmp))) {
            tmp[i++] = dig[v % base];
            v /= base;
        }
    }
    for (j = 0u; j < i; j++) {
        dst[j] = tmp[(uint8_t)(i - 1u - j)];
    }
    return (uint16_t)i;
}

/* ------------------------------------------------------------------ */
/* signed decimal                                                      */
/* ------------------------------------------------------------------ */
static uint16_t fmt_int(char *dst, int32_t v, uint8_t base)
{
    uint16_t n = 0u;
    uint32_t uv;

    if ((base == 10u) && (v < 0)) {
        dst[n++] = '-';
        /* Handle INT32_MIN without overflow */
        uv = (uint32_t)(-(v + 1)) + 1u;
    } else {
        uv = (uint32_t)v;
    }
    n = (uint16_t)(n + fmt_uint(&dst[n], uv, base, 0u));
    return n;
}

/* ------------------------------------------------------------------ */
/* float with fixed 2 decimals                                         */
/* ------------------------------------------------------------------ */
static uint16_t fmt_float2(char *dst, float v)
{
    uint16_t n = 0u;
    int32_t  ip;
    uint32_t frac;
    uint8_t  neg = 0u;

    if (v != v) { v = 0.0f; }                 /* NaN guard */
    if (v < -999999.0f) { v = -999999.0f; }
    if (v >  999999.0f) { v =  999999.0f; }

    if (v < 0.0f) { neg = 1u; v = -v; }

    ip   = (int32_t)v;
    frac = (uint32_t)((v - (float)ip) * 100.0f + 0.5f);
    if (frac >= 100u) { frac = 99u; }   /* guard against rounding carry */

    if (neg) { dst[n++] = '-'; }
    n = (uint16_t)(n + fmt_uint(&dst[n], (uint32_t)ip, 10u, 0u));
    dst[n++] = '.';
    if (frac < 10u) { dst[n++] = '0'; }
    n = (uint16_t)(n + fmt_uint(&dst[n], frac, 10u, 0u));
    return n;
}

/* ------------------------------------------------------------------ */
/* append a field honouring width / zero padding / left alignment      */
/* ------------------------------------------------------------------ */
static uint16_t append_field(char *out, uint16_t n, uint16_t cap,
                             const char *s, uint16_t slen,
                             uint8_t width, uint8_t zero_pad, uint8_t left)
{
    uint16_t pad = (width > slen) ? (uint16_t)(width - slen) : 0u;
    uint16_t i;

    if (!left) {
        for (i = 0u; (i < pad) && (n < cap); i++) {
            out[n++] = zero_pad ? '0' : ' ';
        }
    }
    for (i = 0u; (i < slen) && (n < cap); i++) {
        out[n++] = s[i];
    }
    if (left) {
        for (i = 0u; (i < pad) && (n < cap); i++) {
            out[n++] = ' ';
        }
    }
    return n;
}

/* ------------------------------------------------------------------ */
/* main entry                                                          */
/* ------------------------------------------------------------------ */
void log_printf(const char *fmt, ...)
{
    char        num[24];
    uint16_t    n = LOG_PREFIX_LEN;                     /* reserve the marker */
    const uint16_t cap = (uint16_t)(LOG_BUF_SIZE - 2u); /* room for CRLF */
    va_list     ap;
    uint8_t     i;
    uint16_t    k;

    if (fmt == NULL) { return; }
    va_start(ap, fmt);

    while ((*fmt != '\0') && (n < cap)) {
        char c = *fmt++;

        if (c != '%') {
            s_log_buf[n++] = c;
            continue;
        }

        {
            uint8_t  zero_pad = 0u;
            uint8_t  left     = 0u;
            uint8_t  width    = 0u;
            uint16_t slen     = 0u;
            const char *strarg = NULL;

            if (*fmt == '-') { left = 1u; fmt++; }
            if (*fmt == '0') { zero_pad = 1u; fmt++; }
            while ((*fmt >= '0') && (*fmt <= '9')) {
                width = (uint8_t)(width * 10u + (uint8_t)(*fmt - '0'));
                fmt++;
            }
            /* Precision is parsed but not supported (see header note) */
            if (*fmt == '.') {
                fmt++;
                while ((*fmt >= '0') && (*fmt <= '9')) { fmt++; }
            }
            /* Skip length modifiers l / h */
            while ((*fmt == 'l') || (*fmt == 'h')) { fmt++; }

            if (*fmt == '\0') { break; }
            c = *fmt++;

            switch (c) {
            case 'd':
            case 'i': {
                int32_t v = (int32_t)va_arg(ap, int);
                slen = fmt_int(num, v, 10u);
                n = append_field(s_log_buf, n, cap, num, slen, width, zero_pad, left);
                break;
            }
            case 'u': {
                uint32_t v = (uint32_t)va_arg(ap, unsigned int);
                slen = fmt_uint(num, v, 10u, 0u);
                n = append_field(s_log_buf, n, cap, num, slen, width, zero_pad, left);
                break;
            }
            case 'x':
            case 'X': {
                uint32_t v = (uint32_t)va_arg(ap, unsigned int);
                slen = fmt_uint(num, v, 16u, (c == 'X') ? 1u : 0u);
                n = append_field(s_log_buf, n, cap, num, slen, width, zero_pad, left);
                break;
            }
            case 'f': {
                /* float is promoted to double in varargs, must read as double */
                float v = (float)va_arg(ap, double);
                slen = fmt_float2(num, v);
                n = append_field(s_log_buf, n, cap, num, slen, width, zero_pad, left);
                break;
            }
            case 's': {
                strarg = va_arg(ap, const char *);
                if (strarg == NULL) { strarg = "(null)"; }
                slen = (uint16_t)strlen(strarg);
                n = append_field(s_log_buf, n, cap, strarg, slen, width, zero_pad, left);
                break;
            }
            case 'c': {
                num[0] = (char)va_arg(ap, int);
                n = append_field(s_log_buf, n, cap, num, 1u, width, zero_pad, left);
                break;
            }
            case '%': {
                s_log_buf[n++] = '%';
                break;
            }
            default: {
                /* Unknown specifier: emit it verbatim */
                s_log_buf[n++] = '%';
                if (n < cap) { s_log_buf[n++] = c; }
                break;
            }
            }
        }
    }

    va_end(ap);

    if (n == LOG_PREFIX_LEN) {
        /* Empty format: emit a bare blank line without the marker */
        s_log_buf[0] = '\r';
        s_log_buf[1] = '\n';
        n = 2u;
    } else {
        /* Prepend the "[log] " marker now that we know the line is not empty */
        for (i = 0u; i < LOG_PREFIX_LEN; i++) { s_log_buf[i] = LOG_PREFIX[i]; }
        /* Never let a log line look like FireWater data: ':' -> ';' */
        for (k = LOG_PREFIX_LEN; k < n; k++) {
            if (s_log_buf[k] == ':') { s_log_buf[k] = ';'; }
        }
        if (n < (uint16_t)(cap + 1u)) { s_log_buf[n++] = '\r'; }
        if (n < (uint16_t)(cap + 2u)) { s_log_buf[n++] = '\n'; }
    }

    /* All or nothing: a truncated line would leave the CSV stream broken. */
    if (bsp_uart_tx_space() < n) { return; }
    (void)bsp_uart_tx_write((const uint8_t *)s_log_buf, n);
}

void log_str(const char *s)
{
    if (s == NULL) { return; }
    (void)bsp_uart_tx_write((const uint8_t *)s, (uint16_t)strlen(s));
}

void log_crlf(void)
{
    static const uint8_t crlf[2] = { '\r', '\n' };
    (void)bsp_uart_tx_write(crlf, 2u);
}
