/**
  ******************************************************************************
  * @file    mid_enc.c
  * @brief   MT6701 angle decoding (see mid_enc.h).
  *
  * Layer: Mid.
  *
  * SSI frame, 24 bits MSB first:
  *   bits 23..10  D[13:0]   angle, 0..16383 -> 0..360 deg
  *   bits  9..6   Mg[3:0]   magnetic status (0 = normal)
  *   bits  5..0   CRC[5:0]  x^6 + x + 1 over the 18 bits above
  *
  * Electrical angle = (mech - zero) * pole_pairs * direction, wrapped to
  * [0, 2pi). The zero offset is captured with "enc zero" and stored in its own
  * flash page so it does not disturb the current-calibration record.
  ******************************************************************************
  */

#include "mid_enc.h"
#include <math.h>
#include "alg_pll.h"
#include "board_cfg.h"
#include "bsp_flash.h"
#include "bsp_prof.h"
#include "bsp_ssi.h"

#define ENC_TWO_PI      6.283185307179586f
#define ENC_RAD_PER_LSB (ENC_TWO_PI / 16384.0f)
#define ENC_RAD_TO_DEG  (180.0f / 3.14159265358979f)

typedef struct {
    uint32_t magic;
    uint32_t version;
    float    zero_rad;
    uint32_t dir;
    uint16_t crc;            /* CRC16-CCITT over the bytes above */
    uint16_t pad;
} enc_record_t;

/* Encoder subsystem switch: a sensorless build runs with no encoder
 * at all ("enc off"). */
static uint8_t s_enc_enabled = 1u;

static mid_enc_t s_enc;
/* 1 = a frame with a plausible angle and a good CRC has been seen since
 * boot. Used to tell "no encoder fitted" (all-zero frames) from "the
 * encoder died" so the former can fall back instead of faulting. */
static uint8_t   s_raw_seen_ok;
static float    s_zero_rad;   /* zero_deg converted once */

/* Speed estimator state (see ENC_SPD_WINDOW_MS) */
static uint32_t  s_pll_clears;      /* mid_enc_pll_clear() calls  */
static alg_pll_t s_pll;          /* S8.1 speed observer */
static uint8_t   s_spd_src;      /* 0 window, 1 PLL */
static float     s_win_rpm;      /* window estimator, for comparison */
static uint32_t  s_t_pll;        /* PLL sample timestamp */
static uint8_t  s_have_prev;
static uint32_t s_t_prev;
static float    s_prev_mech;
static float    s_prev_elec;
static float    s_acc_mech;
static float    s_acc_elec;
static uint32_t s_acc_cyc;

/* ------------------------------------------------------------------ */
/* CRC16-CCITT (0x1021, init 0xFFFF), same as the calibration record   */
/* ------------------------------------------------------------------ */
static uint16_t enc_crc16(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint16_t       crc = 0xFFFFu;
    uint32_t       i;
    uint8_t        b;

    for (i = 0u; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)p[i] << 8);
        for (b = 0u; b < 8u; b++) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* CRC-6 is fed three 6-bit chunks; because the polynomial degree equals the
 * chunk width, one 64-entry table does the whole 18-bit frame in 3 lookups
 * (the bitwise loop cost ~2.4us per call, 30% of the whole encoder cost).
 * The table was cross-checked against 15 real frames captured from the chip. */
static const uint8_t s_crc6_tab[64] = {
    0x00u, 0x03u, 0x06u, 0x05u, 0x0Cu, 0x0Fu, 0x0Au, 0x09u,
    0x18u, 0x1Bu, 0x1Eu, 0x1Du, 0x14u, 0x17u, 0x12u, 0x11u,
    0x30u, 0x33u, 0x36u, 0x35u, 0x3Cu, 0x3Fu, 0x3Au, 0x39u,
    0x28u, 0x2Bu, 0x2Eu, 0x2Du, 0x24u, 0x27u, 0x22u, 0x21u,
    0x23u, 0x20u, 0x25u, 0x26u, 0x2Fu, 0x2Cu, 0x29u, 0x2Au,
    0x3Bu, 0x38u, 0x3Du, 0x3Eu, 0x37u, 0x34u, 0x31u, 0x32u,
    0x13u, 0x10u, 0x15u, 0x16u, 0x1Fu, 0x1Cu, 0x19u, 0x1Au,
    0x0Bu, 0x08u, 0x0Du, 0x0Eu, 0x07u, 0x04u, 0x01u, 0x02u,
};

uint8_t mid_enc_crc6(uint32_t bits18)
{
    uint8_t crc;

    crc = s_crc6_tab[(bits18 >> 12) & 0x3Fu];
    crc = s_crc6_tab[crc ^ ((bits18 >> 6) & 0x3Fu)];
    crc = s_crc6_tab[crc ^ (bits18 & 0x3Fu)];
    return crc;
}

/* ------------------------------------------------------------------ */
/* Zero / direction persistence                                        */
/* ------------------------------------------------------------------ */
static void enc_save(void)
{
    enc_record_t rec;

    rec.magic    = ENC_FLASH_MAGIC;
    rec.version  = ENC_FLASH_VERSION;
    rec.zero_rad = s_enc.zero_deg / ENC_RAD_TO_DEG;
    rec.dir      = (uint32_t)s_enc.dir;
    rec.pad      = 0u;
    rec.crc      = enc_crc16(&rec, (uint32_t)(sizeof(rec) - sizeof(rec.crc)
                                              - sizeof(rec.pad)));

    (void)bsp_flash_erase_page(ENC_FLASH_ADDR);
    (void)bsp_flash_program(ENC_FLASH_ADDR, &rec, (uint32_t)sizeof(rec));
}

static void enc_load(void)
{
    enc_record_t rec;

    bsp_flash_read(ENC_FLASH_ADDR, &rec, (uint32_t)sizeof(rec));

    if ((rec.magic == ENC_FLASH_MAGIC) &&
        (rec.version == ENC_FLASH_VERSION) &&
        (rec.crc == enc_crc16(&rec, (uint32_t)(sizeof(rec) - sizeof(rec.crc)
                                               - sizeof(rec.pad)))) &&
        (rec.zero_rad >= 0.0f) && (rec.zero_rad < ENC_TWO_PI) &&
        (rec.dir <= 1u)) {
        s_enc.zero_deg = rec.zero_rad * ENC_RAD_TO_DEG;
        s_zero_rad     = rec.zero_rad;
        s_enc.dir      = (uint8_t)rec.dir;
        s_enc.src      = ENC_SRC_FLASH;
    } else {
        s_enc.zero_deg = 0.0f;
        s_enc.dir      = ENC_DIR_DEFAULT;
        s_enc.src      = ENC_SRC_NONE;
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
void mid_enc_init(void)
{
    s_enc.raw      = 0u;
    s_enc.mg       = 0u;
    s_enc.crc_ok   = 0u;
    s_enc.valid    = 0u;
    s_enc.mech_deg = 0.0f;
    s_enc.elec_deg = 0.0f;
    s_enc.frames   = 0u;
    s_enc.crc_errs = 0u;
    s_enc.io_errs  = 0u;
    s_enc.late     = 0u;
    s_enc.ovr      = 0u;
    s_enc.spd_hz   = 0.0f;
    s_enc.spd_rpm  = 0.0f;

    s_have_prev = 0u;
    s_acc_mech  = 0.0f;
    s_acc_elec  = 0.0f;
    s_acc_cyc   = 0u;

    bsp_ssi_init();
    enc_load();


    /* S8.1: speed observer. The angle fed in is the electrical one, so the
     * PLL output is already in electrical rad/s. */
    alg_pll_init(&s_pll, PLL_WN_HZ, PLL_ZETA,
                 ENC_TWO_PI * (SPD_RPM_MAX / 60.0f) * (float)ENC_POLE_PAIRS);
    s_spd_src = ENC_SPD_DEFAULT;
    s_win_rpm = 0.0f;
}

void mid_enc_update(void)
{
    uint8_t  f[3];
    uint32_t w;
    uint16_t d;
    uint8_t  mg;
    uint8_t  crc_rx;
    uint8_t  rc;
    float    mech;
    float    elec;

    rc = bsp_ssi_poll(f);
    if (s_enc_enabled == 0u) {
        return;                  /* no encoder: nothing to read */
    }

    if (rc == 0u) {
        /* Nothing ready. Two very different cases:
         *  - a transfer is still armed  -> the DMA missed its slot (late)
         *  - nothing was armed at all    -> the bridge is off and this call is
         *    the one that arms the next frame, which is the normal 1 kHz path
         * Counting both as "late" made the stopped state look broken
         * (late == frames), so only the first case is counted. */
        if (bsp_ssi_busy() != 0u) {
            s_enc.late++;
        }
        bsp_ssi_start();
        return;
    }

    s_enc.frames++;

    w      = ((uint32_t)f[0] << 16) | ((uint32_t)f[1] << 8) | (uint32_t)f[2];
    d      = (uint16_t)((w >> 10) & 0x3FFFu);
    mg     = (uint8_t)((w >> 6) & 0x0Fu);
    crc_rx = (uint8_t)(w & 0x3Fu);

    s_enc.raw    = d;
    s_enc.mg     = mg;
    s_enc.crc_ok = (mid_enc_crc6(w >> 6) == crc_rx) ? 1u : 0u;
    if ((d != 0u) && (s_enc.crc_ok != 0u)) {
        s_raw_seen_ok = 1u;      /* a real encoder is answering */
    }
    if (s_enc.crc_ok == 0u) {
        s_enc.crc_errs++;
    }

    /* A corrupt frame must never reach the control loop: keep the previous
     * angle (one period old, 0.09 deg at 5 Hz) instead of a wrong one. */
    if (rc == 2u) {
        s_enc.ovr++;
        return;
    }
    if (s_enc.crc_ok == 0u) {
        return;
    }

    /* Angle: raw -> mechanical degrees, then remove the zero reference. */
    mech = (float)d * ENC_RAD_PER_LSB;                 /* 0 .. 2pi */
    mech -= s_zero_rad;
    while (mech < 0.0f)       { mech += ENC_TWO_PI; }
    while (mech >= ENC_TWO_PI) { mech -= ENC_TWO_PI; }

    /* Electrical angle: x pole pairs, direction reverses the sign. */
    elec = mech * (float)ENC_POLE_PAIRS;
    while (elec >= ENC_TWO_PI) { elec -= ENC_TWO_PI; }
    if (s_enc.dir != 0u) {
        elec = ENC_TWO_PI - elec;
        if (elec >= ENC_TWO_PI) { elec = 0.0f; }
    }

    s_enc.mech_deg = mech * ENC_RAD_TO_DEG;
    s_enc.elec_deg = elec * ENC_RAD_TO_DEG;

    /* ---- S8.1: PLL observer, every frame ---- */
    {
        uint32_t nowp = bsp_prof_now();

        if (s_t_pll != 0u) {
            uint32_t dtp = nowp - s_t_pll;

            /* A gap (after "enc zero" or a stalled loop) would make the
             * loop see a huge delta: restart instead of following it. */
            if ((dtp > 0u) && (dtp < (2u * SYS_CORE_CLK_HZ / 1000u))) {
                float w_rad = alg_pll_step(&s_pll, elec,
                                           (float)dtp / (float)SYS_CORE_CLK_HZ);

                if (s_spd_src == 1u) {
                    float f_elec = w_rad / ENC_TWO_PI;

                    s_enc.spd_hz  = f_elec;
                    s_enc.spd_rpm = f_elec * (60.0f / (float)ENC_POLE_PAIRS);
                }
            } else {
                alg_pll_reset(&s_pll, elec);
            }
        } else {
            alg_pll_reset(&s_pll, elec);
        }
        s_t_pll = nowp;
    }

    /* ---- speed: angle delta over a fixed time window ---- */
    {
        uint32_t now = bsp_prof_now();

        if (s_have_prev != 0u) {
            float    dm = mech - s_prev_mech;
            float    de = elec - s_prev_elec;
            uint32_t dt = now - s_t_prev;

            /* shortest path across the 2pi wrap */
            if (dm > 3.14159265f)  { dm -= ENC_TWO_PI; }
            if (dm < -3.14159265f) { dm += ENC_TWO_PI; }
            if (de > 3.14159265f)  { de -= ENC_TWO_PI; }
            if (de < -3.14159265f) { de += ENC_TWO_PI; }

            if (dt == 0u) {
                /* Two updates at the same instant: keep the reference so
                 * the next call covers this movement (and so a window never
                 * mixes deltas that have no elapsed time behind them). */
            } else if (dt > (2u * SYS_CORE_CLK_HZ / 1000u)) {
                /* A real gap (after "enc zero" or a stalled loop) would
                 * alias the wrap: restart the window instead. Clearing
                 * s_win_rpm matters: leaving the old value behind made the
                 * lost-lock guard read a frozen speed forever. */
                s_acc_mech = 0.0f;
                s_acc_elec = 0.0f;
                s_acc_cyc  = 0u;
                s_win_rpm  = 0.0f;
                s_t_prev    = now;
                s_prev_mech = mech;
                s_prev_elec = elec;
            } else {
                s_acc_mech += dm;
                s_acc_elec += de;
                s_acc_cyc  += dt;
                s_t_prev    = now;
                s_prev_mech = mech;
                s_prev_elec = elec;
            }
            if (s_acc_cyc >= (SYS_CORE_CLK_HZ / 1000u) * ENC_SPD_WINDOW_MS) {
                float dt_s = (float)s_acc_cyc / (float)SYS_CORE_CLK_HZ;
                s_win_rpm = ((s_acc_mech / dt_s) * (60.0f / ENC_TWO_PI));

                /* The window estimator only drives the outputs when it is
                 * the selected source; the PLL keeps them otherwise. */
                if (s_spd_src == 0u) {
                    s_enc.spd_hz  = (s_acc_elec / dt_s) / ENC_TWO_PI;
                    s_enc.spd_rpm = s_win_rpm;
                }
                s_acc_mech = 0.0f;
                s_acc_elec = 0.0f;
                s_acc_cyc  = 0u;
            }
        }

        s_have_prev = 1u;
    }

    s_enc.valid = 1u;
}

void mid_enc_set_enabled(uint8_t en)
{
    s_enc_enabled = (en != 0u) ? 1u : 0u;
    if (s_enc_enabled == 0u) {
        s_enc.valid = 0u;        /* a dead link must never look valid */
    }
}

uint8_t mid_enc_enabled(void) { return s_enc_enabled; }

uint8_t mid_enc_seen_ok(void) { return s_raw_seen_ok; }

void mid_enc_set_spd_src(uint8_t src)
{
    s_spd_src = src;
    if (src == 1u) {
        alg_pll_reset(&s_pll, s_enc.elec_deg * (ENC_TWO_PI / 360.0f));
    }
}

uint8_t mid_enc_spd_src(void)
{
    return s_spd_src;
}

float mid_enc_pll_rpm(void)
{
    return (s_pll.omega / ENC_TWO_PI) * (60.0f / (float)ENC_POLE_PAIRS);
}

uint8_t mid_enc_pll_locked(void)
{
    return s_pll.locked;
}

void mid_enc_set_pll_bw(float wn_hz, float zeta)
{
    alg_pll_set_bw(&s_pll, wn_hz, zeta);
}

/* Drop the PLL state: called when the bridge is disabled so a free-running
 * speed estimate cannot survive the transition (measured: the reported
 * speed stayed at the 2000 rpm clamp for ~6 s after "stop").
 * Deliberately NOT a motion detector: that interacted with the gap path
 * of the PLL and fired during normal acceleration. */
void mid_enc_pll_clear(void)
{
    s_pll_clears++;
    s_pll.omega = 0.0f;
    alg_pll_reset(&s_pll, s_enc.elec_deg * (ENC_TWO_PI / 360.0f));
    s_enc.spd_hz  = 0.0f;
    s_enc.spd_rpm = 0.0f;
    s_win_rpm     = 0.0f;
    s_acc_mech = 0.0f;
    s_acc_elec = 0.0f;
    s_acc_cyc  = 0u;
}

uint32_t mid_enc_pll_clears(void)
{
    return s_pll_clears;
}

float mid_enc_win_rpm(void)
{
    return s_win_rpm;
}

const mid_enc_t *mid_enc_info(void)
{
    return &s_enc;   /* single writer, no shadow copy */
}

void mid_enc_clear_errs(void)
{
    /* Only the error counters: "frames" keeps advancing so the enc
     * command can still show an error rate after a clear. */
    s_enc.crc_errs = 0u;
    s_enc.ovr      = 0u;
    s_enc.late     = 0u;
    s_enc.io_errs  = 0u;
}

uint8_t mid_enc_zero(void)
{
    uint8_t f[3];
    uint32_t w;
    uint16_t d;

    if (bsp_ssi_read(f) != 0u) {
        return 1u;
    }

    w = ((uint32_t)f[0] << 16) | ((uint32_t)f[1] << 8) | (uint32_t)f[2];
    d = (uint16_t)((w >> 10) & 0x3FFFu);

    s_enc.zero_deg = (float)d * ENC_RAD_PER_LSB * ENC_RAD_TO_DEG;
    s_zero_rad     = (float)d * ENC_RAD_PER_LSB;
    if (s_enc.zero_deg >= 360.0f) {
        s_enc.zero_deg -= 360.0f;
    }
    enc_save();
    s_have_prev = 0u;            /* angles jumped: restart the window */

    return 0u;
}

uint8_t mid_enc_zero_set(float deg)
{
    while (deg < 0.0f)    { deg += 360.0f; }
    while (deg >= 360.0f) { deg -= 360.0f; }

    s_enc.zero_deg = deg;
    s_zero_rad     = deg / ENC_RAD_TO_DEG;
    enc_save();
    s_have_prev    = 0u;             /* angles jumped: restart the speed window */
    return 0u;
}

uint8_t mid_enc_set_dir(uint8_t dir)
{
    if (dir > 1u) {
        return 1u;
    }
    s_enc.dir   = dir;
    s_have_prev = 0u;            /* electrical angle flipped */
    enc_save();
    return 0u;
}
