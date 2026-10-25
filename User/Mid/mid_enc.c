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

static mid_enc_t s_enc;
static float    s_zero_rad;   /* zero_deg converted once */

/* Speed estimator state (see ENC_SPD_WINDOW_MS) */
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
    s_enc.spd_hz   = 0.0f;
    s_enc.spd_rpm  = 0.0f;

    s_have_prev = 0u;
    s_acc_mech  = 0.0f;
    s_acc_elec  = 0.0f;
    s_acc_cyc   = 0u;

    bsp_ssi_init();
    enc_load();

}

void mid_enc_update(void)
{
    uint8_t  f[3];
    uint32_t w;
    uint16_t d;
    uint8_t  mg;
    uint8_t  crc_rx;
    float    mech;
    float    elec;

    if (bsp_ssi_read(f) != 0u) {
        s_enc.io_errs++;
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
    if (s_enc.crc_ok == 0u) {
        s_enc.crc_errs++;
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
                /* A large gap (after "enc zero" or a stalled loop) would
                 * alias the wrap: restart the window instead. */
                s_acc_mech = 0.0f;
                s_acc_elec = 0.0f;
                s_acc_cyc  = 0u;
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
                s_enc.spd_hz  = (s_acc_elec / dt_s) / ENC_TWO_PI;
                s_enc.spd_rpm = (s_acc_mech / dt_s) * (60.0f / ENC_TWO_PI);
                s_acc_mech = 0.0f;
                s_acc_elec = 0.0f;
                s_acc_cyc  = 0u;
            }
        }

        s_have_prev = 1u;
    }

    s_enc.valid = 1u;
}

const mid_enc_t *mid_enc_info(void)
{
    return &s_enc;   /* single writer, no shadow copy */
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
