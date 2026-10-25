/**
  ******************************************************************************
  * @file    mid_enc.h
  * @brief   MT6701 angle: frame decode, zero reference, direction, electrical
  *          angle for the FOC loop.
  *
  * Layer: Mid. Reads frames through bsp_ssi.
  ******************************************************************************
  */

#ifndef __MID_ENC_H
#define __MID_ENC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ENC_SRC_NONE    0u
#define ENC_SRC_FLASH   1u

typedef struct {
    uint16_t raw;          /* D[13:0] of the last frame            */
    uint8_t  mg;           /* Mg[3:0] magnetic status, 0 = normal  */
    uint8_t  crc_ok;       /* last frame CRC matched               */
    uint8_t  valid;        /* at least one good frame received      */
    uint8_t  src;          /* zero/dir came from flash or defaults */
    uint8_t  dir;          /* 0 = CCW positive, 1 = reversed        */
    float    mech_deg;     /* mechanical angle after zero, 0..360  */
    float    elec_deg;     /* electrical angle, 0..360             */
    float    spd_hz;       /* electrical frequency, signed         */
    float    spd_rpm;      /* mechanical speed, signed             */
    float    zero_deg;     /* stored zero offset                   */
    uint32_t frames;       /* frames read                          */
    uint32_t crc_errs;     /* frames whose CRC did not match       */
    uint32_t io_errs;
    uint32_t ovr;
    uint32_t late;      /* SPI transfer failures                */
} mid_enc_t;

/* Load the zero/direction record from flash and prepare the bus. */
void mid_enc_init(void);

/* Read and decode one frame. Call once per control frame. */
void mid_enc_update(void);

const mid_enc_t *mid_enc_info(void);

/* Current mechanical angle becomes electrical zero. Persists to flash. */
uint8_t mid_enc_zero(void);

/* Set the electrical zero to an explicit mechanical angle (degrees).
 * Used by the bidirectional-sweep alignment. Persists to flash. */
uint8_t mid_enc_zero_set(float deg);

/* 0 = CCW positive (default), 1 = reversed. Persists to flash. */
uint8_t mid_enc_set_dir(uint8_t dir);

/* CRC-6 (x^6 + x + 1) over the 18 bits D[13:0] + Mg[3:0], MSB first.
 * Exposed for the host tests. */
uint8_t mid_enc_crc6(uint32_t bits18);

#ifdef __cplusplus
}
#endif

#endif /* __MID_ENC_H */
