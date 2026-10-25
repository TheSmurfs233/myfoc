/**
  ******************************************************************************
  * @file    bsp_ssi_fake.c
  * @brief   Host stand-in for the MT6701 SSI reader.
  *
  * Tests set fake_ssi_frame (and optionally fake_ssi_fail) before calling
  * mid_enc_update(); the firmware code under test is unchanged.
  * The host has no notion of time, so a frame is "ready" immediately.
  ******************************************************************************
  */

#include "bsp_ssi.h"

uint8_t  fake_ssi_frame[3];
uint8_t  fake_ssi_fail;
uint8_t  fake_ssi_busy;
uint32_t fake_ssi_reads;

void bsp_ssi_init(void)
{
    fake_ssi_reads = 0u;
}

void bsp_ssi_start(void)
{
    /* nothing to arm on the host */
}

uint8_t bsp_ssi_poll(uint8_t frame[3])
{
    if (fake_ssi_fail != 0u) {
        return 0u;                    /* simulate "not ready yet" */
    }

    fake_ssi_reads++;

    frame[0] = fake_ssi_frame[0];
    frame[1] = fake_ssi_frame[1];
    frame[2] = fake_ssi_frame[2];
    return 1u;
}

uint8_t bsp_ssi_read(uint8_t frame[3])
{
    if (fake_ssi_fail != 0u) {
        frame[0] = 0u;
        frame[1] = 0u;
        frame[2] = 0u;
        return 1u;
    }
    return (bsp_ssi_poll(frame) != 0u) ? 0u : 1u;
}

void bsp_ssi_abort(void)
{
    fake_ssi_busy = 0u;
}

uint8_t bsp_ssi_busy(void)
{
    return fake_ssi_busy; /* tests set this to simulate "armed, not ready" */
}

uint32_t bsp_ssi_ovr_count(void)
{
    return 0u;
}

void bsp_ssi_diag(uint32_t *cr1, uint32_t *ccipr2)
{
    if (cr1 != 0)    { *cr1    = 0u; }
    if (ccipr2 != 0) { *ccipr2 = 0u; }
}
