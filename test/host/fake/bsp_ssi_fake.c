/**
  ******************************************************************************
  * @file    bsp_ssi_fake.c
  * @brief   Host stand-in for the MT6701 SSI reader.
  *
  * Tests set fake_ssi_frame (and optionally fake_ssi_fail) before calling
  * mid_enc_update(); the firmware code under test is unchanged.
  ******************************************************************************
  */

#include "bsp_ssi.h"

uint8_t fake_ssi_frame[3];
uint8_t fake_ssi_fail;
uint32_t fake_ssi_reads;

void bsp_ssi_init(void)
{
    fake_ssi_reads = 0u;
}

uint8_t bsp_ssi_read(uint8_t frame[3])
{
    fake_ssi_reads++;

    if (fake_ssi_fail != 0u) {
        frame[0] = 0u;
        frame[1] = 0u;
        frame[2] = 0u;
        return 1u;
    }

    frame[0] = fake_ssi_frame[0];
    frame[1] = fake_ssi_frame[1];
    frame[2] = fake_ssi_frame[2];
    return 0u;
}
