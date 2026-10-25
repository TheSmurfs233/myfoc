/**
  ******************************************************************************
  * @file    bsp_ssi.c
  * @brief   MT6701 SSI frame reader (see bsp_ssi.h).
  *
  * Layer: Bsp.
  *
  * The chip latches the magnetic field continuously (system delay < 5us) and
  * shifts the frame out while CSN is low: data changes on the rising edge of
  * CLK, the host samples on the falling edge -> SPI mode 1, which is what
  * CubeMX generated. 24 bits at 10.625 MHz take 2.26us.
  ******************************************************************************
  */

#include "bsp_ssi.h"
#include "main.h"
#include "stm32g4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

void bsp_ssi_init(void)
{
    /* CSN idles high; SPI1 itself is initialised by CubeMX (MX_SPI1_Init). */
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}

uint8_t bsp_ssi_read(uint8_t frame[3])
{
    HAL_StatusTypeDef st;

    if (frame == 0) {
        return 1u;
    }

    frame[0] = 0u;
    frame[1] = 0u;
    frame[2] = 0u;

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);

    /* 3 bytes = exactly the 24 clocks the frame needs */
    st = HAL_SPI_Receive(&hspi1, frame, 3u, 2u);

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

    return (st == HAL_OK) ? 0u : 1u;
}
