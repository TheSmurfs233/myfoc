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

    /* The HAL enables SPE per transfer and disables it afterwards. The frame
     * reader below pokes the registers directly, so enable it once here. */
    SPI1->CR1 |= SPI_CR1_SPE;
}

/* Bounded spin so a dead bus cannot hang the control loop. At 10.625 MHz a
 * byte takes ~0.75us (~130 cycles); this is orders of magnitude larger. */
#define SSI_SPIN_LIMIT   20000u

uint8_t bsp_ssi_read(uint8_t frame[3])
{
    uint8_t  i;
    uint32_t spin;
    uint8_t  ok = 0u;

    if (frame == 0) {
        return 1u;
    }

    frame[0] = 0u;
    frame[1] = 0u;
    frame[2] = 0u;

    /* CSN low (BSRR reset) */
    CSN_GPIO_Port->BSRR = (uint32_t)CSN_Pin << 16;

    for (i = 0u; i < 3u; i++) {
        spin = 0u;
        while (((SPI1->SR & SPI_SR_TXE) == 0u) && (spin < SSI_SPIN_LIMIT)) {
            spin++;
        }
        if (spin >= SSI_SPIN_LIMIT) {
            goto done;
        }

        /* dummy byte out (MT6701 has no DI, but clocks must be generated) */
        *(volatile uint8_t *)&SPI1->DR = 0xFFu;

        spin = 0u;
        while (((SPI1->SR & SPI_SR_RXNE) == 0u) && (spin < SSI_SPIN_LIMIT)) {
            spin++;
        }
        if (spin >= SSI_SPIN_LIMIT) {
            goto done;
        }

        frame[i] = *(volatile uint8_t *)&SPI1->DR;
    }

    ok = 1u;

done:
    /* wait for the last bit to leave before releasing CSN */
    spin = 0u;
    while (((SPI1->SR & SPI_SR_BSY) != 0u) && (spin < SSI_SPIN_LIMIT)) {
        spin++;
    }

    /* CSN high (BSRR set) */
    CSN_GPIO_Port->BSRR = CSN_Pin;

    return (ok != 0u) ? 0u : 1u;
}
void bsp_ssi_diag(uint32_t *cfg1, uint32_t *ccipr2)
{
    if (cfg1 != 0)   { *cfg1   = SPI1->CR1;   /* classic SPI IP: BR[5:3] holds the divider */ }
    if (ccipr2 != 0) { *ccipr2 = RCC->CCIPR2; }
}
