/**
  ******************************************************************************
  * @file    bsp_ssi.c
  * @brief   MT6701 SSI frame reader, DMA based (see bsp_ssi.h).
  *
  * Layer: Bsp.
  *
  * Why DMA: the blocking version (HAL_SPI_Receive, later a hand-written polling
  * loop) cost 6.2us per frame, of which only 2.26us was pure shifting and about
  * 1.3us was the CPU spinning on RXNE. In S6 the angle is needed BEFORE the
  * Park transform, so those microseconds land inside the control chain. Moving
  * the RX side to DMA leaves just the three dummy writes (~0.3us) on the CPU.
  *
  * DMA1 channel 3 is free (USART1 uses channels 1 and 2); DMAMUX request 10 is
  * SPI1_RX.
  ******************************************************************************
  */

#include "bsp_ssi.h"
#include "main.h"
#include "stm32g4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

#define SSI_DMA           DMA1_Channel3
/* TXE comes back within one bit time (94 ns ~ 16 cycles); a generous 128
 * iterations is still under 1 us. The old 20000 could have blocked the
 * 20 kHz ISR for over a millisecond if the SPI ever stalled. */
#define SSI_SPIN_LIMIT    128u
#define SSI_BUSY_LIMIT    4u      /* frames in flight before we force a reset */

static volatile uint8_t s_rx[3];
static volatile uint8_t s_busy;
static volatile uint32_t s_ovr;   /* SPI RX overruns (a byte was lost) */
static uint8_t  s_stuck;  /* consecutive ISR periods still busy */

void bsp_ssi_init(void)
{
    /* CSN idles high. SPI1 and DMA1_Channel3 are both initialised by CubeMX:
     * MX_DMA_Init() calls HAL_DMA_Init(&hdma_spi1_rx) with the same settings
     * this driver needs (SPI1_RX, byte, memory increment, normal mode), so
     * only the pieces HAL_DMA_Start() would normally set are done here. */
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

    /* CubeMX also enables DMA1_Channel3 in the NVIC, which is its default when a
     * DMA request is added. That is harmless here: this driver polls the TC flag
     * and never sets TCIE, so no interrupt request can ever be generated. */
    SSI_DMA->CPAR  = (uint32_t)&SPI1->DR;
    SSI_DMA->CMAR  = (uint32_t)s_rx;
    /* Channel priority (VERY_HIGH) and normal mode come from CubeMX. */

    s_busy = 0u;

    /* The HAL enables SPE per transfer; this driver pokes the registers, so
     * enable it once here. */
    SPI1->CR1 |= SPI_CR1_SPE;
}

void bsp_ssi_start(void)
{
    uint32_t spin;
    uint8_t  i;

    if (s_busy != 0u) {
        /* A previous transfer is still in flight. Normally the main loop
         * collects it long before this, so seeing several ISR periods in a row
         * means the DMA or the frame is wedged: drop it and start over. */
        if (++s_stuck >= SSI_BUSY_LIMIT) {
            bsp_ssi_abort();
        }
        return;
    }
    s_stuck = 0u;

    SSI_DMA->CCR   &= ~DMA_CCR_EN;
    SSI_DMA->CNDTR  = 3u;
    DMA1->IFCR      = DMA_IFCR_CGIF3;
    SSI_DMA->CCR   |= DMA_CCR_EN;
    SPI1->CR2      |= SPI_CR2_RXDMAEN;

    /* CSN low (BSRR reset) */
    CSN_GPIO_Port->BSRR = (uint32_t)CSN_Pin << 16;

    /* Three dummy bytes generate the 24 clocks: this is the only CPU time the
     * transfer costs, the DMA collects the incoming bytes meanwhile. */
    for (i = 0u; i < 3u; i++) {
        spin = 0u;
        while (((SPI1->SR & SPI_SR_TXE) == 0u) && (spin < SSI_SPIN_LIMIT)) {
            spin++;
        }
        *(volatile uint8_t *)&SPI1->DR = 0xFFu;
    }

    s_busy = 1u;
}

uint8_t bsp_ssi_poll(uint8_t frame[3])
{
    if (s_busy == 0u) {
        return 0u;
    }
    if ((DMA1->ISR & DMA_ISR_TCIF3) == 0u) {
        return 0u;                    /* still shifting */
    }

    SPI1->CR2   &= ~SPI_CR2_RXDMAEN;
    SSI_DMA->CCR &= ~DMA_CCR_EN;

    /* CSN high (BSRR set) */
    CSN_GPIO_Port->BSRR = CSN_Pin;

    s_busy = 0u;

    if (frame != 0) {
        frame[0] = s_rx[0];
        frame[1] = s_rx[1];
        frame[2] = s_rx[2];
    }

    /* OVR means a received byte was overwritten before the DMA took it: the
     * SPI RX buffer is one byte deep and the DMA can lose the arbitration to
     * the UART DMA or the CPU. The frame is then corrupted even if the CRC
     * happens to pass, so report it separately. Clear by reading DR then SR. */
    if ((SPI1->SR & SPI_SR_OVR) != 0u) {
        (void)SPI1->DR;
        (void)SPI1->SR;
        s_ovr++;
        return 2u;
    }

    return 1u;
}

uint32_t bsp_ssi_ovr_count(void)
{
    return s_ovr;
}

uint8_t bsp_ssi_busy(void)
{
    return s_busy;
}

void bsp_ssi_abort(void)
{
    SPI1->CR2   &= ~SPI_CR2_RXDMAEN;
    SSI_DMA->CCR &= ~DMA_CCR_EN;
    CSN_GPIO_Port->BSRR = CSN_Pin;          /* CSN high */
    s_busy  = 0u;
    s_stuck = 0u;
}

uint8_t bsp_ssi_read(uint8_t frame[3])
{
    uint32_t spin = 0u;

    bsp_ssi_abort();                        /* drop anything left over */
    bsp_ssi_start();
    while ((bsp_ssi_poll(frame) == 0u) && (spin < 500000u)) {
        spin++;
    }
    if (spin >= 500000u) {
        bsp_ssi_abort();
        return 1u;
    }
    return 0u;
}

void bsp_ssi_diag(uint32_t *cr1, uint32_t *ccipr2)
{
    if (cr1 != 0)    { *cr1    = SPI1->CR1; }
    if (ccipr2 != 0) { *ccipr2  = RCC->CCIPR2; }
}
