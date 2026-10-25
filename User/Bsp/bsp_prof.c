/**
  ******************************************************************************
  * @file    bsp_prof.c
  * @brief   DWT cycle counter based timing profiler (see bsp_prof.h).
  *
  * Cycle-to-time conversion uses SYS_CORE_CLK_HZ from board_cfg.h, so the
  * numbers stay correct if the clock ever changes.
  ******************************************************************************
  */

#include "bsp_prof.h"
#include "board_cfg.h"
#include "main.h"

static uint32_t s_min[PROF_SLOT_COUNT];
static uint32_t s_max[PROF_SLOT_COUNT];
static uint32_t s_cnt[PROF_SLOT_COUNT];
static uint64_t s_sum[PROF_SLOT_COUNT];
static uint8_t  s_enabled;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
static uint32_t cycles_to_ns(uint32_t cycles)
{
    /* 170MHz -> 1 cycle = 5.882ns. 64-bit intermediate, no overflow. */
    return (uint32_t)(((uint64_t)cycles * 1000000000ull) /
                      (uint64_t)SYS_CORE_CLK_HZ);
}

/* ------------------------------------------------------------------ */
/* Init / control                                                      */
/* ------------------------------------------------------------------ */
void bsp_prof_init(void)
{
    /* Enable the trace subsystem, reset and start the cycle counter. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    bsp_prof_reset();
    s_enabled = 1u;
}

void bsp_prof_reset(void)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)PROF_SLOT_COUNT; i++) {
        s_min[i] = 0xFFFFFFFFu;
        s_max[i] = 0u;
        s_cnt[i] = 0u;
        s_sum[i] = 0ull;
    }
}

void bsp_prof_enable(uint8_t on)
{
    s_enabled = (on != 0u) ? 1u : 0u;
}

uint8_t bsp_prof_enabled(void)
{
    return s_enabled;
}

uint32_t bsp_prof_now(void)
{
    return DWT->CYCCNT;
}

/* ------------------------------------------------------------------ */
/* Accumulation                                                        */
/* ------------------------------------------------------------------ */
void bsp_prof_add_cycles(prof_slot_t slot, uint32_t cycles)
{
    if (!s_enabled)                                    { return; }
    if ((uint8_t)slot >= (uint8_t)PROF_SLOT_COUNT)     { return; }
    if (cycles == 0u)                                  { return; }

    if (cycles < s_min[slot]) { s_min[slot] = cycles; }
    if (cycles > s_max[slot]) { s_max[slot] = cycles; }
    s_sum[slot] += (uint64_t)cycles;
    s_cnt[slot]++;
}

void bsp_prof_add(prof_slot_t slot, uint32_t t0)
{
    uint32_t now;

    if (!s_enabled) { return; }
    now = DWT->CYCCNT;
    bsp_prof_add_cycles(slot, (uint32_t)(now - t0));
}

/* ------------------------------------------------------------------ */
/* Readout                                                             */
/* ------------------------------------------------------------------ */
uint32_t bsp_prof_count(prof_slot_t slot)
{
    if ((uint8_t)slot >= (uint8_t)PROF_SLOT_COUNT) { return 0u; }
    return s_cnt[slot];
}

uint32_t bsp_prof_min_ns(prof_slot_t slot)
{
    if ((uint8_t)slot >= (uint8_t)PROF_SLOT_COUNT) { return 0u; }
    if (s_cnt[slot] == 0u)                         { return 0u; }
    return cycles_to_ns(s_min[slot]);
}

uint32_t bsp_prof_avg_ns(prof_slot_t slot)
{
    if ((uint8_t)slot >= (uint8_t)PROF_SLOT_COUNT) { return 0u; }
    if (s_cnt[slot] == 0u)                         { return 0u; }
    return cycles_to_ns((uint32_t)(s_sum[slot] / (uint64_t)s_cnt[slot]));
}

uint32_t bsp_prof_max_ns(prof_slot_t slot)
{
    if ((uint8_t)slot >= (uint8_t)PROF_SLOT_COUNT) { return 0u; }
    if (s_cnt[slot] == 0u)                         { return 0u; }
    return cycles_to_ns(s_max[slot]);
}
