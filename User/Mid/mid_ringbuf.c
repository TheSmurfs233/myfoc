/**
  ******************************************************************************
  * @file    mid_ringbuf.c
  * @brief   Ring buffer implementation.
  *
  * head is written only by the producer, tail only by the consumer, so no
  * interrupt masking is required for the single-producer/single-consumer case.
  * In this project: producer = DMA callback, consumer = main loop.
  ******************************************************************************
  */

#include "mid_ringbuf.h"

void ring_init(ringbuf_t *r, uint8_t *mem, uint16_t size)
{
    /* Only power-of-two sizes are supported, otherwise the mask trick fails */
    if ((size == 0u) || ((size & (uint16_t)(size - 1u)) != 0u)) {
        r->buf  = mem;
        r->size = 0u;
        r->mask = 0u;
    } else {
        r->buf  = mem;
        r->size = size;
        r->mask = (uint16_t)(size - 1u);
    }
    r->head = 0u;
    r->tail = 0u;
}

void ring_reset(ringbuf_t *r)
{
    r->head = 0u;
    r->tail = 0u;
}

uint16_t ring_avail(const ringbuf_t *r)
{
    return (uint16_t)((uint16_t)(r->head - r->tail) & r->mask);
}

uint16_t ring_space(const ringbuf_t *r)
{
    if (r->size == 0u) { return 0u; }
    return (uint16_t)(r->size - 1u - ring_avail(r));
}

uint16_t ring_push(ringbuf_t *r, const uint8_t *src, uint16_t len)
{
    uint16_t space = ring_space(r);
    uint16_t i;
    uint16_t head;

    if (len > space) { len = space; }
    head = r->head;
    for (i = 0u; i < len; i++) {
        r->buf[head] = src[i];
        head = (uint16_t)((head + 1u) & r->mask);
    }
    r->head = head;
    return len;
}

uint16_t ring_peek(const ringbuf_t *r, uint8_t *dst, uint16_t max)
{
    uint16_t avail = ring_avail(r);
    uint16_t n;
    uint16_t i;
    uint16_t tail;

    if (max > avail) { n = avail; } else { n = max; }

    /* Truncate to the non-wrapping segment */
    tail = r->tail;
    if ((uint16_t)(tail + n) > r->mask) {
        n = (uint16_t)(r->size - tail);
        if (n > avail) { n = avail; }
        if (n > max)   { n = max; }
    }
    for (i = 0u; i < n; i++) {
        dst[i] = r->buf[(uint16_t)((tail + i) & r->mask)];
    }
    return n;
}

uint16_t ring_pop(ringbuf_t *r, uint8_t *dst, uint16_t max)
{
    uint16_t total = 0u;

    /* Loop so a read spanning the wrap point still fills the caller's buffer.
     * ring_peek() alone would stop at the wrap boundary and return short. */
    while (total < max) {
        uint16_t n = ring_peek(r, &dst[total], (uint16_t)(max - total));
        if (n == 0u) { break; }
        ring_drop(r, n);
        total = (uint16_t)(total + n);
    }
    return total;
}

void ring_drop(ringbuf_t *r, uint16_t len)
{
    uint16_t avail = ring_avail(r);
    if (len > avail) { len = avail; }
    r->tail = (uint16_t)((r->tail + len) & r->mask);
}
