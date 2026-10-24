/**
  ******************************************************************************
  * @file    mid_ringbuf.h
  * @brief   Single-producer / single-consumer lock-free ring buffer.
  *          Capacity must be a power of two.
  ******************************************************************************
  */

#ifndef __MID_RINGBUF_H
#define __MID_RINGBUF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    uint8_t          *buf;
    uint16_t          size;    /* must be a power of two */
    uint16_t          mask;    /* size - 1 */
    volatile uint16_t head;    /* write index, producer only */
    volatile uint16_t tail;    /* read index, consumer only */
} ringbuf_t;

void     ring_init(ringbuf_t *r, uint8_t *mem, uint16_t size);
void     ring_reset(ringbuf_t *r);

/* Number of stored bytes */
uint16_t ring_avail(const ringbuf_t *r);
/* Free space (usable capacity is size-1) */
uint16_t ring_space(const ringbuf_t *r);

/* Push data, returns the number of bytes actually written (truncated if full) */
uint16_t ring_push(ringbuf_t *r, const uint8_t *src, uint16_t len);
/* Pop and discard, returns the number of bytes actually read */
uint16_t ring_pop(ringbuf_t *r, uint8_t *dst, uint16_t max);
/* Peek without consuming; only returns the contiguous segment from tail */
uint16_t ring_peek(const ringbuf_t *r, uint8_t *dst, uint16_t max);
/* Drop len bytes (pair with ring_peek) */
void     ring_drop(ringbuf_t *r, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __MID_RINGBUF_H */
