//
// Created by TranQuocCuong on 7/17/2025.
//
#include "ringbuffer.h"

ring_stt_t RINGBUF_Init(RINGBUF *ring, uint8_t *buf, uint32_t maxSize) {
    if (!buf || maxSize == 0)
        return RING_FAIL;
    ring->w = 0;
    ring->rd = 0;
    ring->buf = buf;
    ring->size = maxSize;
}

ring_stt_t RINGBUF_Put(RINGBUF *ring, uint8_t data) {
    uint16_t temp = ring->w;
    temp++;
    if (temp >= ring->size) {
        temp = 0;
    }

    if (temp == ring->rd) { //full
        return RING_FAIL;
    }

    ring->buf[ring->w] = data;
    ring->w = temp;
    return RING_OK;
}

ring_stt_t RINGBUF_Get(RINGBUF *ring, uint8_t *data) {
    if (ring->w == ring->rd) { //empty
        return RING_FAIL;
    }

    *data = ring->buf[ring->rd];
    ring->rd++;

    if (ring->rd >= ring->size) {
        ring->rd = 0;
    }

    return RING_OK;
}