//
// Created by TranQuocCuong on 7/17/2025.
//

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    RING_FAIL = -1,
    RING_OK = 0,
} ring_stt_t;

typedef struct {
    uint32_t w;
    uint32_t rd;
    uint32_t size;
    uint8_t *buf;
} RINGBUF;

ring_stt_t RINGBUF_Init(RINGBUF *ring, uint8_t *buf, uint32_t maxSize);
ring_stt_t RINGBUF_Put(RINGBUF *ring, uint8_t data);
ring_stt_t RINGBUF_Get(RINGBUF *ring, uint8_t *data);

#endif //RINGBUFFER_H
