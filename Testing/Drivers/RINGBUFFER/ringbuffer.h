#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	RING_FAIL = -1,
	RING_OK = 0,
} RingStt;

typedef struct {
	uint32_t w;
	uint32_t rd;
	uint32_t size;
	uint8_t *buf;
} RingBuf;

RingStt InitRingBuf(RingBuf *ring, uint8_t *buf, uint32_t maxSize);
RingStt PutRingBuf(RingBuf *ring, uint8_t data);
RingStt GetRingBuf(RingBuf *ring, uint8_t *data);

#endif //RINGBUFFER_H
