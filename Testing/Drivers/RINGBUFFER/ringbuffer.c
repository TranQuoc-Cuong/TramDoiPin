#include "ringbuffer.h"

RingStt InitRingBuf(RingBuf *ring, uint8_t *buf, uint32_t maxSize) {
	if (!buf || maxSize == 0) {
		
		return RING_FAIL;
	}
	
	ring->w = 0;
	ring->rd = 0;
	ring->buf = buf;
	ring->size = maxSize;
	
	return RING_OK;
}

RingStt PutRingBuf(RingBuf *ring, uint8_t data) {
	uint16_t temp = ring->w;
	temp++;
	if (temp >= ring->size) {
		temp = 0;
	}

	if (temp == ring->rd) {
		
		return RING_FAIL;
	}

	ring->buf[ring->w] = data;
	ring->w = temp;
	
	return RING_OK;
}

RingStt GetRingBuf(RingBuf *ring, uint8_t *data) {
	if (ring->w == ring->rd) {
		
		return RING_FAIL;
	}

	*data = ring->buf[ring->rd];
	ring->rd++;

	if (ring->rd >= ring->size) {
		ring->rd = 0;
	}

	return RING_OK;
}
