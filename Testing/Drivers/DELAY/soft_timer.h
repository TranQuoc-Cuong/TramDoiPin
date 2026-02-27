#ifndef SOFT_TIMER_H
#define SOFT_TIMER_H

#include <stdint.h>

#include "main.h"

typedef struct {
	uint32_t start_time;  
	uint32_t interval;    
	uint8_t  timeout;     
} SoftTimer;

static inline void StartSoftTimer(SoftTimer *timer, uint32_t ms) {
	timer->interval = ms;
	timer->start_time = HAL_GetTick();
	timer->timeout = 0;
}

static inline uint8_t IsExpiredSoftTimer(SoftTimer *timer) {
	if ((HAL_GetTick() - timer->start_time) >= timer->interval) {
		timer->start_time = HAL_GetTick(); 
		timer->timeout = 1;
		
		return 1;
	}
	
	return 0;
}

static inline uint8_t IsExpiredOneShotSoftTimer(SoftTimer *timer) {
	if (1 == timer->timeout) return 1;

	if ((HAL_GetTick() - timer->start_time) >= timer->interval) {
			timer->timeout = 1;
		
			return 1;
	}
	
	return 0;
}

#endif /* SOFT_TIMER_H */
