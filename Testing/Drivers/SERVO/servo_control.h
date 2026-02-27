#ifndef INC_SERVO_CONTROL_H_H
#define INC_SERVO_CONTROL_H_H

#include "main.h"

typedef struct {
	TIM_HandleTypeDef* htim;
	uint32_t           channel;
	uint8_t            is_initialized;
} ServoConfig;

void InitServo(uint8_t servo_id, TIM_HandleTypeDef* htim, uint32_t channel);
void SetAngleServo(uint8_t servo_id, uint8_t angle);
void OpenAllServo(uint8_t angle);
void CloseAllServo(uint8_t angle);
void RunManagerServo(void);

#endif // INC_SERVO_CONTROL_H_H
