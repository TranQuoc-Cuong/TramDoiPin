#ifndef INC_SERVO_CONTROL_H_H
#define INC_SERVO_CONTROL_H_H

#include "main.h"

void Servos_Init(TIM_HandleTypeDef *htim);

void Servo_SetAngle(uint8_t servo_id, uint8_t angle);

void All_Servo_Angle(uint8_t angle);


#endif // INC_SERVO_CONTROL_H_H
