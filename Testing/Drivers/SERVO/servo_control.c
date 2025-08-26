#include "servo_control.h"

static TIM_HandleTypeDef* servo_timer_handle = NULL;

static const uint32_t servo_channels[] = {
	TIM_CHANNEL_1,
	TIM_CHANNEL_2,
	TIM_CHANNEL_3,
	TIM_CHANNEL_4
};

void Servos_Init(TIM_HandleTypeDef* htim){
	servo_timer_handle = htim;
	
	for(int i = 0; i < 4; i++){
		HAL_TIM_PWM_Start(htim, servo_channels[i]);
	};
};

void Servo_SetAngle(uint8_t servo_id, uint8_t angle) {
    if (servo_timer_handle == NULL || servo_id < 1 || servo_id > 4) {
        return;
    }

    if (angle > 180) {
        angle = 180;
    }

    const uint16_t MIN_PULSE = 500;
    const uint16_t MAX_PULSE = 2500;

    uint16_t pulse = MIN_PULSE + (uint16_t)((angle / 180.0f) * (MAX_PULSE - MIN_PULSE));

    uint32_t channel = servo_channels[servo_id - 1];

    __HAL_TIM_SET_COMPARE(servo_timer_handle, channel, pulse);
}

void All_Servo_Angle(uint8_t angle){
	Servo_SetAngle(1, angle);
	HAL_Delay(200);
	Servo_SetAngle(2, angle);
	HAL_Delay(200);
	Servo_SetAngle(3, angle);
	HAL_Delay(200);
	Servo_SetAngle(4, angle);
}
