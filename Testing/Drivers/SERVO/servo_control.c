#include "servo_control.h"

#include "soft_timer.h"

#define MAX_SERVOS 3

uint8_t angle_open_internal = 180U;
uint8_t angle_close_internal = 40U;

static ServoConfig servo_config[MAX_SERVOS];

static SoftTimer servo_timer;
static uint8_t servo_step = 0;
static uint8_t servo_mode = 0;

void OpenAllServo(uint8_t angle) {
	angle_open_internal = angle;
	servo_step = 1;
	servo_mode = 1;
	StartSoftTimer(&servo_timer, 0);
}

void CloseAllServo(uint8_t angle) {
	angle_close_internal = angle;
	servo_step = 1;
	servo_mode = 2;
	StartSoftTimer(&servo_timer, 0);
}

void RunManagerServo(void) { 
  if (0 == servo_step) return;
	
	if (IsExpiredSoftTimer(&servo_timer)) {
		if (1 == servo_mode) {
			switch (servo_step) {
				case 1:
					SetAngleServo(1, angle_open_internal);
					StartSoftTimer(&servo_timer, 4000);
					servo_step++;
					break;
				
				case 2:
					SetAngleServo(2, angle_open_internal);
					StartSoftTimer(&servo_timer, 4000);
					servo_step++;
					break;
				
				case 3:
					SetAngleServo(3, angle_open_internal);
					servo_step = 0;
					break;
			}
		} else if (2 == servo_mode) {
			switch (servo_step) {
				case 1:
					SetAngleServo(1, angle_close_internal);
					StartSoftTimer(&servo_timer, 4000);
					servo_step++;
					break;
				
				case 2:
					SetAngleServo(2, angle_close_internal);
					StartSoftTimer(&servo_timer, 4000);
					servo_step++;
					break;
				
				case 3:
					SetAngleServo(3, angle_close_internal);
					servo_step = 0;
					break;
			}
		}
	}
}

/**
 * @brief Initialing id, htim, channel for Servo
 * @param id: The number of channel (1, 2, 3)
 * @param htim: Timer channel you use for servo (&htim1, &htim2, ...)
 * @param channel: Channel of Timer above (TIM_CHANNEL_1, TIM_CHANNEL_2, ...)
 **/
void InitServo(uint8_t servo_id ,TIM_HandleTypeDef* htim, uint32_t channel) {
	if (servo_id < 1 || servo_id > MAX_SERVOS || htim == NULL) {
		
		return;
	}
	
	uint8_t index = servo_id - 1;
	
	servo_config[index].htim = htim;
	servo_config[index].channel = channel;
	servo_config[index].is_initialized = 1;
	
	HAL_TIM_PWM_Start(htim, channel);
}

/**
 * @brief Set angle for each Servo.
 * @param servo_id: The number of channel (1, 2, 3).
 * @param angle: Angle you want Servo act (0 -> 180).
 **/
void SetAngleServo(uint8_t servo_id, uint8_t angle) {
	if (servo_id < 1 || servo_id > MAX_SERVOS) {
		
		return;
	}
	
	uint8_t index = servo_id - 1;
	
	if (servo_config[index].is_initialized == 0) {
		
		return;
	}
	
	if (angle > 180) {
		angle = 180;
	}
	
	const uint16_t MIN_PULSE = 500;
	const uint16_t MAX_PULSE = 2500;

	uint16_t pulse = MIN_PULSE + (uint16_t)((angle / 180.0f) * (MAX_PULSE - MIN_PULSE));

	TIM_HandleTypeDef* htim = servo_config[index].htim;
	uint32_t channel = servo_config[index].channel;

	__HAL_TIM_SET_COMPARE(htim, channel, pulse);
}
