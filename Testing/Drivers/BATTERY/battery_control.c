#include "battery_control.h"

/*
	Gan tai
	GPIOB_12 pin1
	GPIOB_14 pin2
	GPIOB_0 pin3
	GPIOA_6 pin4
*/

/*
	Sac pin
	GPIOB_13 pin1
	GPIOB_15 pin2
	GPIOA_7 pin3
	GPIOA_5 pin4
*/

// --- Định nghĩa phần cứng ở một nơi duy nhất ---
// Kênh 1
#define CHARGE_1_PORT   GPIOB
#define CHARGE_1_PIN    GPIO_PIN_13
#define LOAD_1_PORT     GPIOB
#define LOAD_1_PIN      GPIO_PIN_12

// Kênh 2
#define CHARGE_2_PORT   GPIOB
#define CHARGE_2_PIN    GPIO_PIN_15
#define LOAD_2_PORT     GPIOB
#define LOAD_2_PIN      GPIO_PIN_14

// Kênh 3
#define CHARGE_3_PORT   GPIOA
#define CHARGE_3_PIN    GPIO_PIN_7
#define LOAD_3_PORT     GPIOB
#define LOAD_3_PIN      GPIO_PIN_0

// Kênh 4
#define CHARGE_4_PORT   GPIOA
#define CHARGE_4_PIN    GPIO_PIN_5
#define LOAD_4_PORT     GPIOA
#define LOAD_4_PIN      GPIO_PIN_6

/**
 * @brief Đặt trạng thái cho một kênh pin cụ thể.
 * @param channel_id: Số thứ tự kênh (1, 2, 3, hoặc 4)
 * @param new_state: Trạng thái muốn đặt (STATE_IDLE, STATE_CHARGING, STATE_TESTING)
 */
void Channel_SetState(uint8_t channel_id, ChannelState_t new_state) {
    GPIO_TypeDef* charge_port;
    uint16_t      charge_pin;
    GPIO_TypeDef* load_port;
    uint16_t      load_pin;

    // Xác định đúng Port và Pin cho kênh được chọn
    switch(channel_id) {
        case 1:
            charge_port = CHARGE_1_PORT; charge_pin = CHARGE_1_PIN;
            load_port = LOAD_1_PORT; load_pin = LOAD_1_PIN;
            break;
        case 2:
            charge_port = CHARGE_2_PORT; charge_pin = CHARGE_2_PIN;
            load_port = LOAD_2_PORT; load_pin = LOAD_2_PIN;
            break;
        case 3:
            charge_port = CHARGE_3_PORT; charge_pin = CHARGE_3_PIN;
            load_port = LOAD_3_PORT; load_pin = LOAD_3_PIN;
            break;
        case 4:
            charge_port = CHARGE_4_PORT; charge_pin = CHARGE_4_PIN;
            load_port = LOAD_4_PORT; load_pin = LOAD_4_PIN;
            break;
        default:
            return; 
    }

    switch (new_state) {
        case STATE_IDLE:
            HAL_GPIO_WritePin(charge_port, charge_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(load_port, load_pin, GPIO_PIN_RESET);
            break;
            
        case STATE_CHARGING:
            HAL_GPIO_WritePin(load_port, load_pin, GPIO_PIN_RESET); 
            HAL_GPIO_WritePin(charge_port, charge_pin, GPIO_PIN_SET);
            break;
            
        case STATE_TESTING:
            HAL_GPIO_WritePin(charge_port, charge_pin, GPIO_PIN_RESET); 
            HAL_GPIO_WritePin(load_port, load_pin, GPIO_PIN_SET);
            break;
    }
}


/**
 * @brief Đặt tất cả các kênh về trạng thái IDLE (Tắt hết)
 */
void Channels_SetAll_Idle(void) {
    for (int i = 1; i <= 4; i++) {
        Channel_SetState(i, STATE_IDLE);
    }
}

void Channels_SetAll_Charge(void){
	for(int i = 1; i <= 4; i++){
		Channel_SetState(i, STATE_CHARGING);
	}
}

void Channels_SetAll_Testing(void){
	for(int i = 1; i <= 4; i++){
		Channel_SetState(i, STATE_TESTING);
	}
}

void Channels_SetAll(ChannelState_t trangThai_pin1, ChannelState_t trangThai_pin2, ChannelState_t trangThai_pin3, ChannelState_t trangThai_pin4){
	Channel_SetState(1, trangThai_pin1);
	Channel_SetState(2, trangThai_pin2);
	Channel_SetState(3, trangThai_pin3);
	Channel_SetState(4, trangThai_pin4);
}
