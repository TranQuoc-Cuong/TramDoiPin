#ifndef INC_BATTERY_CONTROL_H_
#define INC_BATTERY_CONTROL_H_

#include "main.h"

// Định nghĩa 3 trạng thái bạn yêu cầu
typedef enum {
    STATE_IDLE,      // Trạng thái 1: Tắt sạc, Tắt tải
    STATE_CHARGING,  // Trạng thái 2: Bật sạc, Tắt tải
    STATE_TESTING    // Trạng thái 3: Tắt sạc, Bật tải
} ChannelState_t;

// Khai báo hàm để main.c có thể gọi
void Channel_SetState(uint8_t channel_id, ChannelState_t new_state);
void Channels_SetAll_Idle(void);
void Channels_SetAll_Charge(void);
void Channels_SetAll_Testing(void);
void Channels_SetAll(ChannelState_t trangThai_pin1, ChannelState_t trangThai_pin2, ChannelState_t trangThai_pin3, ChannelState_t trangThai_pin4);

#endif /* INC_BATTERY_CONTROL_H_ */
