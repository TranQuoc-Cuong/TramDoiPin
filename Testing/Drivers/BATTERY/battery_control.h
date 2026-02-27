#ifndef INC_BATTERY_CONTROL_H_
#define INC_BATTERY_CONTROL_H_

#include "main.h"

#define MAX_CHANNEL_BATTERY 3 

typedef enum {
	STATE_IDLE,      
	STATE_CHARGING,  
	STATE_TESTING    
} ChannelState;

typedef struct {
	GPIO_TypeDef*  gpio_port_charge;
	uint16_t       gpio_pin_charge;
	
	GPIO_TypeDef*  gpio_port_load;
	uint16_t       gpio_pin_load;
	
	ChannelState   state;
	uint8_t        is_initialized;
} BatteryChannel;

void InitChannelBattery(uint8_t id, GPIO_TypeDef* port_charge, uint16_t pin_charge, 
																		GPIO_TypeDef* port_load, uint16_t pin_load);
void SetStateChannelBattery(uint8_t id, ChannelState state);
void SetIdleAllChannelBattery(void);
void SetChargeAllChannelBattery(void);
void SetLoadAllChannelBattery(void);
void SetAllStateChannelBattery(ChannelState state_1, ChannelState state_2, ChannelState state_3);

#endif /* INC_BATTERY_CONTROL_H_ */
