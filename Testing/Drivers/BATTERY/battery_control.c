#include "battery_control.h"

/*
	Sac pin
	GPIOB_13 pin1
	GPIOB_15 pin2
	GPIOA_7 pin3
*/

/*
	Gan tai
	GPIOB_12 pin1
	GPIOB_14 pin2
	GPIOB_0 pin3
*/

BatteryChannel battery_channel[MAX_CHANNEL_BATTERY] = {0};

/**
 * @brief Set state of channel battery.
 * @param id: The number of channel (1, 2, 3)
 * @param state: state which you want (STATE_IDLE, STATE_CHARGING, STATE_TESTING)
 **/
void SetStateChannelBattery(uint8_t id, ChannelState state) {
	if ((id < 1) || (id > MAX_CHANNEL_BATTERY)) {
		
		return;
	}
	
	uint8_t index = id - 1;
	
	if (battery_channel[index].is_initialized == 0) {
		
		return; 
  }
	
	switch (state) {
		case STATE_IDLE:
			battery_channel[index].state = state;
	
			HAL_GPIO_WritePin(battery_channel[index].gpio_port_charge, battery_channel[index].gpio_pin_charge, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(battery_channel[index].gpio_port_load, battery_channel[index].gpio_pin_load, GPIO_PIN_RESET);
			break;
				
		case STATE_CHARGING:
			battery_channel[index].state = state;
	
			HAL_GPIO_WritePin(battery_channel[index].gpio_port_charge, battery_channel[index].gpio_pin_charge, GPIO_PIN_SET);
			HAL_GPIO_WritePin(battery_channel[index].gpio_port_load, battery_channel[index].gpio_pin_load, GPIO_PIN_RESET);
			break;
				
		case STATE_TESTING:
			battery_channel[index].state = state;
	
			HAL_GPIO_WritePin(battery_channel[index].gpio_port_charge, battery_channel[index].gpio_pin_charge, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(battery_channel[index].gpio_port_load, battery_channel[index].gpio_pin_load, GPIO_PIN_SET);
			break;
	}
}

/**
 * @brief Set state for each channel battery.
 * @param state_1: state battery of channel 1 (STATE_IDLE, STATE_CHARGING, STATE_TESTING).
 * @param state_2: state battery of channel 2 (STATE_IDLE, STATE_CHARGING, STATE_TESTING).
 * @param state_3: state battery of channel 3 (STATE_IDLE, STATE_CHARGING, STATE_TESTING).
 **/
void SetAllStateChannelBattery(ChannelState state_1, ChannelState state_2, ChannelState state_3) {
	SetStateChannelBattery(1, state_1);
	SetStateChannelBattery(2, state_2);
	SetStateChannelBattery(3, state_3);
}

/**
 * @brief Sett all idle state for all channel batterys.
 * @param 
 **/
void SetIdleAllChannelBattery(void) {
	SetAllStateChannelBattery(STATE_IDLE, STATE_IDLE, STATE_IDLE);
}

/**
 * @brief Sett all charge state for all channel batterys.
 * @param 
 **/
void SetChargeAllChannelBattery(void) {
	SetAllStateChannelBattery(STATE_CHARGING, STATE_CHARGING, STATE_CHARGING);
}

/**
 * @brief Set all load state for all channel batterys.
 * @param 
 **/
void SetLoadAllChannelBattery(void) {
	SetAllStateChannelBattery(STATE_TESTING, STATE_TESTING, STATE_TESTING);
}

/**
 * @brief Initialing id, port,pin of charge and port, pin of load for channel battery. 
 * @param id: The id for channel battery, which help you choose channel (1, 2, 3).
 * @param port_charge: The port of GPIO for charge (GPIOA, GPIOB, ...).
 * @param pin_charge: The pin of GPIO for charge (GPIO0, GPIO1, ...).
 * @param port_load: The port of GPIO for load (GPIOA, GPIOB, ...).
 * @param pin_load: The pin of GPIO for load (GPIO0, GPIO1, ...).
 * 
 **/
void InitChannelBattery(uint8_t id, GPIO_TypeDef* port_charge, uint16_t pin_charge, 
																		GPIO_TypeDef* port_load, uint16_t pin_load) {
	if ((id < 1) || (id > MAX_CHANNEL_BATTERY)) {
		
		return;
	}
	
	uint8_t index = id - 1;
	
	battery_channel[index].gpio_port_charge = port_charge;
	battery_channel[index].gpio_pin_charge = pin_charge;

	battery_channel[index].gpio_port_load = port_load;
	battery_channel[index].gpio_pin_load = pin_load;

	battery_channel[index].state = STATE_IDLE;
	battery_channel[index].is_initialized = 1;
																			
	SetStateChannelBattery(id, STATE_IDLE);
}
