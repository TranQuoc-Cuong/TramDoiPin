#include "battery_diagnostics.h"

#include <math.h>

#include "battery_control.h"
#include "ina219_manager.h"
#include "soft_timer.h"
#include "servo_control.h"

#define VOLTAGE_DEAD_THRESHOLD  2800.0f 
#define IR_DEGRADED_THRESHOLD   200.0f 
#define TEMP_HIGH_THRESHOLD     60.0f  

BatteryHealth CheckDiagnose(uint8_t channel_id, float temperature) {
	if (temperature > TEMP_HIGH_THRESHOLD) {
		
		return BATT_UNSAFE_TEMP;
	}
	
	INA219Data data = ReadManagerINA219(channel_id);
	if (data.voltage < VOLTAGE_DEAD_THRESHOLD) {
		
		return BATT_DEAD;
	}

	return BATT_HEALTHY;
}

BatteryHealth RunIRTestDiagnose(uint8_t channel_id) {
	float v_open;
	float v_load;
	float i_load;
	float voltage_drop;
	float ir_milliohms;
	
	INA219Data data_during;
	
	SetStateChannelBattery(channel_id, STATE_IDLE);
	
	
	INA219Data data_before = ReadManagerINA219(channel_id);
	v_open = data_before.voltage;
	
	SetStateChannelBattery(channel_id, STATE_TESTING);
	HAL_Delay(100);
	
	data_during = ReadManagerINA219(channel_id);
	
	SetStateChannelBattery(channel_id, STATE_IDLE); 
	HAL_Delay(500);

	v_load = data_during.voltage;
	i_load = fabs(data_during.current);

	voltage_drop = v_open - v_load;
	ir_milliohms = ((voltage_drop / i_load)) * (1000.0f) / (10.0f);

	if (ir_milliohms > IR_DEGRADED_THRESHOLD) {
		
		return BATT_DEGRADED;
	} else {
		
		return BATT_HEALTHY;
	}

	//return BATT_UNKNOWN_ERROR;
}
