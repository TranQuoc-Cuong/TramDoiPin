#include "ina219_manager.h"

#include "main.h"

#define NUM_SENSORS 3

static INA219_t ina_sensors[NUM_SENSORS];
static uint8_t sensor_addresses[NUM_SENSORS];
//static const float LOW_BATTERY_THRESHOLD = 15.0f;

void SetAddressManagerINA219(uint8_t id, uint8_t address) {
	if (id < 1 || id > NUM_SENSORS) {
		
		return; 
	}
	
	uint8_t array_index = id - 1;
	
	sensor_addresses[array_index] = address;
}

uint8_t InitManagerINA219(I2C_HandleTypeDef* hi2c) {
	uint8_t result = 1;
	
	for (int i = 0; i < NUM_SENSORS; i++) {
		if (0 == INA219_Init(&ina_sensors[i], hi2c, sensor_addresses[i], INA219_setCalibration_32V_Average)) {
			result = 0; 
		}
	}
	
	return result; 
}

INA219Data ReadManagerINA219(uint8_t sensor_id) {
	INA219Data data = {0}; 
	
	if (sensor_id < 1 || sensor_id > NUM_SENSORS) {
		
		return data; 
	}

	INA219_t* sensor_handle = &ina_sensors[sensor_id - 1];
	
	data.voltage = INA219_ReadBusVoltage(sensor_handle);
	data.current = INA219_ReadCurrent(sensor_handle);
		
	data.power = INA219_ReadPower(sensor_handle);
	data.soc_percent = INA219_GetBatteryLife(sensor_handle, 4200.0f, 2900.0f);
	data.state = INA219_HealthCheck(sensor_handle, 15.0f, data.soc_percent);
	
	return data;
}
