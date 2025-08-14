#include "ina219_manager.h"
#include "INA219.h" 

#define NUM_SENSORS 4
#define NUM_SAMPLES 5

static INA219_t ina_sensors[NUM_SENSORS];

static const uint8_t sensor_addresses[NUM_SENSORS] = {0x40, 0x41, 0x44, 0x45};

static const float LOW_BATTERY_THRESHOLD = 15.0f;


uint8_t INA219_Manager_Init(I2C_HandleTypeDef* hi2c) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (!INA219_Init(&ina_sensors[i], hi2c, sensor_addresses[i])) {
            return 0; 
        }
    }
    return 1; 
}

INA219_Data_t INA219_Manager_Read(uint8_t sensor_id) {
    INA219_Data_t data = {0}; 
		
    if (sensor_id < 1 || sensor_id > NUM_SENSORS) {
        return data; 
    }

    INA219_t* sensor_handle = &ina_sensors[sensor_id - 1];
		
		float total_voltage = 0;
    float total_current = 0;
		
		for (int i = 0; i < NUM_SAMPLES; i++) {
        total_voltage += INA219_ReadBusVoltage(sensor_handle);
        total_current += INA219_ReadCurrent(sensor_handle);
        HAL_Delay(5); 
    }
		
		// Tính trung bình
    data.voltage = (total_voltage / NUM_SAMPLES); 
    data.current = total_current / NUM_SAMPLES;
    
    data.power = data.voltage * data.current;
    data.soc_percent = INA219_GetBatteryLife(sensor_handle, 4200.0f, 2900.0f);
    data.state = INA219_HealthCheck(sensor_handle, 15.0f, data.soc_percent);
    return data;
}
