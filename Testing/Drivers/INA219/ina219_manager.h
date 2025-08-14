#ifndef INC_INA219_MANAGER_H_
#define INC_INA219_MANAGER_H_

#include "main.h"
#include "INA219.h"


typedef struct {
    float voltage;
    float current;
    float power;
		float soc_percent;
		enum BatteryState state; 
} INA219_Data_t;

uint8_t INA219_Manager_Init(I2C_HandleTypeDef* hi2c);

INA219_Data_t INA219_Manager_Read(uint8_t sensor_id);

#endif // INC_INA219_MANAGER_H_ 
