#ifndef INC_INA219_MANAGER_H_
#define INC_INA219_MANAGER_H_

#include "INA219.h"

typedef struct {
	float voltage;
	float current;
	float power;
	float soc_percent;
	enum BatteryState state; 
} INA219Data;

void SetAddressManagerINA219(uint8_t id, uint8_t address);
uint8_t InitManagerINA219(I2C_HandleTypeDef* hi2c);
INA219Data ReadManagerINA219(uint8_t sensor_id);

#endif // INC_INA219_MANAGER_H_ 
