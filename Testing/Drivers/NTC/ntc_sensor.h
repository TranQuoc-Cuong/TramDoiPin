#ifndef INC_NTC_SENSOR_H_
#define INC_NTC_SENSOR_H_

#include "main.h"

#define NTC_ERROR_VAL -273.0f

float GetTemperatureNTC(ADC_HandleTypeDef* hadc, uint32_t adc_channel);

#endif // INC_NTC_SENSOR_H_
