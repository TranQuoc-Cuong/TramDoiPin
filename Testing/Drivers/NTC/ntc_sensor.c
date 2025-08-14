#include "ntc_sensor.h"
#include <math.h>

const float B_COEFF = 3435.0f;        
const float V_REF   = 3.3f;       

const float T0_KELVIN = 298.15f; 
const float R0_OHMS   = 10000.0f;  


float NTC_GetTemperature(ADC_HandleTypeDef* hadc, uint32_t adc_channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = adc_channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
		
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 1000);
    uint16_t adc_value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    float v_out = (float)adc_value / 4095.0f * V_REF;
    
    if (v_out >= V_REF) { 
        return -273.15f;
    }
    if (v_out == 0) { 
        return -273.15f;
    }

    float r_ntc = R0_OHMS * (3.3f - v_out) / v_out;
    
    if (r_ntc <= 0) {
        return -273.15f; 
    }

    float temp_K = 1.0f / ( (1.0f/T0_KELVIN) + (1.0f/B_COEFF) * logf(r_ntc / R0_OHMS) );
    float temp_C = temp_K - 273.15f;
    
    return temp_C;
}
