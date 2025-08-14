#include "battery_diagnostics.h"
#include "ina219_manager.h"
#include "battery_control.h"

#define VOLTAGE_DEAD_THRESHOLD  2.8f  
#define IR_DEGRADED_THRESHOLD   100.0f // Trên 100mOhm là pin chai
#define TEMP_HIGH_THRESHOLD     60.0f  // Trên 60 do C là nguy hiem

BatteryHealth_t Diagnose_Check(uint8_t channel_id, float temperature) {
    if (temperature > TEMP_HIGH_THRESHOLD) {
        return BATT_UNSAFE_TEMP;
    }
		
    INA219_Data_t data = INA219_Manager_Read(channel_id);
    if (data.voltage < VOLTAGE_DEAD_THRESHOLD) {
        return BATT_DEAD;
    }

    return BATT_HEALTHY;
}

BatteryHealth_t Diagnose_RunIRTest(uint8_t channel_id) {
    float v_open;
		float v_load;
		float i_load;
		float voltage_drop;
		float ir_milliohms;
		Channel_SetState(channel_id, STATE_IDLE);
    HAL_Delay(100);
    INA219_Data_t data_before = INA219_Manager_Read(channel_id);
    Channel_SetState(channel_id, STATE_TESTING);
    HAL_Delay(200);
    INA219_Data_t data_during = INA219_Manager_Read(channel_id);
    Channel_SetState(channel_id, STATE_IDLE); 

    v_open = data_before.voltage;
    v_load = data_during.voltage;
    i_load = data_during.current * (-1.0f);; 
 
    voltage_drop = v_open - v_load;
    ir_milliohms = ((voltage_drop / i_load)) * (1000.0f)/  (10.0f);

    if (ir_milliohms > IR_DEGRADED_THRESHOLD) {
      return BATT_DEGRADED;
    } else {
        return BATT_HEALTHY;
    }

    return BATT_UNKNOWN_ERROR;
}
