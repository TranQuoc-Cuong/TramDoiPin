#ifndef INC_BATTERY_DIAGNOSTICS_H_
#define INC_BATTERY_DIAGNOSTICS_H_

#include "main.h"

typedef enum {
    BATT_HEALTHY,        
    BATT_DEGRADED,        
    BATT_DEAD,            
    BATT_UNSAFE_TEMP,     
    BATT_UNKNOWN_ERROR    
} BatteryHealth_t;



BatteryHealth_t Diagnose_Check(uint8_t channel_id, float temperature);

BatteryHealth_t Diagnose_RunIRTest(uint8_t channel_id);

#endif /* INC_BATTERY_DIAGNOSTICS_H_ */
