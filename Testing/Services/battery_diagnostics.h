#ifndef INC_BATTERY_DIAGNOSTICS_H_
#define INC_BATTERY_DIAGNOSTICS_H_

#include <stdint.h>

typedef enum {
	BATT_HEALTHY,        
	BATT_DEGRADED,        
	BATT_DEAD,            
	BATT_UNSAFE_TEMP,     
	BATT_UNKNOWN_ERROR    
} BatteryHealth;

BatteryHealth CheckDiagnose(uint8_t channel_id, float temperature);
BatteryHealth RunIRTestDiagnose(uint8_t channel_id);

#endif /* INC_BATTERY_DIAGNOSTICS_H_ */
