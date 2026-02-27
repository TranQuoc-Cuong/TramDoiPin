/*
 * INA219.c
 *
 *  Created on: Dec 30, 2020
 *  Updated on: Jan 29, 2022
 *       Author: Piotr Smolen <komuch@gmail.com>
 *       Updated by: Brandon Thibeaux <github: thibeaux>
 */

#include "INA219.h"

#include "main.h"

static uint16_t ina219_calibrationValue = 0;
static int16_t  ina219_currentDivider_mA = 0;
static float    ina219_powerMultiplier_mW = 0.0f;

/*
 * @brief:		Read a register from the IN219 sensor.
 * @param:		Pointer to the device object that was made from the struct. EX:  (&ina219)
 * @param:		register address in hexadecimal
 * @retval:		16 bit unsigned integer that represents the register's contents.
 */
static uint16_t Read16(INA219_t *ina219, uint8_t Register) {
	uint8_t Value[2];

	HAL_I2C_Mem_Read(ina219->ina219_i2c, (INA219_ADDRESS << 1), Register, 1, Value, 2, 1000);

	return ((Value[0] << 8) | Value[1]);
}

/*
 * @brief:		Write to a register on the IN219 sensor.
 * @param:		Pointer to the device object that was made from the struct. EX:  (&ina219)
 * @param:		Register address in hexadecimal
 * @param:		16 bit integer in hexadecimal that is the value you want to write to the register.
 * @retval:		HAL_StatusTypeDef, this will include an enum value representing
 * 				if the I2C transmission was successful or not.
 * 				typedef enum
				{
				  HAL_OK       = 0x00U,
				  HAL_ERROR    = 0x01U,
				  HAL_BUSY     = 0x02U,
				  HAL_TIMEOUT  = 0x03U
				} HAL_StatusTypeDef;
 */
static HAL_StatusTypeDef Write16(INA219_t *ina219, uint8_t Register, uint16_t Value) {
	uint8_t addr[2];
	addr[0] = (Value >> 8) & 0xff;  // upper byte
	addr[1] = (Value >> 0) & 0xff; // lower byte
	
	return HAL_I2C_Mem_Write(ina219->ina219_i2c, (INA219_ADDRESS << 1), Register, 1, (uint8_t*)addr, 2, 1000);
}

/*
 * @brief: 		This function will read the battery voltage level being read.
 * @param:		Pointer to the device object that was made from the struct. EX:  (&ina219)
 * @retval:		Returns voltage level in mili-volts
 */
uint16_t INA219_ReadBusVoltage(INA219_t *ina219) {
	uint16_t result = Read16(ina219, INA219_REG_BUSVOLTAGE);

	return ((result >> 3) * 4);
}

/*
 *  @brief:	  	Gets the raw current value (16-bit signed integer, so +-32767)
 *  @param:		Pointer to the device object that was made from the struct. EX:  (&ina219)
 *  @retval:	The raw current reading
 */
int16_t INA219_ReadCurrent_raw(INA219_t *ina219) {
	int16_t result = Read16(ina219, INA219_REG_CURRENT);

	return (result);
}

/*
 * @brief:  	Gets the current value in mA, taking into account the
 *          	config settings and current LSB
 * @param:		Pointer to the device object that was made from the struct. EX:  (&ina219)
 * @return: 	The current reading convereted to milliamps
 */
int16_t INA219_ReadCurrent(INA219_t *ina219) {
	int16_t result = INA219_ReadCurrent_raw(ina219);

	return (result / ina219_currentDivider_mA);
}

/*
 * @brief: 		This function will read the shunt voltage level.
 * @param:		Pointer to the device object that was made from the struct. EX:  (&ina219)
 * @retval:		Returns voltage level in mili-volts. This value represents the difference
 * 				between the voltage of the power supply and the bus voltage after the shunt
 * 				resistor.
 */
uint16_t INA219_ReadShuntVolage(INA219_t *ina219) {
	uint16_t result = Read16(ina219, INA219_REG_SHUNTVOLTAGE);

	return (result * 0.01);
}
/*
 * @brief: 	This reads the power register then multiplies it by the power multiplier.
 * 			Power multiplier is initialize in the calibration function.
 * @param:	Pointer to the device object that was made from the struct. EX:  (&ina219)
 * @retval:	Returns power level in mili-watts
 */
uint16_t INA219_ReadPower(INA219_t *ina219) {
	uint16_t result = Read16(ina219, INA219_REG_POWER);

	result = result * ina219_powerMultiplier_mW; // power is the power register times the power_LSB (power multiplier)

	return (result);
}

/*
 * @brief:		This takes a minimum and maximum value an turn the current voltage
 * 				level into a percentage to give you a reference on how much battery life is left on your device.
 * @param:	<object pointer> <maximum mili-voltage level> <minumum mili-voltage level>
 * @retval: 	Percentage floating point value
 * @example: 	GetBatteryLife(&ina219, 6000, 4000)
 * 				returns 75.02%
 */
float INA219_GetBatteryLife(INA219_t *ina219, float batteryMax, float batteryMin) {
	float  percentageLife = 0.0f;
	
	uint16_t vbus = INA219_ReadBusVoltage(ina219);

	percentageLife = (vbus - batteryMin) / (batteryMax - batteryMin);

	if(percentageLife >= 0) {
		
		return percentageLife * 100;
	} else {
		
		return 0.0f;
	}
}

/*
 * @brief:		The goal of this function is to make sure the battery is working as we
 * 				specify it to. If there is a discrepancy that is found in this check we
 * 				update the battery's state to alert the main function to take action.
 * @param:		Pointer to the device object that was made from the struct. EX:  (&ina219)
 * @param:		Floating point value for the percent of battery life we want to
 * 				dedicate as a threshold that indicates a "LOW" state
 * @param:		Floating point value that represents out batteries current life percentage ratio
 * @retval:		We will return an enum state that represents the battery's state. This tells
 * 				the program that called the health check function what state our battery is
 * 				at and whether we have entered a "LOW" state. This way the program can take
 * 				appropriate action.
 */
enum BatteryState INA219_HealthCheck(INA219_t *ina219, float batteryPercentageThreshold, float batteryPercentage) {
	switch(ina219->battery_state) {
		case (Battery_START):
			/* Enter your start up functionality here */
			ina219->battery_state = Battery_OK;
			break;

		case (Battery_OK):
			/* Enter your battery OK state functionality here */
			if(batteryPercentage > batteryPercentageThreshold) { // is battery life below given threshold?
				ina219->battery_state = Battery_OK;
			} else {
				ina219->battery_state = Battery_LOW;
			}
			break;

		case (Battery_LOW):
			/* Enter your battery LOW state functionality here */
			if(batteryPercentage > batteryPercentageThreshold) { // is battery life below given threshold?
				ina219->battery_state = Battery_OK;
			} else {
				ina219->battery_state = Battery_LOW;
			}
			break;

		default:
			/*
			 * If program encounters a bug or a value outside what is expected we go here.
			 * Feel free to add functionality if needed.
			*/
			ina219->battery_state = Battery_START;
			break;
	}

	return ina219->battery_state;
}

static void INA219_Reset(INA219_t *ina219) {
	Write16(ina219, INA219_REG_CONFIG, INA219_CONFIG_RESET);
	HAL_Delay(1);
}

static void INA219_setCalibration(INA219_t *ina219, uint16_t CalibrationData) {
	Write16(ina219, INA219_REG_CALIBRATION, CalibrationData);
}

static uint16_t INA219_getConfig(INA219_t *ina219) {
	uint16_t result = Read16(ina219, INA219_REG_CONFIG);

	return result;
}

static void INA219_setConfig(INA219_t *ina219, uint16_t Config) {
	Write16(ina219, INA219_REG_CONFIG, Config);
}

void INA219_setCalibration_32V_Average(INA219_t *ina219) {
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	             INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT_128S_69MS |
	             INA219_CONFIG_SADCRES_12BIT_128S_69MS |
	             INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	ina219_calibrationValue = 4096;
	ina219_currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
	ina219_powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

	INA219_setCalibration(ina219, ina219_calibrationValue);
	INA219_setConfig(ina219, config);
}

void INA219_setCalibration_32V_2A(INA219_t *ina219) {
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	             INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
	             INA219_CONFIG_SADCRES_12BIT_1S_532US |
	             INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	ina219_calibrationValue = 4096;
	ina219_currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
	ina219_powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

	INA219_setCalibration(ina219, ina219_calibrationValue);
	INA219_setConfig(ina219, config);
}

void INA219_setCalibration_32V_1A(INA219_t *ina219)
{
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
	                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
	                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	ina219_calibrationValue = 10240;
	ina219_currentDivider_mA = 25;    // Current LSB = 40uA per bit (1000/40 = 25)
	ina219_powerMultiplier_mW = 0.8f; // Power LSB = 800uW per bit

	INA219_setCalibration(ina219, ina219_calibrationValue);
	INA219_setConfig(ina219, config);
}

void INA219_setCalibration_16V_400mA(INA219_t *ina219)
{
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
	                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
	                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
	                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	ina219_calibrationValue = 8192;
	ina219_currentDivider_mA = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
	ina219_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

	INA219_setCalibration(ina219, ina219_calibrationValue);
	INA219_setConfig(ina219, config);
}

void INA219_setPowerMode(INA219_t *ina219, uint8_t Mode) {
	uint16_t config = INA219_getConfig(ina219);

	switch (Mode) {
		case INA219_CONFIG_MODE_POWERDOWN:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_POWERDOWN & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(ina219, config);
			break;

		case INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(ina219, config);
			break;

		case INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(ina219, config);
			break;

		case INA219_CONFIG_MODE_ADCOFF:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_ADCOFF & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(ina219, config);
			break;
	}
}

uint8_t INA219_Init(INA219_t *ina219, I2C_HandleTypeDef *i2c, uint8_t Address, INA219_CalibrationCallback calFunc) {
	ina219->ina219_i2c = i2c;
	ina219->Address = Address;

	ina219_calibrationValue = 0;
	ina219_currentDivider_mA = 0;
	ina219_powerMultiplier_mW = 0;

	uint8_t ina219_isReady = HAL_I2C_IsDeviceReady(i2c, (Address << 1), 3, 2);

	if(ina219_isReady == HAL_OK) {
		ina219->battery_state = Battery_START;
		INA219_HealthCheck(ina219, 0.0f, 1.0f);
		INA219_Reset(ina219);

		if (calFunc != NULL) {
			calFunc(ina219);
		} else {
			INA219_setCalibration_32V_Average(ina219);
		}

		return 1;
	} else {

		return 0;
	}
}
