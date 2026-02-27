/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "battery_control.h"
#include "servo_control.h"
#include "ina219_manager.h"
#include "ntc_sensor.h"
#include "handle_cmd.h"
#include "uart_dn.h"
#include "battery_diagnostics.h"
#include "soft_timer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
const float threshold_pin_empty = 800.0f;
const float threshold_temperature = 55.0f;
const uint8_t angle_open = 180U;
const uint8_t angle_close = 40U;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const uint32_t ADC_Channel[] = {
	ADC_CHANNEL_0,
	ADC_CHANNEL_1,
	ADC_CHANNEL_2,
};

SoftTimer timer_sensor;

volatile SystemState status_machine = IDLE_WAIT_CMD;

PinChannel pin_channels[3] = {0};

uint8_t id_pin_percent_max = 0;
uint8_t id_pin_empty = 0;

volatile int8_t esp32_pin_check_status = -1; 
uint32_t timestamp_gui_checkpin = 0; // tý coi nó là gì sua nó lai

#define ESP32_CHECK_TIMEOUT 5000    // này n?a    

#define CHARGE_FULL_COUNTER_LIMIT  20

uint8_t id_pin_dang_kiem_tra = 4; 

const char* get_channel_state_string(StatusChannel state) {
	switch(state) {
		case pin_empty:       return "T";
		case pin_charge:  		return "S";
		case pin_full: 		    return "D";
		case pin_over_heat:   return "N";
		case pin_error:       return "L";
		default:              return "R";
	}
}

uint8_t percent[3];

void ReportStatusTask(void) {
	char report_buffer[50];

	for (int i = 0; i < 3; i++) {
	if(pin_channels[i].status == pin_empty) {
		percent[i] = 0;
		pin_channels[i].check_percent_pin = 0;
	}
	
	if(pin_channels[i].status == pin_full) {
		percent[i] = 100;
		pin_channels[i].check_percent_pin = 0;
	}
	
	if(pin_channels[i].check_percent_pin == 1) {
		percent[i] = pin_channels[i].data_pin.soc_percent;
		pin_channels[i].check_percent_pin = 0;
	}
	
	sprintf(report_buffer, "P:%d,%%:%d,C:%s,V:%.2f,I:%.2f,T:%.1f\n",
							i + 1,
							percent[i],
							get_channel_state_string(pin_channels[i].status),
							pin_channels[i].data_pin.voltage,
							pin_channels[i].data_pin.current,
							pin_channels[i].temperatures);
	
	TransmitStringHandlerUART(report_buffer);
	}
	
	TransmitStringHandlerUART("\n");
}

volatile uint8_t fit_pin_remove = 4;
volatile uint8_t fit_pin_initial = 4;

void CheckPinNewInitial(uint8_t id, PinChannel* id_channel) {
	if (id_channel->data_pin.voltage > threshold_pin_empty) {
		BatteryHealth health_status = CheckDiagnose(id + 1, id_channel->temperatures);
		
		if (health_status == BATT_UNSAFE_TEMP || health_status == BATT_DEAD) {
			id_channel->status = pin_error;
		} else {
				health_status = RunIRTestDiagnose(id + 1);
			
				if (health_status == BATT_HEALTHY) {
					
					if (id_channel->data_pin.soc_percent >= 98.0f) {
						id_channel->status = pin_full;
						
						if(status_machine == ALL_SLOTS_EMPTY) status_machine = IDLE_WAIT_CMD;
					} else {
								id_channel->status = pin_charge;
							
								if(status_machine == ALL_SLOTS_EMPTY) status_machine = IDLE_WAIT_CMD;
							
								id_channel->check_percent_pin = 1;
								id_channel->last_check_percent = HAL_GetTick();
					}
				} else {
						id_channel->status = pin_error;
				}
		}
	}
}


void HandlerStateCharge(PinChannel* channel) {
	
	if (channel->temperatures <= (NTC_ERROR_VAL + 1.0f)) { 
		channel->status = pin_error;
		return; 
	}
	
	if (channel->temperatures > threshold_temperature || channel->temperatures < 5.0f) { 
		channel->status = pin_over_heat; 
		
		return; 
	}
	
	if ((channel->data_pin.current < 1.6f && channel->data_pin.current > -2.3f) 
	|| (channel->data_pin.current < 0.1f && channel->data_pin.current > -1.0f)) {
		if (channel->charge_done_counter == 0) {
			channel->charge_done_timestamp = HAL_GetTick();
		}
			channel->charge_done_counter++;

			if (channel->charge_done_counter >= CHARGE_FULL_COUNTER_LIMIT) {
					if (channel->data_pin.current < 0.1f && channel->data_pin.current > -1.0f) {
							channel->status = pin_empty;
					} else {
							channel->status = pin_full;
					}
					channel->charge_done_counter = 0; 
			}
	} else {
			channel->charge_done_counter = 0;
	}
	
	if (channel->charge_done_counter > 0 && channel->charge_done_counter < 3) {
			if (HAL_GetTick() - channel->charge_done_timestamp > 6000) {
					channel->charge_done_counter = 0;
			}
	}
}

//Initial All Pin in Battery control
void InitialAllPinBattery(void) {
	InitChannelBattery(1, GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_12);
	InitChannelBattery(2, GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_14);
	InitChannelBattery(3, GPIOA, GPIO_PIN_7, GPIOB, GPIO_PIN_0);
}

//Initial Servo Pin
void InitialAllPinServo(void) {
	InitServo(1, &htim2, TIM_CHANNEL_1);
	InitServo(2, &htim2, TIM_CHANNEL_2);
	InitServo(3, &htim2, TIM_CHANNEL_3);
}

void InitialAllINA219(void) {
	SetAddressManagerINA219(1, 0x40);
	SetAddressManagerINA219(2, 0x41);
	SetAddressManagerINA219(3, 0x44);
}

void InitialStateBeginChannelBattery(void) {
	for(uint8_t i = 0; i < 3; i++) {
		pin_channels[i].status = pin_empty;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	
	InitHandlerUART(&huart1);
	
	InitialAllPinBattery();
	SetIdleAllChannelBattery();
	
	InitialAllPinServo();
	OpenAllServo(angle_open);
	
	InitialStateBeginChannelBattery();
	
	InitialAllINA219();
	if (0 == InitManagerINA219(&hi2c1)) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
	
	char rx_line_buffer[100];
	char cmd_buffer[100];     
	
	StartSoftTimer(&timer_sensor, 500);
	
	TransmitStringHandlerUART("He thong khoi dong");
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		RunManagerServo();
		
    if (GetLineHandlerUART(rx_line_buffer, 100)) {
			strcpy(cmd_buffer, rx_line_buffer);
			CMDProcessAll(cmd_buffer);
			TransmitStringHandlerUART("ok\r\n");
    }
		
		if (IsExpiredSoftTimer(&timer_sensor)) {
			for(uint8_t id = 0; id < 3; id++){
					pin_channels[id].temperatures = GetTemperatureNTC(&hadc1, ADC_Channel[id]);
					pin_channels[id].data_pin = ReadManagerINA219(id + 1);
			}

			uint8_t temp_max_percent = 0;
			uint8_t found_empty_slot = 0;
			id_pin_percent_max = 4; 
			id_pin_empty = 4;    
			
			if(status_machine == IDLE_WAIT_CMD) status_machine = ALL_SLOTS_EMPTY;
			for (int i = 0; i < 3; i++) {
				if(pin_channels[i].status == pin_empty) {
					percent[i] = 0;
				} else if(pin_channels[i].status == pin_full) {
					if(status_machine == ALL_SLOTS_EMPTY) status_machine = IDLE_WAIT_CMD;
					percent[i] = 100;
				} else if(pin_channels[i].check_percent_pin == 1) { 
					percent[i] = pin_channels[i].data_pin.soc_percent;
					pin_channels[i].check_percent_pin = 0;
				}

				if ((pin_channels[i].status == pin_charge || pin_channels[i].status == pin_full) && percent[i] >= temp_max_percent) {
					if(status_machine == ALL_SLOTS_EMPTY) status_machine = IDLE_WAIT_CMD;
					temp_max_percent = percent[i];
					id_pin_percent_max = i;
				}

				if ((pin_channels[i].status == pin_empty) &&  (0 == found_empty_slot)) {
					id_pin_empty = i;
					found_empty_slot = 1;
				}
			}			
			ReportStatusTask();
		}

		switch (status_machine) {				
			case INSTALL_BATTERY:
				for(uint8_t i = 0; i < 3; i++){
					if(pin_channels[i].status == pin_empty || pin_channels[i].status == pin_error){
						SetAngleServo(i + 1, angle_open);
						
						if((pin_channels[i].status == pin_error) && (pin_channels[i].data_pin.voltage < threshold_pin_empty)) {
								pin_channels[i].status = pin_empty;
						}
						
						if(pin_channels[i].data_pin.voltage > threshold_pin_empty){
							esp32_pin_check_status = -1;
							timestamp_gui_checkpin = HAL_GetTick();
							id_pin_dang_kiem_tra = i;
							
							TransmitStringHandlerUART("checkpin\n");
							
							status_machine = INSTALL_WAIT_ESP_PROCESS;
							break;
						}
					}
				}
				break;
				
			case ALL_SLOTS_EMPTY:
				OpenAllServo(angle_open); 
			
				for(uint8_t i = 0; i < 3; i++){
					SetStateChannelBattery(i + 1, STATE_IDLE);
					
					if(pin_channels[i].data_pin.voltage > threshold_pin_empty){
						esp32_pin_check_status = -1;
						timestamp_gui_checkpin = HAL_GetTick();
						id_pin_dang_kiem_tra = i;
						
						TransmitStringHandlerUART("checkpin\n");
						
						status_machine = INSTALL_WAIT_ESP_PROCESS;
					}
				}
				break;
				
			case IDLE_WAIT_CMD:
				OpenAllServo(angle_open); 
			
				for (uint8_t i = 0; i < 3; i++) {
					switch(pin_channels[i].status) {
						case pin_empty:
							SetStateChannelBattery(i + 1, STATE_IDLE);
							if(pin_channels[i].data_pin.voltage > threshold_pin_empty){
								esp32_pin_check_status = -1;
								timestamp_gui_checkpin = HAL_GetTick();
								id_pin_dang_kiem_tra = i;
								
								TransmitStringHandlerUART("checkpin\n");
								
								status_machine = INSTALL_WAIT_ESP_PROCESS;
							}
							break;
			
						case pin_charge:
							HandlerStateCharge(&(pin_channels[i]));
							SetStateChannelBattery(i + 1, STATE_CHARGING);
							break;
						
						case pin_over_heat:
							if(pin_channels[i].temperatures < threshold_temperature - 5) {
								pin_channels[i].status = pin_charge;
							}
							SetStateChannelBattery(i + 1, STATE_IDLE);
							break;
							
						case pin_full:
						case pin_error:
							if(pin_channels[i].data_pin.voltage < threshold_pin_empty) {
								pin_channels[i].status = pin_empty;
							}
							SetStateChannelBattery(i + 1, STATE_IDLE);
							break;
			
						default:
						
						break;
					}
				}
			break;
				
			case WAIT_BATTERY_REMOVAL:
					SetAngleServo(fit_pin_remove + 1, angle_open);
					SetStateChannelBattery(fit_pin_remove + 1, STATE_IDLE);
			
					if (pin_channels[fit_pin_remove].data_pin.voltage < threshold_pin_empty) {
							pin_channels[fit_pin_remove].status = pin_empty;
							SetAngleServo(fit_pin_remove + 1, angle_close); 
							fit_pin_remove = 4; 
							status_machine = IDLE_WAIT_CMD; 
					}
					break;

			case SWAP_WAIT_INSERTION:
					for(uint8_t i = 0; i < 3; i++)
					{
						if(i == fit_pin_initial){
							SetAngleServo(fit_pin_initial + 1, angle_open);
						}
						else{
							SetAngleServo(i + 1, angle_close);
						}
					}
					
					if (pin_channels[fit_pin_initial].data_pin.voltage > threshold_pin_empty) {
							esp32_pin_check_status = -1;
							timestamp_gui_checkpin = HAL_GetTick();
							
							TransmitStringHandlerUART("checkpin\n"); 
						
							status_machine = SWAP_WAIT_ESP_PROCESS;
					}
					break;

			case SWAP_WAIT_REMOVAL:
					for(uint8_t i = 0; i < 3; i++)
					{
						if(i == fit_pin_remove){
							SetAngleServo(fit_pin_remove + 1, angle_open);
						}
						else{
							SetAngleServo(i + 1, angle_close);
						}
					}
					SetStateChannelBattery(fit_pin_remove + 1, STATE_IDLE);
					
					if (pin_channels[fit_pin_remove].data_pin.voltage < threshold_pin_empty) {
							pin_channels[fit_pin_remove].status = pin_empty;
							SetAngleServo(fit_pin_remove + 1, angle_close); 
						
							fit_pin_remove = 4;
							fit_pin_initial = 4;
							status_machine = IDLE_WAIT_CMD;
					}
					break;
					
			case INSTALL_WAIT_ESP_PROCESS:					
				if (esp32_pin_check_status == 1) {
						CheckPinNewInitial(id_pin_dang_kiem_tra, &pin_channels[id_pin_dang_kiem_tra]);
						
						if (pin_channels[id_pin_dang_kiem_tra].status == pin_charge || pin_channels[id_pin_dang_kiem_tra].status == pin_full) {
								CloseAllServo(angle_close);
								status_machine = IDLE_WAIT_CMD;
						} else {
								status_machine = INSTALL_ERROR_REMOVAL;
						}
						esp32_pin_check_status = -1;
				}
				else if (esp32_pin_check_status == 0) {
						status_machine = INSTALL_ERROR_REMOVAL;
						esp32_pin_check_status = -1; 
				}
				else if (HAL_GetTick() - timestamp_gui_checkpin > ESP32_CHECK_TIMEOUT) {
						status_machine = INSTALL_ERROR_REMOVAL;
						esp32_pin_check_status = -1;
				}
				break;

			case INSTALL_ERROR_REMOVAL:
				SetAngleServo(id_pin_dang_kiem_tra + 1, angle_open);
				
				if (pin_channels[id_pin_dang_kiem_tra].data_pin.voltage < threshold_pin_empty) {
						pin_channels[id_pin_dang_kiem_tra].status = pin_empty;
						id_pin_dang_kiem_tra = 4;
						status_machine = INSTALL_BATTERY; 
				}
				break;

			case SWAP_WAIT_ESP_PROCESS:
				if (esp32_pin_check_status == 1) {
						CheckPinNewInitial(fit_pin_initial, &pin_channels[fit_pin_initial]);

						if (pin_channels[fit_pin_initial].status == pin_charge || pin_channels[fit_pin_initial].status == pin_full) {
								SetAngleServo(fit_pin_initial + 1, angle_close);
								status_machine = SWAP_WAIT_REMOVAL;
						} else {
								status_machine = SWAP_WAIT_REMOVAL;
						}
						esp32_pin_check_status = -1;
				}
				else if (esp32_pin_check_status == 0) {
						status_machine = SWAP_WAIT_REMOVAL;
						esp32_pin_check_status = -1;
				}
				else if (HAL_GetTick() - timestamp_gui_checkpin > ESP32_CHECK_TIMEOUT) {
						status_machine = SWAP_WAIT_REMOVAL;
						esp32_pin_check_status = -1;
				}
				break;

			case SWAP_ERROR_REMOVAL:
				SetAngleServo(fit_pin_initial + 1, angle_open);
				
				if (pin_channels[fit_pin_initial].data_pin.voltage < threshold_pin_empty) {
						pin_channels[fit_pin_initial].status = pin_empty;
						status_machine = SWAP_WAIT_INSERTION;
				}
				break;	
		}
			
		for (uint8_t i = 0; i < 3; i++) {
			if (pin_channels[i].status == pin_charge && HAL_GetTick() - pin_channels[i].last_check_percent >= 300000) {
					SetStateChannelBattery(i + 1, STATE_IDLE); // Ngắt sạc
					HAL_Delay(100);
					pin_channels[i].data_pin = ReadManagerINA219(i + 1);
					pin_channels[i].last_check_percent = HAL_GetTick();
					pin_channels[i].check_percent_pin = 1;
					SetStateChannelBattery(i + 1, STATE_CHARGING); // Bật sạc lại
			}
		}
//		float temperatures[4];
//		INA219_Data_t data_pin[4];
//		  
//		SetStateChannelBattery(4, STATE_CHARGING);
//    if (HAL_GetTick() - last_sensor_update_tick >= 1000) 
//		{  
//    last_sensor_update_tick = HAL_GetTick();
//    for(uint8_t i = 0; i < 4; i++) {
//        temperatures[i] = NTC_GetTemperature(&hadc1, ADC_Channel[i]);
//        data_pin[i] = INA219_Manager_Read(i + 1);
//        
//        char data_transmit[100];
//        sprintf(data_transmit, "DATA%d: V=%.2f, C=%.2fmA, T=%.1f\r\n", 
//                i+1, data_pin[i].voltage, data_pin[i].current, temperatures[i]);
//        TransmitStringHandlerUART(data_transmit);
//    }
//		}  /* USER CODE END 3 */
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB12 PB13 PB14
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
