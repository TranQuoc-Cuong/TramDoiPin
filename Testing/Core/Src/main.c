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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define NGUONGPINTRONG 500.0f
#define NGUONGNHIETDO 55.0f
#define GOCMO 180
#define GOCDONG 10

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
	ADC_CHANNEL_3
};

uint32_t last_sensor_update_tick = 0;

typedef enum{
	pin_trong,
	pin_sac,
	pin_day,
	pin_qua_nhiet,
	pin_loi
} status_channel_t;

typedef enum {
	LAP_PIN,
	KHONG_CO_PIN,
	CHO_LENH,
	LAY_PIN_CHO_THAO,
	DOI_PIN_CHO_LAP_VAO,
	DOI_PIN_DANG_KIEM_TRA,
	DOI_PIN_CHO_THAO_RA
}status_machine_t;

status_machine_t status_machine = CHO_LENH;

typedef	struct{
	INA219_Data_t data_pin;
	float temperatures; 
	
	status_channel_t status;
	
	uint8_t charge_done_counter;
	uint32_t charge_done_timestamp;
	
	uint8_t check_percent_pin;
	uint32_t last_check_percent;
}pin_channel_t;

pin_channel_t pin_channels[4] = {0};

uint8_t id_pin_percent_max = 0;
uint8_t id_pin_trong = 0;

const char* get_channel_state_string(status_channel_t state) {
    switch(state) {
        case pin_trong:     return "T";
        case pin_sac:  		  return "S";
        case pin_day: 		  return "D";
        case pin_qua_nhiet: return "N";
        case pin_loi:       return "L";
        default:            return "R";
    }
}

static uint8_t percent[4];

void report_status_task(void) {
    char report_buffer[50];
    for (int i = 0; i < 4; i++) 
	{
		if(pin_channels[i].status == pin_trong){
			percent[i] = 0;
			pin_channels[i].check_percent_pin = 0;
		}
		if(pin_channels[i].status == pin_day){
			percent[i] = 100;
			pin_channels[i].check_percent_pin = 0;
		}
		
		if(pin_channels[i].check_percent_pin == 1){
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
        UART_Handler_TransmitString(report_buffer);
    }
		UART_Handler_TransmitString("\n");
}

uint8_t fit_pin_thao = 5;
uint8_t fit_pin_lap = 5;

void CMD_Process(char* command_buffer) {
		if (strncmp(command_buffer, "laypin:", 7) == 0) {
			if (id_pin_percent_max < 4) { 
				fit_pin_thao = id_pin_percent_max;
				status_machine = LAY_PIN_CHO_THAO;
				char tin_gui[14];
				sprintf(tin_gui, "laypin:%d.\n", fit_pin_thao + 1);
				UART_Handler_TransmitString(tin_gui);
			} 
			else {
				UART_Handler_TransmitString("Khong co pin san sang de lay!\r\n");
			}
		}
		else if (strncmp(command_buffer, "back:", 5) == 0){
			status_machine = CHO_LENH;
		}
		else if (strncmp(command_buffer, "doipin:", 7) == 0) {
			if (id_pin_trong < 4 && id_pin_percent_max < 4) {
					fit_pin_thao = id_pin_percent_max;
					fit_pin_lap = id_pin_trong;
					status_machine = DOI_PIN_CHO_LAP_VAO;
					char tin_gui[21];
					sprintf(tin_gui, "doipin:%d,laypin:%d\n", fit_pin_lap + 1, fit_pin_thao + 1);
					UART_Handler_TransmitString(tin_gui);
			} else {
					UART_Handler_TransmitString("Khong the doi pin luc nay!\r\n");
			}
		}
		else if (strncmp(command_buffer, "lappin:", 7) == 0){
			uint8_t so_pin_trong = 0;
			for(uint8_t i = 0; i < 4; i++){
				if(pin_channels[i].status == pin_trong) ++so_pin_trong;				
			}
			if(so_pin_trong == 0) UART_Handler_TransmitString("lappin:0\n");
			else {UART_Handler_TransmitString("lappin:1\n"); status_machine = LAP_PIN;}
		}
    else if (strncmp(command_buffer, "sac:", 4) == 0) {
        CMD_ParseSimple(command_buffer + 4, STATE_CHARGING);
    } 
    else if (strncmp(command_buffer, "test:", 5) == 0) {
        CMD_ParseSimple(command_buffer + 5, STATE_TESTING);
    } 
    else if (strncmp(command_buffer, "pinoff:", 7) == 0) {
        CMD_ParseSimple(command_buffer + 7, STATE_IDLE);
    } 
    else if (strncmp(command_buffer, "servo:", 6) == 0) {
        CMD_ParseServo(command_buffer + 6);
    }
}

//cac ham kt pin

	void kiemTraPinMoiLapVao(uint8_t id, pin_channel_t *id_channel) {
			if (id_channel->data_pin.voltage > NGUONGPINTRONG) {
					BatteryHealth_t health_status = Diagnose_Check(id + 1, id_channel->temperatures);
					
					if (health_status == BATT_UNSAFE_TEMP || health_status == BATT_DEAD) {
							id_channel->status = pin_loi;
					} else {
							health_status = Diagnose_RunIRTest(id + 1);
							if (health_status == BATT_HEALTHY) {
									if (id_channel->data_pin.soc_percent >= 98.0f) {
											id_channel->status = pin_day;
											if(status_machine == KHONG_CO_PIN) status_machine = CHO_LENH;
									} else {
											id_channel->status = pin_sac;
											if(status_machine == KHONG_CO_PIN) status_machine = CHO_LENH;
											id_channel->check_percent_pin = 1;
											id_channel->last_check_percent = HAL_GetTick();
									}
							} else {
									id_channel->status = pin_loi;
							}
					}
			}
	}


	void xuLyTrangThaiSac(pin_channel_t *channel) {
			if (channel->temperatures > NGUONGNHIETDO || channel->temperatures < 5.0f) { 
					channel->status = pin_qua_nhiet; // Coi mọi giá trị bất thường là quá nhiệt (hoặc lỗi)
					return; 
			}
			
			if ((channel->data_pin.current < 1.6 && channel->data_pin.current > -2.3) || (channel->data_pin.current < 0.1 && channel->data_pin.current > -1.0)) {
					if (channel->charge_done_counter == 0) {
							channel->charge_done_timestamp = HAL_GetTick();
					}
					channel->charge_done_counter++;

					if (channel->charge_done_counter >= 3) {
							if (channel->data_pin.current < 0.1 && channel->data_pin.current > -1.0) {
									channel->status = pin_trong;
							} else { // (channel->data_pin.current < 1.6 && channel->data_pin.current > -2.3)
									channel->status = pin_day;
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
	UART_Handler_Init(&huart1);
	Channels_SetAll_Idle(); // trang thai mac dinh cua sac
	Servos_Init(&htim2); // Them tat ca kenh Servo
	INA219_Manager_Init(&hi2c1); // khoi  tao cam bien dong ap
	
	char rx_line_buffer[100];
	char cmd_buffer[100];     
	
	for(uint8_t i = 0; i < 4; i++){
		pin_channels[i].status = pin_trong;
  };
	
	All_Servo_Angle(GOCMO);
	
	last_sensor_update_tick = HAL_GetTick();
	
	UART_Handler_TransmitString("He thong khoi dong");
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		
    if (UART_Handler_GetLine(rx_line_buffer, 100))
    {
        strcpy(cmd_buffer, rx_line_buffer);
        CMD_Process(cmd_buffer);
				UART_Handler_TransmitString("ok\r\n");
    }
		
		if (HAL_GetTick() - last_sensor_update_tick >= 2000) {
			
			last_sensor_update_tick = HAL_GetTick();

			for(uint8_t i = 0; i < 4; i++){
					HAL_Delay(100);
					pin_channels[i].temperatures = NTC_GetTemperature(&hadc1, ADC_Channel[i]);
					pin_channels[i].data_pin = INA219_Manager_Read(i + 1);
					HAL_Delay(200);
			}

			uint8_t temp_max_percent = 0;
			uint8_t found_empty_slot = 0;
			id_pin_percent_max = 5; 
			id_pin_trong = 5;       
			if(status_machine == CHO_LENH) status_machine = KHONG_CO_PIN;
			for (int i = 0; i < 4; i++) {
					if(pin_channels[i].status == pin_trong) {
							percent[i] = 0;
					} else if(pin_channels[i].status == pin_day) {
							if(status_machine == KHONG_CO_PIN) status_machine = CHO_LENH;
							percent[i] = 100;
					} else if(pin_channels[i].check_percent_pin == 1) { 
							percent[i] = pin_channels[i].data_pin.soc_percent;
							pin_channels[i].check_percent_pin = 0;
					}

					if ((pin_channels[i].status == pin_sac || pin_channels[i].status == pin_day) && percent[i] >= temp_max_percent) {
							if(status_machine == KHONG_CO_PIN) status_machine = CHO_LENH;
							temp_max_percent = percent[i];
							id_pin_percent_max = i;
					}

					if ((pin_channels[i].status == pin_trong) && !found_empty_slot) {
							id_pin_trong = i;
							found_empty_slot = 1;
					}
			}			
			report_status_task();
		}

		switch (status_machine)
		{				
			case LAP_PIN:
				for(uint8_t i = 0; i < 4; i++){
					if(pin_channels[i].status == pin_trong || pin_channels[i].status == pin_loi){
						Servo_SetAngle(i + 1, GOCMO);
						
						if(pin_channels[i].status == pin_loi){
							if(pin_channels[i].data_pin.voltage < NGUONGPINTRONG)
								pin_channels[i].status = pin_trong;
						}
						
						if(pin_channels[i].data_pin.voltage > NGUONGPINTRONG){
							kiemTraPinMoiLapVao(i, &(pin_channels[i]));
							
							if (pin_channels[i].status == pin_sac || pin_channels[i].status == pin_day) {
								All_Servo_Angle(GOCDONG); 
								status_machine = CHO_LENH; 
								break; 
							}
							else if (pin_channels[i].status == pin_loi) {
								Servo_SetAngle(i + 1, GOCMO);
							}
						}
					}
				}
				break;
				
			case KHONG_CO_PIN:
				All_Servo_Angle(GOCMO); 
			
				for(uint8_t i = 0; i < 4; i++){
					Channel_SetState(i + 1, STATE_IDLE);
					kiemTraPinMoiLapVao(i, &(pin_channels[i]));
				}
				break;
				
			case CHO_LENH:
				All_Servo_Angle(GOCDONG); 
			
				for (uint8_t i = 0; i < 4; i++) {
					switch(pin_channels[i].status) {
						case pin_trong:
							Channel_SetState(i + 1, STATE_IDLE);
							kiemTraPinMoiLapVao(i, &(pin_channels[i]));
							break;
			
						case pin_sac:
							xuLyTrangThaiSac(&(pin_channels[i]));
							Channel_SetState(i + 1, STATE_CHARGING);
							break;
						
						case pin_qua_nhiet:
							if(pin_channels[i].temperatures < NGUONGNHIETDO - 5) {
								pin_channels[i].status = pin_sac;
							}
							Channel_SetState(i + 1, STATE_IDLE);
							break;
							
						case pin_day:
						case pin_loi:
							if(pin_channels[i].data_pin.voltage < NGUONGPINTRONG) {
								pin_channels[i].status = pin_trong;
							}
							Channel_SetState(i + 1, STATE_IDLE);
							break;
			
						default:
						
						break;
					}
				}
			break;
				
			case LAY_PIN_CHO_THAO:
					Servo_SetAngle(fit_pin_thao + 1, GOCMO);
					Channel_SetState(fit_pin_thao + 1, STATE_IDLE);
			
					if (pin_channels[fit_pin_thao].data_pin.voltage < NGUONGPINTRONG) {
							pin_channels[fit_pin_thao].status = pin_trong;
							Servo_SetAngle(fit_pin_thao + 1, GOCDONG); 
							fit_pin_thao = 5; 
							status_machine = CHO_LENH; 
					}
					break;

			case DOI_PIN_CHO_LAP_VAO:
					for(uint8_t i = 0; i < 4; i++)
					{
						if(i == fit_pin_lap){
							Servo_SetAngle(fit_pin_lap + 1, GOCMO);
						}
						else{
							Servo_SetAngle(i + 1, GOCDONG);
						}
					}
					
					if (pin_channels[fit_pin_lap].data_pin.voltage > NGUONGPINTRONG) {
							status_machine = DOI_PIN_DANG_KIEM_TRA; 
					}
					break;
			
			case DOI_PIN_DANG_KIEM_TRA:
					kiemTraPinMoiLapVao(fit_pin_lap, &pin_channels[fit_pin_lap]);
					if (pin_channels[fit_pin_lap].status == pin_sac || pin_channels[fit_pin_lap].status == pin_day) {
							Servo_SetAngle(fit_pin_lap + 1, GOCDONG);
							status_machine = DOI_PIN_CHO_THAO_RA; 
					}
					
					if(pin_channels[fit_pin_lap].status == pin_loi){
							if(pin_channels[fit_pin_lap].data_pin.voltage < NGUONGPINTRONG)
								pin_channels[fit_pin_lap].status = pin_trong;
					}

					else if (pin_channels[fit_pin_lap].status == pin_loi) {
							status_machine = DOI_PIN_CHO_LAP_VAO; 
					}
					break;

			case DOI_PIN_CHO_THAO_RA:
					for(uint8_t i = 0; i < 4; i++)
					{
						if(i == fit_pin_thao){
							Servo_SetAngle(fit_pin_thao + 1, GOCMO);
						}
						else{
							Servo_SetAngle(i + 1, GOCDONG);
						}
					}
					Channel_SetState(fit_pin_thao + 1, STATE_IDLE);
					
					if (pin_channels[fit_pin_thao].data_pin.voltage < NGUONGPINTRONG) {
							pin_channels[fit_pin_thao].status = pin_trong;
							Servo_SetAngle(fit_pin_thao + 1, GOCDONG); 
						
							fit_pin_thao = 5;
							fit_pin_lap = 5;
							status_machine = CHO_LENH;
					}
					break;
		}
			
		for (uint8_t i = 0; i < 4; i++) {
			if (pin_channels[i].status == pin_sac && HAL_GetTick() - pin_channels[i].last_check_percent >= 300000) {
					Channel_SetState(i + 1, STATE_IDLE); // Ngắt sạc
					HAL_Delay(100);
					pin_channels[i].data_pin = INA219_Manager_Read(i + 1);
					pin_channels[i].last_check_percent = HAL_GetTick();
					pin_channels[i].check_percent_pin = 1;
					Channel_SetState(i + 1, STATE_CHARGING); // Bật sạc lại
			}
		}

		
//    if (HAL_GetTick() - last_sensor_update_tick >= 1000) 
//		{  
//    last_sensor_update_tick = HAL_GetTick();
//    for(uint8_t i = 0; i < 4; i++) {
//      //  temperatures[i] = NTC_GetTemperature(&hadc1, ADC_Channel[i]);
//      //  data_pin[i] = INA219_Manager_Read(i + 1);
//        
//        char data_transmit[100];
//   //     sprintf(data_transmit, "DATA%d: V=%.2f, C=%.2fmA, T=%.1f\r\n", 
//  //              i+1, data_pin[i].voltage, data_pin[i].current, temperatures[i]);
//        UART_Handler_TransmitString(data_transmit);
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
