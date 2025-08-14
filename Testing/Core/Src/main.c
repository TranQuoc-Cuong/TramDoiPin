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

INA219_Data_t data_pin[4];

float temperatures[4]; 
const uint32_t ADC_Channel[] = {
	ADC_CHANNEL_0,
	ADC_CHANNEL_1,
	ADC_CHANNEL_2,
	ADC_CHANNEL_3
};

uint32_t last_sensor_update_tick = 0;
uint8_t charge_done_counter[4] = {0}; // Mảng đếm số lần dòng thấp cho mỗi kênh
uint32_t charge_done_timestamp[4] = {0}; // Mảng lưu mốc thời gian cho mỗi kênh

uint32_t last_check_percent[4] = {0};

typedef enum{
	pin_trong,
	pin_sac,
	pin_day,
	pin_qua_nhiet,
	pin_loi,
} status_channel_t;

status_channel_t pin_channels[4];
status_channel_t previous_pin_channels[4];

uint8_t check_percent_pin[4] = {0};

const char* get_channel_state_string(status_channel_t state) {
    switch(state) {
        case pin_trong:     return "T";
        case pin_sac:       return "S";
        case pin_day:       return "D";
        case pin_qua_nhiet: return "N";
        case pin_loi:       return "L";
        default:            return "R";
    }
}

void report_status_task(void) {
    char report_buffer[128];
		static uint8_t percent[4];
    for (int i = 0; i < 4; i++) 
	{
		if(pin_channels[i] == pin_trong){
			percent[i] = 0;
			check_percent_pin[i] = 0;
		}
		if(pin_channels[i] == pin_day){
			percent[i] = 100;
			check_percent_pin[i] = 0;
		}
		
		if(check_percent_pin[i] == 1){
			percent[i] = data_pin[i].soc_percent;
			check_percent_pin[i] = 0;
		}
    sprintf(report_buffer, "P:%d,%%:%d,C:%s,V:%.2f,I:%.2f,T:%.1f\n",
                i + 1,
								percent[i],
                get_channel_state_string(pin_channels[i]),
                data_pin[i].voltage,
                data_pin[i].current,
                temperatures[i]);
        UART_Handler_TransmitString(report_buffer);
    }
		UART_Handler_TransmitString("\n");
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
	
	char rx_line_buffer[100]; // Buffer g?c d? nh?n d? li?u
	char cmd_buffer[100];     // Buffer sao ch�p d? x? l� l?nh
	
	for(uint8_t i = 0; i < 4; i++){
		pin_channels[i] = pin_trong;
		previous_pin_channels[i] = pin_trong; // Đồng bộ hóa trạng thái cũ
		Servo_SetAngle(i + 1, 180); // Quay servo ra vị trí chờ pin
  };
	
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
			temperatures[i] = NTC_GetTemperature(&hadc1, ADC_Channel[i]);
			data_pin[i] = INA219_Manager_Read(i + 1);
    }
		
		for(uint8_t i = 0; i < 4; i++){
			switch(pin_channels[i]){
				
				case pin_trong:
					if(data_pin[i].voltage > NGUONGPINTRONG){ 
						BatteryHealth_t health_status = Diagnose_Check(i + 1, temperatures[i]);
						if(health_status == BATT_UNSAFE_TEMP || health_status == BATT_DEAD){
							pin_channels[i] = pin_loi;
						} 
						else {
							health_status = Diagnose_RunIRTest(i + 1);
							if (health_status == BATT_HEALTHY) {
								if (data_pin[i].soc_percent >= 98.0f) {
									pin_channels[i] = pin_day;
								} else {
									pin_channels[i] = pin_sac;
									check_percent_pin[i] = 1;
									last_check_percent[i] = HAL_GetTick();
								}
							} else {
									pin_channels[i] = pin_loi;
							}
						}
					}
				break;

				case pin_sac:
					if(temperatures[i] > NGUONGNHIETDO){
						pin_channels[i] = pin_qua_nhiet;
					} 
					if ((data_pin[i].current < 1.6 && data_pin[i].current > -2.3) || (data_pin[i].current < 0.1 && data_pin[i].current > -1.0))
					{
						if (charge_done_counter[i] == 0) {
							charge_done_timestamp[i] = HAL_GetTick();
						}
							charge_done_counter[i]++;
						if(charge_done_counter[i] >= 3 && (data_pin[i].current < 0.1 && data_pin[i].current > -1.0)){
							pin_channels[i] = pin_trong;
							charge_done_counter[i] = 0;
						}
						else if(charge_done_counter[i] >= 3 && (data_pin[i].current < 1.6 && data_pin[i].current > -2.3)){
							pin_channels[i] = pin_day;
							charge_done_counter[i] = 0;
						}
					}
					else {
						charge_done_counter[i] = 0;
					}

					if (charge_done_counter[i] > 0 && charge_done_counter[i] < 3) {
						if (HAL_GetTick() - charge_done_timestamp[i] > 6000) {
								charge_done_counter[i] = 0;
						}
					}
				break;

				case pin_qua_nhiet:
					if(temperatures[i] < NGUONGNHIETDO - 5){ // Thêm một khoảng an toàn
						pin_channels[i] = pin_sac;
					}
				break;

				case pin_loi:
				case pin_day:
					if(data_pin[i].voltage < NGUONGPINTRONG){ // Ngưỡng rút pin
						pin_channels[i] = pin_trong;
					}
				break;
			}
		}
		report_status_task();
	}
		
	for (uint8_t i = 0; i < 4; i++) {
    if (pin_channels[i] == pin_sac && HAL_GetTick() - last_check_percent[i] >= 300000) {
        Channel_SetState(i + 1, STATE_IDLE); // Ngắt sạc
        HAL_Delay(100);
        data_pin[i] = INA219_Manager_Read(i + 1);
        last_check_percent[i] = HAL_GetTick();
				check_percent_pin[i] = 1;
				Channel_SetState(i + 1, STATE_CHARGING); // Bật sạc lại
    }
	}
		
		for(uint8_t i = 0; i < 4; i++){
			if(pin_channels[i] != previous_pin_channels[i]){
				switch(pin_channels[i]){
					
					case pin_trong:
						Channel_SetState(i + 1, STATE_IDLE);
            Servo_SetAngle(i + 1, 180); // Mở cửa chờ
          break;
					
					case pin_sac:
						Channel_SetState(i + 1, STATE_CHARGING);
            Servo_SetAngle(i + 1, 10);   // Đóng cửa sạc
          break;
          
					case pin_day:
            Channel_SetState(i + 1, STATE_IDLE);
            Servo_SetAngle(i + 1, 180); // Đẩy pin ra
          break;
                   
					case pin_loi:
            Channel_SetState(i + 1, STATE_IDLE);
            Servo_SetAngle(i + 1, 180); // Đẩy pin ra
          break;
                    
					case pin_qua_nhiet:
            Channel_SetState(i + 1, STATE_IDLE); // Tạm ngắt sạc
            break;
        }
        previous_pin_channels[i] = pin_channels[i]; // Cập nhật trạng thái cũ
			}
		}
		
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
