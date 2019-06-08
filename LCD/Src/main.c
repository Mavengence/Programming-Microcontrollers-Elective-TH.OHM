/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint8_t aRxBuffer[20];
uint8_t aTxBuffer[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char displayBuffer[9];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//uint8_t buffer[20];
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Transmit_IT(&huart3, (uint8_t *) aRxBuffer,20);
	HAL_UART_Transmit_IT(&huart2, (uint8_t *) aRxBuffer,20);
	HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer,1);
	HAL_UART_Receive_IT(&huart2, (uint8_t *) aRxBuffer,1);
	 
	
	lcd_gpio_setup();
	lcd_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		//uint8_t bla[] = "hallo welt\r\n";
		//HAL_UART_Receive_IT(
    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = RTC_MONTH_MAY;
  sDate.Date = 0x17;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A 
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x15;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
  sAlarm.AlarmDateWeekDay = RTC_WEEKDAY_FRIDAY;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the TimeStamp 
  */
  if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
	
	
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9600;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4095;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 4999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
	
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Green_LED_Pin|LD3_Pin|Blue_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RGB_On_GPIO_Port, RGB_On_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RGB_LED2_Pin|RGB_LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Green_LED_Pin LD3_Pin Blue_LED_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin|LD3_Pin|Blue_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RGB_On_Pin */
  GPIO_InitStruct.Pin = RGB_On_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RGB_On_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB_LED2_Pin RGB_LED3_Pin */
  GPIO_InitStruct.Pin = RGB_LED2_Pin|RGB_LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG9 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t reciveBuffer[20];
int counter = 0;
int showTemp = 1;
int showAlarm;

void printTemp() {
				lcd_clear(); 
				lcd_setcursor(0,1);
				lcd_string("Temp:");
				lcd_setcursor(0, 2);
				reciveBuffer[5] = '\0';
				lcd_string((char*)&reciveBuffer[0]);
}

void printHum() {
				lcd_clear(); 
				lcd_setcursor(0,1);
				lcd_string("Hum:");
				lcd_setcursor(0, 2);
				reciveBuffer[7] = '\0';
				lcd_string((char*)&reciveBuffer[5]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
		if (huart->Instance == USART2){
			HAL_UART_Receive_IT(&huart2, aRxBuffer, 1);
			reciveBuffer[counter] = *aRxBuffer;			
			if(reciveBuffer[counter] == '\n')
			{
				HAL_UART_Transmit(&huart3, reciveBuffer, counter, HAL_MAX_DELAY);
				uint8_t bla[] = "\r\n";
				HAL_UART_Transmit(&huart3, bla, 2, HAL_MAX_DELAY);
				
				if (reciveBuffer[7] == '1')
				{
					lcd_clear(); 
					lcd_setcursor(0,1);
					lcd_string("ALARM!");
				} else { 	
					if(showTemp)
					{
						showTemp = 0;
						printTemp();
						showAlarm = (int)reciveBuffer[7] - 49;
					} else {
						showTemp = 1;
						printHum();
					}
				}

				counter = 0;
			} else
				counter++;
		} else {
			HAL_UART_Receive_IT(&huart3, aTxBuffer,1);
			HAL_UART_Transmit(&huart2, aTxBuffer, 1, HAL_MAX_DELAY);
		}
}


static __IO uint32_t uwTimingDelay;
void lcd_gpio_setup (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
		
	  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  
  GPIO_InitStructure.Pin = LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PortData, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.Pin = LCD_RS | LCD_RW | LCD_E ;
  HAL_GPIO_Init(PortCnt, &GPIO_InitStructure);
}

////////////////////////////////////////////////////////////////////////////////
// Erzeugt einen Enable-Puls
static void lcd_enable( void )
{
    HAL_GPIO_WritePin(PortCnt, LCD_E,1);
    HAL_Delay( LCD_ENABLE_US );
    HAL_GPIO_WritePin(PortCnt, LCD_E,0);
	  HAL_Delay(LCD_ENABLE_US );		
}
 
////////////////////////////////////////////////////////////////////////////////
// Sendet eine 4-bit Ausgabeoperation an das LCD
static void lcd_out( int zahl )
{
int i, j, stelle, mod, snap;
int bit[8];
int nibblesLOW[4];
int nibblesHIGH[4];
    
    stelle = 0;
 	
 	for (i=0; i<=7 ; i++)
 		{
 		mod=zahl%2; 		
 		bit[stelle]=mod;
	
		if (stelle<=3) 
		   {
		   nibblesLOW[stelle]=bit[stelle];
		   }
        else 
			 {
			 snap = stelle - 4;
			 nibblesHIGH[snap]=bit[stelle];
			 }
 		
 		stelle = stelle + 1; 		
 		zahl = zahl/2; 		 		
		}
    
    if (nibblesHIGH[0]==1) HAL_GPIO_WritePin(PortData, LCD_D4,1);
    else HAL_GPIO_WritePin(PortData, LCD_D4,0);
        
    if (nibblesHIGH[1]==1) HAL_GPIO_WritePin(PortData, LCD_D5,1);
    else HAL_GPIO_WritePin(PortData, LCD_D5,0);
        
    if (nibblesHIGH[2]==1) HAL_GPIO_WritePin(PortData, LCD_D6,1);
    else HAL_GPIO_WritePin(PortData, LCD_D6,0);
        
    if (nibblesHIGH[3]==1) HAL_GPIO_WritePin(PortData, LCD_D7,1);
    else HAL_GPIO_WritePin(PortData, LCD_D7,0);
        
    lcd_enable();
    
    if (nibblesLOW[0]==1) HAL_GPIO_WritePin(PortData, LCD_D4,1);
    else HAL_GPIO_WritePin(PortData, LCD_D4,0);
        
    if (nibblesLOW[1]==1) HAL_GPIO_WritePin(PortData, LCD_D5,1);
    else HAL_GPIO_WritePin(PortData, LCD_D5,0);
        
    if (nibblesLOW[2]==1) HAL_GPIO_WritePin(PortData, LCD_D6,1);
    else HAL_GPIO_WritePin(PortData, LCD_D6,0);
        
    if (nibblesLOW[3]==1) HAL_GPIO_WritePin(PortData, LCD_D7,1);
    else HAL_GPIO_WritePin(PortData, LCD_D7,0);
        
    lcd_enable();    
}

 ////////////////////////////////////////////////////////////////////////////////
// Sendet ein Datenbyte an das LCD
void lcd_data ( int zahl )
{
    HAL_GPIO_WritePin(PortCnt, LCD_RS,1);         //(RS: 1=Data, 0=Command)    
    lcd_out( zahl );
    HAL_Delay( LCD_WRITEDATA_US );
}
 
////////////////////////////////////////////////////////////////////////////////
// Sendet einen Befehl an das LCD
void lcd_command( int zahl )
{
    HAL_GPIO_WritePin(PortCnt, LCD_RS,0);         //(RS: 1=Data, 0=Command)    
    lcd_out( zahl );
    HAL_Delay( LCD_COMMAND_US );
}
////////////////////////////////////////////////////////////////////////////////
// Initialisierung: muss ganz am Anfang des Programms aufgerufen werden.
void lcd_init( void )
{
   
	  
    //alles auf '0' setzen
    HAL_GPIO_WritePin(PortCnt, LCD_RS,0);		//Register Select
    HAL_GPIO_WritePin(PortCnt, LCD_RW,0);		//Read Write
    HAL_GPIO_WritePin(PortCnt, LCD_E,0);		//Enable
    HAL_GPIO_WritePin(PortData, LCD_D3,0);	//DB0-D3
	  HAL_GPIO_WritePin(PortData, LCD_D4,0);	//DB4
    HAL_GPIO_WritePin(PortData, LCD_D5,0);	//DB5
    HAL_GPIO_WritePin(PortData, LCD_D6,0);	//DB6
    HAL_GPIO_WritePin(PortData, LCD_D7,0);	//DB7

    // warten auf die Bereitschaft des LCD
    HAL_Delay( LCD_BOOTUP_MS );
   
    // Soft-Reset muss 3mal hintereinander gesendet werden zur Initialisierung

    //lcd_out( LCD_8BIT_RESET ); 
    HAL_GPIO_WritePin(PortData, LCD_D4,0);	//DB4
    HAL_GPIO_WritePin(PortData, LCD_D5,1);	//DB5
    HAL_GPIO_WritePin(PortData, LCD_D6,0);	//DB6
    HAL_GPIO_WritePin(PortData, LCD_D7,0);	//DB7
		lcd_enable();
    HAL_Delay( LCD_SOFT_RESET_MS);

    lcd_enable();
    HAL_Delay( LCD_SOFT_RESET_MS);

    lcd_enable();
    HAL_Delay( LCD_SOFT_RESET_MS );
     
	//lcd_out( LCD_4BIT_RESET ); 
//       HAL_GPIO_WritePin(PortData, LCD_D4,0);	//DB4
//       HAL_GPIO_WritePin(PortData, LCD_D5,1);	//DB5
//       HAL_GPIO_WritePin(PortData, LCD_D6,0);	//DB6
//       HAL_GPIO_WritePin(PortData, LCD_D7,0);	//DB7
//    	lcd_enable();  
//     HAL_Delay( LCD_SET_4BITMODE_MS );
		
	lcd_command( LCD_SET_FUNCTION +
                  LCD_FUNCTION_2LINE);

    HAL_Delay( LCD_SET_4BITMODE_MS );

	lcd_command( LCD_DISPLAY_ON +
                 LCD_CURSOR_ON //+
				 /*LCD_BLINKING_ON*/);

    HAL_Delay( LCD_SET_4BITMODE_MS );
}
////////////////////////////////////////////////////////////////////////////////
// Initialisierung: muss ganz am Anfang des Programms aufgerufen werden.
// void lcd_init( void )
// {

//     //alles auf '0' setzen
//     HAL_GPIO_WritePin(Port, LCD_E,0);
//     HAL_GPIO_WritePin(Port, LCD_RS,0);
//     HAL_GPIO_WritePin(Port, LCD_RW,0);
//     HAL_GPIO_WritePin(Port, LCD_D4,0);
//     HAL_GPIO_WritePin(Port, LCD_D5,0);
//     HAL_GPIO_WritePin(Port, LCD_D6,0);
//     HAL_GPIO_WritePin(Port, LCD_D7,0);
//   

//     // warten auf die Bereitschaft des LCD
//     HAL_Delay( LCD_BOOTUP_MS );
//     
//     // Soft-Reset muss 3mal hintereinander gesendet werden zur Initialisierung

//     lcd_out( LCD_4BIT_RESET ); 
//    
//     HAL_Delay( LCD_SOFT_RESET_MS);


//     lcd_enable();
//     HAL_Delay( LCD_SOFT_RESET_MS);


//     lcd_enable();
//     HAL_Delay( LCD_SOFT_RESET_MS );
//         
//      
//     // 8-bit Modus / 2 Zeilen / 5x7 
//     lcd_command( LCD_SET_FUNCTION +
//                  LCD_FUNCTION_8BIT +
//                  LCD_FUNCTION_2LINE +
//                  LCD_FUNCTION_5X7 );
//                  
//     HAL_Delay( LCD_SET_4BITMODE_MS );

//     // Display ein / Cursor ein / Blinken ein 
//     lcd_command( LCD_SET_DISPLAY +
//                  LCD_DISPLAY_ON +
//                  LCD_CURSOR_ON +
//                  LCD_BLINKING_ON);

//     HAL_Delay( LCD_SET_4BITMODE_MS );
//  
//     // Cursor inkrement / kein Scrollen =
//     lcd_command( LCD_SET_ENTRY +
//                  LCD_ENTRY_INCREASE +
//                  LCD_ENTRY_NOSHIFT );
//  
//     HAL_Delay( LCD_SET_4BITMODE_MS );
//     lcd_clear();
// }
//   
////////////////////////////////////////////////////////////////////////////////
// Setzt den Cursor in Zeile y (1..2) Spalte x (0..15)
void lcd_setcursor( uint8_t x, uint8_t y )// Setzt den Cursor in Zeile y (1..2) Spalte x (0..15)
{
    uint8_t data;
 
    switch (y)
    {
        case 1:    // 1. Zeile
            data = LCD_SET_DDADR + LCD_DDADR_LINE1 + x;
            break;
 
        case 2:    // 2. Zeile
            data = LCD_SET_DDADR + LCD_DDADR_LINE2 + x;
            break;
 
        default:
            return;                                   // für den Fall einer falschen Zeile
    }
 
    lcd_command( data );
}

////////////////////////////////////////////////////////////////////////////////
// Sendet den Befehl zur Löschung des Displays
void lcd_clear( void )
{
    lcd_command( LCD_CLEAR_DISPLAY );
    HAL_Delay( LCD_CLEAR_DISPLAY_MS );
}
 
////////////////////////////////////////////////////////////////////////////////
// Sendet den Befehl: Cursor Home
void lcd_home( void )
{
    lcd_command( LCD_CURSOR_HOME );
    HAL_Delay( LCD_CURSOR_HOME_MS );
}
 

 
////////////////////////////////////////////////////////////////////////////////
// Schreibt einen String auf das LCD
 
void lcd_string( const char *data )
{
    while( *data != '\0' )
        lcd_data( *data++ );
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
