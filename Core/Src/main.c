/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include "nrf24_Button.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define Debug

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t TxAddr[] = {0x00,0x34,0x36,0x52,0x53};
uint8_t RxAddr[] = {0x00,0x34,0x36,0x52,0x53};

uint16_t Press_Counter=0;
char counter_str[5];

uint8_t TxData[16];
uint8_t Button_ID = 0;
uint8_t RF_Channel_FRQ = 0;
uint16_t Batt_Vol = 0;
#define MIN_BATT_Vol 1000
uint8_t Need_To_Be_Charged = 0;
uint8_t Pressed_Key = 0;
uint8_t Sleep_Time = 0;
uint8_t Packet_Size = 16;
uint32_t adcValue;
float vddVoltage;
#define VREFINT_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF80078))
#define VREFINT_VOLTAGE 3
uint16_t vrefintCalibrationValue;

uint8_t Read_Button_ID(void)
{
	uint8_t Addr = 0;
	Addr |= (HAL_GPIO_ReadPin(GPIOB, Dev_ID0_Pin) << 0);
	Addr |= (HAL_GPIO_ReadPin(GPIOB, Dev_ID1_Pin) << 1);
	Addr |= (HAL_GPIO_ReadPin(GPIOC, Dev_ID2_Pin) << 2);
	Addr |= (HAL_GPIO_ReadPin(GPIOC, Dev_ID3_Pin) << 3);
	Addr |= (HAL_GPIO_ReadPin(GPIOC, Dev_ID4_Pin) << 4);
	Addr |= (HAL_GPIO_ReadPin(GPIOH, Dev_ID5_Pin) << 5);
	Addr |= (HAL_GPIO_ReadPin(GPIOH, Dev_ID6_Pin) << 6);
	Addr |= (HAL_GPIO_ReadPin(Dev_ID7_GPIO_Port, Dev_ID7_Pin) << 7);

	return Addr;
}

// Dev_ID = 1  to 6   ---> Channel 100
// Dev_ID = 7  to 12  ---> Channel 101
// Dev_ID = 13 to 18  ---> Channel 102
// Dev_ID = 19 to 24  ---> Channel 103
// Dev_ID = 25 to 30  ---> Channel 104
// Dev_ID = 31 to 36  ---> Channel 105
uint8_t RF_Channel_Lookup(uint8_t ID)
{
	uint8_t RF_CH = 100 + ((ID-1) / 6);
	return RF_CH;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */

  HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
  HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);

  /*** Wake up from stop mode in External Interrupt ***/
  SystemClock_Config();
  HAL_ResumeTick();
#ifndef Debug
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"WAKEUP from EXTI\n\n",18, HAL_MAX_DELAY);
#endif


  for (volatile uint32_t i = 0; i < 100000; i++);

  Press_Counter++;
  if(Press_Counter>65534) Press_Counter=0;

  if (GPIO_Pin == Key_IN1_Pin)
  {
#ifndef Debug
	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Key1 Was pressed\n\n",18, HAL_MAX_DELAY);
#endif
	  Pressed_Key = 1;
  }
  if (GPIO_Pin == Key_IN2_Pin)
  {
#ifndef Debug
	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Key2 Was pressed\n\n",18, HAL_MAX_DELAY);
#endif
	  Pressed_Key = 2;
  }
  if (GPIO_Pin == Key_IN3_Pin)
  {
#ifndef Debug
	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Key3 Was pressed\n\n",18, HAL_MAX_DELAY);
#endif
	  Pressed_Key = 3;
  }
  if (GPIO_Pin == Key_IN4_Pin)
  {
#ifndef Debug
	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Key4 Was pressed\n\n",18, HAL_MAX_DELAY);
#endif
	  Pressed_Key = 4;
  }
  //__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
  HAL_NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
  HAL_NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
  HAL_NVIC_ClearPendingIRQ(EXTI4_15_IRQn);

  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
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
  MX_ADC_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(10000);

  Button_ID = Read_Button_ID();                                       //Reading the Button ID through DIP Switch
  TxData[0] = Button_ID;
  RF_Channel_FRQ = RF_Channel_Lookup(Button_ID);                      //Define the Frequency Channel based on the Button_ID
  TxAddr[0] = Button_ID;                                              //Set the RF Module TxAddr based on Button_ID
  RxAddr[0] = Button_ID;                                              //Set the RF Module RxAddr based on Button_ID

  nRF24_Reset(nRF_CSN_Pin, nRF24_REG_STATUS);

  nRF24_Module_Setup(nRF_CSN_Pin, Transmitter_Mode_noAA_noCRC,
		             RF_Channel_FRQ, TxAddr, RxAddr, MBPS1_0dBm,
					 Packet_Size);

  nRF24_Register_Display(nRF_CSN_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	  //HAL_GPIO_TogglePin(GPIOB, LED_BLUE_Pin);
	  //HAL_GPIO_TogglePin(GPIOB, LED_RED_Pin);
	  //HAL_Delay(500);

	  // Start ADC conversion

	  HAL_ADC_Start(&hadc);

	  if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK)
	  {
		  adcValue = HAL_ADC_GetValue(&hadc);
		  vrefintCalibrationValue = *VREFINT_CAL_ADDR;
		  // Calculate VDD voltage
		  vddVoltage = ((float)vrefintCalibrationValue * VREFINT_VOLTAGE) / (float)adcValue;

		  if(vddVoltage < 3)
		  {
			  Need_To_Be_Charged = 1;
		  }
		  else
		  {
			  Need_To_Be_Charged = 0;
		  }
	  }
	  else
	  {
		  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"ADC ERROR\n", 10, 100);
	  }

	  HAL_Delay(1);
	  //key1
	  if (Pressed_Key == 1)
	  {
		  Pressed_Key = 0;

		  //Check for the Battery Voltage to select the LED Color 2n4 a   1n3 b
		  while(HAL_GPIO_ReadPin(GPIOB, Key_IN1_Pin) == GPIO_PIN_SET)
		  {
			  if(Need_To_Be_Charged == 1)
			  {
				  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_RESET);
			  }
			  else
			  {
			  	  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	    	  }
		  }
		  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

		  TxData[1] = 1;
		  nRF24_Transmit(nRF_CSN_Pin, TxData, Packet_Size);
#ifndef Debug
		  nRF24_Transmit_Report(nRF_CSN_Pin);
#endif

	  }
	  //key2
	  if (Pressed_Key == 2)
	  {
		  Pressed_Key = 0;

		  //Check the Battery Voltage to select the LED Color 2n4 a   1n3 b
		  while(HAL_GPIO_ReadPin(GPIOA, Key_IN2_Pin) == GPIO_PIN_SET)
		  {
			  if(Need_To_Be_Charged == 1)
			  {
				  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_RESET);
			  }
			  else
			  {
			  	  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	    	  }
		  }
		  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

		  TxData[1] = 2;
		  nRF24_Transmit(nRF_CSN_Pin, TxData, Packet_Size);
#ifndef Debug
		  nRF24_Transmit_Report(nRF_CSN_Pin);
#endif

	  }
	  //key3
	  if (Pressed_Key == 3)
	  {
		  Pressed_Key = 0;

		  //Check for the Battery Voltage to select the LED Color 2n4 a   1n3 b
		  while(HAL_GPIO_ReadPin(GPIOB, Key_IN3_Pin) == GPIO_PIN_SET)
		  {
			  if(Need_To_Be_Charged == 1)
			  {
				  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_RESET);
			  }
			  else
			  {
			  	  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	    	  }
		  }
		  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

		  TxData[1] = 3;
		  nRF24_Transmit(nRF_CSN_Pin, TxData, Packet_Size);
#ifndef Debug
		  nRF24_Transmit_Report(nRF_CSN_Pin);
#endif

	  }

	  //key4
	  if (Pressed_Key == 4)
	  {
		  Pressed_Key = 0;

		  //Check for the Battery Voltage to select the LED Color 2n4 a   1n3 b
		  while(HAL_GPIO_ReadPin(GPIOA, Key_IN4_Pin) == GPIO_PIN_SET)
		  {
			  if(Need_To_Be_Charged == 1)
			  {
				  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_RESET);
			  }
			  else
			  {
			  	  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	    	  }
		  }
		  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

		  TxData[1] = 4;
		  nRF24_Transmit(nRF_CSN_Pin, TxData, Packet_Size);
#ifndef Debug
		  nRF24_Transmit_Report(nRF_CSN_Pin);
#endif

	  }

	  HAL_Delay(1);
#ifndef Debug
	  char *message = "About to Go into the STOP mode\n\n";
	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)message, strlen (message), 100);    //Last Words
#endif
	  HAL_SuspendTick();                                                          //Suspend the sys tick before going into stop mode
	  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);         //Enter the stop mode


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, nRF_CSN_Pin|LED_GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nRF_CE_GPIO_Port, nRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Dev_ID2_Pin Dev_ID3_Pin Dev_ID4_Pin */
  GPIO_InitStruct.Pin = Dev_ID2_Pin|Dev_ID3_Pin|Dev_ID4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Dev_ID5_Pin Dev_ID6_Pin */
  GPIO_InitStruct.Pin = Dev_ID5_Pin|Dev_ID6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : Dev_ID7_Pin */
  GPIO_InitStruct.Pin = Dev_ID7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Dev_ID7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Key_IN4_Pin Key_IN2_Pin */
  GPIO_InitStruct.Pin = Key_IN4_Pin|Key_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : nRF_CSN_Pin */
  GPIO_InitStruct.Pin = nRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nRF_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nRF_IRQ_Pin */
  GPIO_InitStruct.Pin = nRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nRF_CE_Pin */
  GPIO_InitStruct.Pin = nRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nRF_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Key_IN1_Pin Key_IN3_Pin */
  GPIO_InitStruct.Pin = Key_IN1_Pin|Key_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Dev_ID0_Pin Dev_ID1_Pin */
  GPIO_InitStruct.Pin = Dev_ID0_Pin|Dev_ID1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void PWR_EnterStopMode(void)
{
	HAL_SuspendTick();
	HAL_PWR_EnableSleepOnExit();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

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
