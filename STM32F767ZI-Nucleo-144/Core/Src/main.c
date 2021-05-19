/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {

	// Colour Pulse Width Measurements
	int redPulseWidth;
	int greenPulseWidth;
	int bluePulseWidth;

	// Variables for final Colour RGB Values
	uint8_t redValue;
	uint8_t greenValue;
	uint8_t blueValue;

	uint8_t doCalibration;

	// Function Pointers
	int (*getRedPulseWidth)(void);
	int (*getGreenPulseWidth)(void);
	int (*getBluePulseWidth)(void);
}TCS320_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// Calibration constants
static const uint16_t TCS320_RED_MIN = 55;
static const uint16_t TCS320_RED_MAX = 1400;
static const uint16_t TCS320_GREEN_MIN = 10;
static const uint16_t TCS320_GREEN_MAX = 1620;
static const uint16_t TCS320_BLUE_MIN = 10;
static const uint16_t TCS320_BLUE_MAX = 1200;

// Input-Capture variables
static uint32_t captures[2] = {0};
static volatile uint8_t captureDone = 0;
static uint8_t isRisingEdge = 1;
static uint16_t pulseWidth = 0;

// TCS320 Colour Sensor initialization
static TCS320_t colourSensor = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

// Prototypes of the TCS320_t function pointers
static int getRedPulseWidth(void);
static int getGreenPulseWidth(void);
static int getBluePulseWidth(void);

// Standard mapping function
static long map(long x, long in_min, long in_max, long out_min, long out_max);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  char msg[64] = {0};
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
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  sprintf(msg, "RGB Colour Sensor Demo Program\r\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  // Set Colour Sensors frequency scaling to 20%
  HAL_GPIO_WritePin(GPIOE, S0Pin_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(S1Pin_GPIO_Port, S1Pin_Pin, GPIO_PIN_RESET);

  // Assign function pointers to read RGB Pulse widths
  colourSensor.getRedPulseWidth = getRedPulseWidth;
  colourSensor.getGreenPulseWidth = getGreenPulseWidth;
  colourSensor.getBluePulseWidth = getBluePulseWidth;

  // Do Calibration (1 for yes, 0 for not)
  colourSensor.doCalibration = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Set sensor to read Red only
	  int r = colourSensor.getRedPulseWidth();
	  HAL_Delay(200);

	  // Set sensor to read Green only
	  int g = colourSensor.getGreenPulseWidth();
	  HAL_Delay(200);

	  // Set sensor to read Blue only
	  int b = colourSensor.getBluePulseWidth();

	  // Send r,g,b values over UART to PC running Classification program.
	  sprintf(msg, "%d,%d,%d\r\n", r, g, b);
	  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	  HAL_Delay(200);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(S1Pin_GPIO_Port, S1Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, S3Pin_Pin|S2Pin_Pin|S0Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : S1Pin_Pin */
  GPIO_InitStruct.Pin = S1Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(S1Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S3Pin_Pin S2Pin_Pin S0Pin_Pin */
  GPIO_InitStruct.Pin = S3Pin_Pin|S2Pin_Pin|S0Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static long map(long x, long in_min, long in_max, long out_min, long out_max){

	in_max += 1;
	out_max += 1;

	long ret = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

	if((ret < out_min) && (out_min > out_max)){
		return ret;
	}
	else if((ret < out_min) && (out_min < out_max)){
		return out_min;
	}
	else if((ret > out_max) && (out_min > out_max)){
		return ret;
	}
	else if((ret > out_max) && (out_min < out_max)){
		return out_max;
	}
	else{
		return ret;
	}

}

static int getRedPulseWidth(void){

	// Set Sensor to read Red only
	HAL_GPIO_WritePin(S2Pin_GPIO_Port, S2Pin_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(S3Pin_GPIO_Port, S3Pin_Pin, GPIO_PIN_RESET);

	const int averagePoints = 25;
	int avg = 0;
	char data[32] = {0};

	for(int i = 0; i < averagePoints; ++i){

		// Start the input capture in interrupt mode
		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

		while(!captureDone){}

		colourSensor.redPulseWidth = pulseWidth;

		colourSensor.redValue = map(colourSensor.redPulseWidth, TCS320_RED_MIN,
				TCS320_RED_MAX, 255, 0);

		if(colourSensor.doCalibration){
			sprintf(data, "rw: %d, rv: %d\r\n", colourSensor.redPulseWidth, colourSensor.redValue);
			HAL_UART_Transmit(&huart3, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
		}
		avg += colourSensor.redValue;

		captureDone = 0;

		memset(data, '\0', sizeof(data));
	}


	return (int)(avg/averagePoints);
}

static int getGreenPulseWidth(void){

	// Set Sensor to read Green only
	HAL_GPIO_WritePin(S2Pin_GPIO_Port, S2Pin_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(S3Pin_GPIO_Port, S3Pin_Pin, GPIO_PIN_SET);

	char data[32] = {0};
	const int averagePoints = 25;
	int avg = 0;

	for(int i = 0; i < averagePoints; ++i){

		// Start the input capture in interrupt mode
		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

		while(!captureDone){}

		colourSensor.greenPulseWidth = pulseWidth;

		colourSensor.greenValue = map(colourSensor.greenPulseWidth, TCS320_GREEN_MIN,
				TCS320_GREEN_MAX, 255, 0);

		if(colourSensor.doCalibration){
			sprintf(data, "gw: %d, gv: %d\r\n", colourSensor.greenPulseWidth, colourSensor.greenValue);
			HAL_UART_Transmit(&huart3, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
		}
		avg += colourSensor.greenValue;

		captureDone = 0;

		memset(data, '\0', sizeof(data));
	}

	return (int)(avg/averagePoints);
}

static int getBluePulseWidth(void){
	// Set Sensor to read Blue only
	HAL_GPIO_WritePin(S2Pin_GPIO_Port, S2Pin_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(S3Pin_GPIO_Port, S3Pin_Pin, GPIO_PIN_SET);

	char data[32] = {0};
	const int averagePoints = 25;
	int avg = 0;

	for(int i = 0; i < averagePoints; ++i){

		// Start the input capture in interrupt mode
		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

		while(!captureDone){}

		colourSensor.bluePulseWidth = pulseWidth;

		colourSensor.blueValue = map(colourSensor.bluePulseWidth, TCS320_BLUE_MIN,
				TCS320_BLUE_MAX, 255, 0);

		if(colourSensor.doCalibration){
			sprintf(data, "bw: %d, bv: %d\r\n", colourSensor.bluePulseWidth, colourSensor.blueValue);
			HAL_UART_Transmit(&huart3, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
		}
		avg += colourSensor.blueValue;
		captureDone = 0;

		memset(data, '\0', sizeof(data));
	}

	return (int)(avg/averagePoints);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){

		if(isRisingEdge){ // rising edge
			captures[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			isRisingEdge = 0;
		}
		else{ // falling edge
			captures[1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			isRisingEdge = 1;

			// reset counter
			__HAL_TIM_SET_COUNTER(htim, 0);

			if(captures[1] >= captures[0]){
				pulseWidth = captures[1] - captures[0];
			}
			else{
				pulseWidth = (htim->Instance->ARR - captures[0]) + captures[1];
			}
			captureDone = 1;

			// Stop the input capture in interrupt mode
			HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);

		}
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
