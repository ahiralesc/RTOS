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
// 1. For sprintf function
#include "stdio.h"
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
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay(uint16_t length) {
	/* delay in micro-seconds */
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while((__HAL_TIM_GET_COUNTER(&htim6))<length);
}

float temperature = 0.0;
float humidity = 0.0;
uint8_t state = 0;


void set_pin_output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_Init_Struct = {0};
	GPIO_Init_Struct.Pin = GPIO_Pin;
	GPIO_Init_Struct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Init_Struct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_Init_Struct);
}

void set_pin_input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_Init_Struct = {0};
	GPIO_Init_Struct.Pin = GPIO_Pin;
	GPIO_Init_Struct.Mode = GPIO_MODE_INPUT;
	GPIO_Init_Struct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_Init_Struct);
}

/* **************** Start DHT11 Functions **************** */
#define DHT11_PORT DHT11_PA3_GPIO_Port
#define DHT11_PIN DHT11_PA3_Pin

void DHT11_start(void){
	set_pin_output(DHT11_PORT, DHT11_PIN);
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
	delay(18000); // 18 ms
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
	delay(20);
	set_pin_input(DHT11_PORT, DHT11_PIN);
}

uint8_t DHT11_state(void){
	uint8_t response = 0;
	delay(40);
	if(!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))){
		delay(20);
		if(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
				response = 1;
		else 	response = -1;
	}
	// wait for pin to go low
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
	return response;
}

uint8_t DHT11_read(void){
	uint8_t i, j;
	for( j=0; j<8; j++){
		// wait for the pin to go high
		while(!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));
		// wait 40 us
		delay(40);
		if( !(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) ) {
			i &= ~(1<<(7-j));
		} else i |= (1<<(7-j));
		// wait for the pin to go low
		while((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));
	}
	return i;
}


/* **************** End DHT11 Functions **************** */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char snd_buff[20] = {0};
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
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);

  sprintf(snd_buff, "Incia inst %d\r\n", 0);
  HAL_UART_Transmit_IT(&huart3, snd_buff, sizeof(snd_buff));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/*	  DHT11_start();
	  state = DHT11_state();
	  if(state) {
		  uint8_t rh_byte1 = DHT11_read();
		  uint8_t rh_byte2 = DHT11_read();
		  uint8_t temp_byte1 = DHT11_read();
		  uint8_t temp_byte2 = DHT11_read();
		  uint8_t checksum = DHT11_read();
		  temperature = (float) temp_byte1;
		  humidity = (float) rh_byte1;

		  sprintf(snd_buff, "Temp: %.2f\r\n", temperature);
		  HAL_UART_Transmit_IT(&huart3, snd_buff, sizeof(snd_buff));
		  sprintf(snd_buff, "Hum: %.2f\r\n", humidity );
		  HAL_UART_Transmit_IT(&huart3, snd_buff, sizeof(snd_buff));
	  }else{
		  sprintf(snd_buff, "valio\r\n", temperature);
		  HAL_UART_Transmit_IT(&huart3, snd_buff, sizeof(snd_buff));
	  }*/

	  sprintf(snd_buff, "basura %d\r\n", 0);
	    HAL_UART_Transmit_IT(&huart3, snd_buff, sizeof(snd_buff));
	  HAL_Delay(1000);
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim6.Init.Prescaler = 50-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_PA3_GPIO_Port, DHT11_PA3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DHT11_PA3_Pin */
  GPIO_InitStruct.Pin = DHT11_PA3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_PA3_GPIO_Port, &GPIO_InitStruct);

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
