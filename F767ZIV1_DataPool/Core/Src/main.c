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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "data_pool.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

osThreadId toggleRedHandle;
osThreadId toggleGreenHandle;
osThreadId rateControlHandle;
osSemaphoreId dataPoolSempHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void toggleRedHook(void const * argument);
void toggleGreenHook(void const * argument);
void rateControlHook(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void toggle(GPIO_TypeDef*  GPIOx, uint16_t GPIO_Pin, uint32_t frequency, int duration);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of dataPoolSemp */
  osSemaphoreDef(dataPoolSemp);
  dataPoolSempHandle = osSemaphoreCreate(osSemaphore(dataPoolSemp), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of toggleRed */
  osThreadDef(toggleRed, toggleRedHook, osPriorityNormal, 0, 128);
  toggleRedHandle = osThreadCreate(osThread(toggleRed), NULL);

  /* definition and creation of toggleGreen */
  osThreadDef(toggleGreen, toggleGreenHook, osPriorityNormal, 0, 128);
  toggleGreenHandle = osThreadCreate(osThread(toggleGreen), NULL);

  /* definition and creation of rateControl */
  osThreadDef(rateControl, rateControlHook, osPriorityNormal, 0, 128);
  rateControlHandle = osThreadCreate(osThread(rateControl), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ERed_GPIO_Port, ERed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EGreen_GPIO_Port, EGreen_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ERed_Pin */
  GPIO_InitStruct.Pin = ERed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ERed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EGreen_Pin */
  GPIO_InitStruct.Pin = EGreen_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EGreen_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void toggle(GPIO_TypeDef*  GPIOx, uint16_t GPIO_Pin, uint32_t frequency, int duration)
{
	TickType_t start = xTaskGetTickCount();

	do {
		HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
		HAL_Delay(frequency);
	} while((xTaskGetTickCount() - start) <= duration);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_toggleRedHook */
/**
  * @brief  Function implementing the toggleRed thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_toggleRedHook */
void toggleRedHook(void const * argument)
{
	int frequency = 0;
	for(;;)
	{
		/* Sleep for half a second, so that the rateColtrol task starts first */
		osDelay(500);

		/* Get the toggle rate from the shared data pool. Both the get and set rate
		 * functions are shared. They use a semaphore to protect access to the data
		 * pool. But, no semaphores are visible here.
		 */
		frequency = get_frequency(green);

		toggle(EGreen_GPIO_Port, EGreen_Pin, frequency, 2000);
	}
}

/* USER CODE BEGIN Header_toggleGreenHook */
/**
* @brief Function implementing the toggleGreen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_toggleGreenHook */
void toggleGreenHook(void const * argument)
{
	int frequency = 0;

	for(;;)
	{
		/* Sleep for half a second, so that the rateColtrol task starts first */
		osDelay(500);

		/* Get the toggle rate from the shared data pool. Both the get and set rate
		 * functions are shared. They use a semaphore to protect access to the data
		 * pool. But, no semaphores are visible here.
		 */
		frequency = get_frequency(red);

		toggle(ERed_GPIO_Port, ERed_Pin, frequency, 2000);
	}
}


/* USER CODE BEGIN Header_rateControlHook */
/**
* @brief Function implementing the rateControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rateControlHook */
void rateControlHook(void const * argument)
{
	int rate = 0;
	int next = 0;

	for(;;)
	{
		/* Set the flashing rate at random */
		rate = 100 + rand() % 300;

		/* Select a task randomly */
		next = rand() % 2;
		if( next == 0 )
			set_frequency(red, rate);
		else
			set_frequency(green, rate);

		HAL_Delay(rand() % 2500);
	}
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
