/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

//osThreadId defaultTaskHandle;
osThreadId ReadingTaskHandle;
osThreadId UploadTaskHandle;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART5_Init(void);
//void StartDefaultTask(void const * argument);
void ReadingTaskHook(void const * argument);
void UploadTaskHook(void const * argument);

/* USER CODE BEGIN PFP */
void sending_function();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

xQueueHandle xQueue1;
uint16_t raw;
SemaphoreHandle_t xData_Available;
SemaphoreHandle_t xData_cont;
int middle = 0;
uint8_t buffer3[5];
uint8_t whoops[] = "Whoops\r\n";

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
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  xQueue1 = xQueueCreate(20, sizeof(char));
  xData_Available = xSemaphoreCreateBinary();
  xData_cont = xSemaphoreCreateBinary();

  //xSemaphoreGive( xData_cont );

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(ReadingTask, ReadingTaskHook, osPriorityNormal, 0, 128);
  ReadingTaskHandle = osThreadCreate(osThread(ReadingTask), NULL);
  osThreadDef(UploadTask, UploadTaskHook, osPriorityNormal, 0, 128);
  UploadTaskHandle = osThreadCreate(osThread(UploadTask), NULL);
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
uint16_t *myString;

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

void UploadTaskHook(void const * argument)
{
	for(;;)
	{
		xSemaphoreTake( xData_Available,  portMAX_DELAY);
		char msg2[10];
		sprintf(msg2, "%lu\r\n", uxQueueSpacesAvailable( xQueue1 ));
		HAL_UART_Transmit(&huart3, (uint8_t*)msg2, strlen(msg2), 100);
		char msg3[5];
		sprintf(msg3, "%u\r\n", *myString);
		HAL_UART_Transmit(&huart3, (uint8_t*)msg3, strlen(msg3), 100);
		//xQueueSend(xQueue1, msg3, portMAX_DELAY);
		for(int i = 0; i<sizeof(msg3); i++){
			if( xQueueSend( xQueue1, &msg3+i, ( TickType_t ) 0 ) != pdPASS )
			{
				uint8_t buffer9[] = "Failed to Upload\r\n";
				HAL_UART_Transmit(&huart3, buffer9, sizeof(buffer9), 100);
			} else {
				uint8_t buffer9[] = "Upload :)\r\n";
				HAL_UART_Transmit(&huart3, buffer9, sizeof(buffer9), 100);
			}
		}
		if (uxQueueSpacesAvailable( xQueue1 ) == 0){
			sending_function();
		}
		xSemaphoreGive( xData_cont);
	}
}

void ReadingTaskHook(void const * argument)
{
	TickType_t tickCount;
		/* The cycle time period. The task will be unblocked at time (in ticks)*/
		TickType_t frequency = 5000;

		/* Get the current tck count */
		tickCount = xTaskGetTickCount();

		uint8_t buffer[19] = "AT+BAND=868500000\r\n";
		uint8_t buffer4[16] = "AT+NETWORKID=5\r\n";
		uint8_t buffer5[16] = "AT+ADDRESS=101\r\n";

		HAL_UART_Transmit(&huart5, buffer, sizeof(buffer), 100);

		HAL_StatusTypeDef status1 = HAL_UART_Receive(&huart5, buffer3, sizeof(buffer3), 1000);
		if(status1 != HAL_OK){
		   HAL_UART_Transmit(&huart3, whoops, sizeof(whoops), 100);
		}
		else{
		   HAL_UART_Transmit(&huart3, buffer3, sizeof(buffer3), 100);
		}

		vTaskDelayUntil( &tickCount, 1000 );

		HAL_UART_Transmit(&huart5, buffer4, sizeof(buffer4), 100);

		HAL_StatusTypeDef status2 = HAL_UART_Receive(&huart5, buffer3, sizeof(buffer3), 1000);
		if(status2 != HAL_OK){
    	   HAL_UART_Transmit(&huart3, whoops, sizeof(whoops), 100);
		}
		else{
		   HAL_UART_Transmit(&huart3, buffer3, sizeof(buffer3), 100);
		}

		vTaskDelayUntil( &tickCount, 1000 );

		HAL_UART_Transmit(&huart5, buffer5, sizeof(buffer5), 100);

   	    HAL_StatusTypeDef status4 = HAL_UART_Receive(&huart5, buffer3, sizeof(buffer3), 1000);
		if(status4 != HAL_OK){
		   HAL_UART_Transmit(&huart3, whoops, sizeof(whoops), 100);
		}
		else{
		   HAL_UART_Transmit(&huart3, buffer3, sizeof(buffer3), 100);
		}

		vTaskDelayUntil( &tickCount, 1000 );

		for(;;)
		{
			uint8_t buffer1[] = "Reading\r\n";
			HAL_UART_Transmit(&huart3, buffer1, sizeof(buffer1), 100);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			raw = HAL_ADC_GetValue(&hadc1);

			if(raw < 60){
					  if (middle == 0){
						  raw = 10;
					  } else {
						  raw = 40;
					  }
				  }
				  else{
					  if(raw < 135){
						  raw = 20;
						  middle = 0;
					  }
					  else {
						 raw = 30;
						 middle = 1;
					  }
				  }
			myString=&raw;
			xSemaphoreGive(xData_Available);
			xSemaphoreTake(xData_cont,  portMAX_DELAY);
			vTaskDelayUntil( &tickCount, frequency );
		}
}

void sending_function() {
	char msg[20];
	TickType_t tickCount;
	tickCount = xTaskGetTickCount();
	  /*uint8_t buffer3[5];
	  uint8_t whoops[] = "Whoops\r\n";*/

	for(int i = 0; i<4; i++){
		xQueueReceive( xQueue1, &msg, ( TickType_t ) 0 );
		char buffer6[17];
		uint8_t buffer_long[20];
		sprintf(buffer6, "AT+SEND=102,3,%c\r\n", *msg);
		HAL_UART_Transmit(&huart3, buffer6, sizeof(buffer6), 100);

		HAL_UART_Transmit(&huart5, buffer6, sizeof(buffer6), 100);

		HAL_StatusTypeDef status1 = HAL_UART_Receive(&huart5, buffer_long, sizeof(buffer_long), 1000);
	    if(status1 != HAL_OK){
		   HAL_UART_Transmit(&huart3, whoops, sizeof(whoops), 100);
	    }
	    else{
	       HAL_UART_Transmit(&huart3, buffer_long, sizeof(buffer_long), 100);
	    }

	    vTaskDelayUntil( &tickCount, 1000 );

		xQueueReceive( xQueue1, &msg, ( TickType_t ) 0 );
		xQueueReceive( xQueue1, &msg, ( TickType_t ) 0 );
		xQueueReceive( xQueue1, &msg, ( TickType_t ) 0 );
		xQueueReceive( xQueue1, &msg, ( TickType_t ) 0 );
	}

	//xQueueReceive( xQueue1, &( msg ), ( TickType_t ) 0 );
	//HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
	//xQueueReset( xQueue1 );
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
