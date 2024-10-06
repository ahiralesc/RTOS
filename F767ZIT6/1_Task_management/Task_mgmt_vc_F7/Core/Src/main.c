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
#include "cmsis_os.h"

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

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for coordTask */
osThreadId_t coordTaskHandle;
const osThreadAttr_t coordTask_attributes = {
  .name = "coordTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for controlTaskOne */
osThreadId_t controlTaskOneHandle;
const osThreadAttr_t controlTaskOne_attributes = {
  .name = "controlTaskOne",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for controlTaskTwo */
osThreadId_t controlTaskTwoHandle;
const osThreadAttr_t controlTaskTwo_attributes = {
  .name = "controlTaskTwo",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for coordQueue */
osMessageQueueId_t coordQueueHandle;
const osMessageQueueAttr_t coordQueue_attributes = {
  .name = "coordQueue"
};
/* Definitions for crtQueueOne */
osMessageQueueId_t crtQueueOneHandle;
const osMessageQueueAttr_t crtQueueOne_attributes = {
  .name = "crtQueueOne"
};
/* Definitions for crtQueueTwo */
osMessageQueueId_t crtQueueTwoHandle;
const osMessageQueueAttr_t crtQueueTwo_attributes = {
  .name = "crtQueueTwo"
};
/* Definitions for cmdEvent */
osEventFlagsId_t cmdEventHandle;
const osEventFlagsAttr_t cmdEvent_attributes = {
  .name = "cmdEvent"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void coordTaskImpl(void *argument);
void controlTaskOneImpl(void *argument);
void controlTaskTwoImpl(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Control message format Ver. 1.
 *  char ctrl_msg[] = {
 *		"sen": "XYZ",  // 3 character sensor type
 *  	"sid": "123",  // 3 digit sensor id
 *  	"cmd": "ON_",  // 3 character command
 *		"dur": "045",  // 3 digit duration in minutes
 *		"val": "075",  // 3 digit value (optional, depending on the sensor type)
 *		"sts": "OK_",  // 3 character status (OK_, ERR, etc.)
 *	}
 * Example messages :
 * REL001ON_005162OK_000---
 * CO2003MEA120162OK_400PPM
 * */


// 2. Define the message queue message format.

#define MSG_SZ 46

typedef struct{
	uint8_t buffer[MSG_SZ];
	uint8_t sID;
} MSQ_TYPE;


#define CMD_RX_EVNT  0x00000001U // Command received event.
#define LED_ON_EVNT  0x00000002U // LED on event.
#define LED_OFF_EVNT 0x00000004U // LED off event.


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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of coordQueue */
  coordQueueHandle = osMessageQueueNew (16, sizeof(MSQ_TYPE), &coordQueue_attributes);

  /* creation of crtQueueOne */
  crtQueueOneHandle = osMessageQueueNew (16, sizeof(MSQ_TYPE), &crtQueueOne_attributes);

  /* creation of crtQueueTwo */
  crtQueueTwoHandle = osMessageQueueNew (16, sizeof(MSQ_TYPE), &crtQueueTwo_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of coordTask */
  coordTaskHandle = osThreadNew(coordTaskImpl, NULL, &coordTask_attributes);

  /* creation of controlTaskOne */
  controlTaskOneHandle = osThreadNew(controlTaskOneImpl, NULL, &controlTaskOne_attributes);

  /* creation of controlTaskTwo */
  controlTaskTwoHandle = osThreadNew(controlTaskTwoImpl, NULL, &controlTaskTwo_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of cmdEvent */
  cmdEventHandle = osEventFlagsNew(&cmdEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IGreen_Pin|IRed_Pin|IBlue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IGreen_Pin IRed_Pin IBlue_Pin */
  GPIO_InitStruct.Pin = IGreen_Pin|IRed_Pin|IBlue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	char			buffer[MSG_SZ];
	MSQ_TYPE    	msg;
	osStatus_t   	msq_status;
	JSONStatus_t 	json_status;

	// Receive a fixed size JSON formatted message
	HAL_UART_Receive_IT(&huart3, buffer, sizeof(buffer));
	// Validate that the JSON message format is correct
	json_status = JSON_Validate(buffer, sizeof(buffer)-1);

	if(json_status == JSONSuccess) {
		// Prepare the control message
		strcpy(msg.buffer, buffer);
		msg.sID = 0U;
		// Place the control message in the message queue
		msq_status = osMessageQueuePut(controlMsgQueueHandle, &msg, 0U, 0U);
		if(msq_status == 0){
			// Signal the control task
			osEventFlagsSet(cmdEventHandle, CMD_RX_EVNT);
		}
	}
}


void ledControl(const uint8_t *args){
	char id[4];
	// Extract the ID and option
	strncpy(id, (char*)args, 3);
	id[3] = '\x0';
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_coordTaskImpl */
/**
* @brief Function implementing the coordTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_coordTaskImpl */
void coordTaskImpl(void *argument)
{
  /* USER CODE BEGIN coordTaskImpl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END coordTaskImpl */
}

/* USER CODE BEGIN Header_controlTaskOneImpl */
/**
* @brief Function implementing the controlTaskOne thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_controlTaskOneImpl */
void controlTaskOneImpl(void *argument)
{
  /* USER CODE BEGIN controlTaskOneImpl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END controlTaskOneImpl */
}

/* USER CODE BEGIN Header_controlTaskTwoImpl */
/**
* @brief Function implementing the controlTaskTwo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_controlTaskTwoImpl */
void controlTaskTwoImpl(void *argument)
{
  /* USER CODE BEGIN controlTaskTwoImpl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END controlTaskTwoImpl */
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
