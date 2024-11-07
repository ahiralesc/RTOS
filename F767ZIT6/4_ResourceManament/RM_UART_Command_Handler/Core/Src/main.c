/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "core_json.h"
#include "stdlib.h"
#include <string.h>
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
/* Definitions for coordinator */
osThreadId_t coordinatorHandle;
const osThreadAttr_t coordinator_attributes = {
  .name = "coordinator",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for selenoidControl */
osThreadId_t selenoidControlHandle;
const osThreadAttr_t selenoidControl_attributes = {
  .name = "selenoidControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for selenoidMsgQueue */
osMessageQueueId_t selenoidMsgQueueHandle;
const osMessageQueueAttr_t selenoidMsgQueue_attributes = {
  .name = "selenoidMsgQueue"
};
/* Definitions for deferredCoordinatorBS */
osSemaphoreId_t deferredCoordinatorBSHandle;
const osSemaphoreAttr_t deferredCoordinatorBS_attributes = {
  .name = "deferredCoordinatorBS"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void coordinatorHandler(void *argument);
void selenoidControllerHandler(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 *  The test message is assumed constant. i.e. {"cmd":"00","id":"01","frequency":"30","duration":"01"}
 *  The length of the message is 44 + 1 for \n.
 */
#define BUFFER_SZ 55
typedef char * Ctrl_msg;


// UART buffer
uint8_t buffer[BUFFER_SZ];
uint8_t ctrBuffer[BUFFER_SZ];

typedef enum {
	SELENOID = 0, // SELENOID
	PUMP,
	HUMIDITY      // HUMIDITY
} CRT_Type;


// Message format for LED control (LED_Crt_msg)
typedef struct{
	uint8_t id;
	uint8_t frequency;
	uint8_t duration;
} SELENOID_Ctrl_msg;


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

  /* Create the semaphores(s) */
  /* creation of deferredCoordinatorBS */
  deferredCoordinatorBSHandle = osSemaphoreNew(1, 0, &deferredCoordinatorBS_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  HAL_UART_Receive_IT(&huart3, buffer, BUFFER_SZ);

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of selenoidMsgQueue */
  selenoidMsgQueueHandle = osMessageQueueNew (16, sizeof(Ctrl_msg), &selenoidMsgQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of coordinator */
  coordinatorHandle = osThreadNew(coordinatorHandler, NULL, &coordinator_attributes);

  /* creation of selenoidControl */
  selenoidControlHandle = osThreadNew(selenoidControllerHandler, NULL, &selenoidControl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IGreen_Pin|IRed_Pin|IBlue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IGreen_Pin IRed_Pin IBlue_Pin */
  GPIO_InitStruct.Pin = IGreen_Pin|IRed_Pin|IBlue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		// Receive the USART message.
		HAL_UART_Receive_IT(&huart3, buffer, sizeof(buffer));

		// Copy and signal to deferred processing to the coordinatorHandle
		memcpy(ctrBuffer, buffer, BUFFER_SZ);
		ctrBuffer[BUFFER_SZ] = '\0';

		osSemaphoreRelease(deferredCoordinatorBSHandle);
	}
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

/* USER CODE BEGIN Header_coordinatorHandler */
/**
* @brief Function implementing the coordinator thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_coordinatorHandler */
void coordinatorHandler(void *argument)
{
  /* USER CODE BEGIN coordinatorHandler */
	/* Infinite loop */
	//osStatus_t status;

	// JSON variables
	const char key [] = "cmd";	// command type.
	size_t key_length = sizeof(key) - 1;

	// Extracted values from JSON
	char * val;
	size_t val_length;


	for(;;)
	{
		if (osSemaphoreAcquire(deferredCoordinatorBSHandle, osWaitForever) == osOK){
			size_t msg_length = sizeof(ctrBuffer);
			if (JSON_Validate( (const char *) ctrBuffer, msg_length ) == JSONSuccess){
				// Evaluate to which controller the message corresponds
				if (JSON_Search( (char *) ctrBuffer, msg_length, key, key_length, &val, &val_length) == JSONSuccess){
					CRT_Type cmd = atoi(val);

					// Forward the message to the appropriate controller.
					switch(cmd)
					{
						case SELENOID:
							// Parse the selenoid control message
							SELENOID_Ctrl_msg selenoid;
							if (JSON_Search( (char *) ctrBuffer, msg_length, "id", 2, &val, &val_length) == JSONSuccess)
								selenoid.id = atoi(val);
							if (JSON_Search( (char *) ctrBuffer, msg_length, "duration", 8, &val, &val_length) == JSONSuccess)
								selenoid.duration = atoi(val);
							if (JSON_Search( (char *) ctrBuffer, msg_length, "frequency", 9, &val, &val_length) == JSONSuccess)
								selenoid.frequency = atoi(val);
							osMessageQueuePut(selenoidMsgQueueHandle, &selenoid, 0U, 0U);
							break;
						case PUMP:
							// Parse the pump control message
							break;
						case HUMIDITY:
							// Parse the humidity sensor control message
							break;
							// Unrecognized control message
						default:
					}
				}
			}

			// Reset the control buffer
			memset(ctrBuffer, 0, BUFFER_SZ);
		}
	}
  /* USER CODE END coordinatorHandler */
}

/* USER CODE BEGIN Header_selenoidControllerHandler */
/**
* @brief Function implementing the selenoidControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_selenoidControllerHandler */
void selenoidControllerHandler(void *argument)
{
  /* USER CODE BEGIN selenoidControllerHandler */
	SELENOID_Ctrl_msg selenoid;
	Ctrl_msg msg;
  /* Infinite loop */
	for(;;)
	{
		osStatus_t status = osMessageQueueGet(selenoidMsgQueueHandle,  &selenoid, NULL, 0);
		// Perform the work in the selenoid that corresponds.

  }
  /* USER CODE END selenoidControllerHandler */
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
