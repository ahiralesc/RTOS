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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for selenoidControl */
osThreadId_t selenoidControlHandle;
const osThreadAttr_t selenoidControl_attributes = {
  .name = "selenoidControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ctrlMsgQueue */
osMessageQueueId_t ctrlMsgQueueHandle;
const osMessageQueueAttr_t ctrlMsgQueue_attributes = {
  .name = "ctrlMsgQueue"
};
/* Definitions for selenoidQueue */
osMessageQueueId_t selenoidQueueHandle;
const osMessageQueueAttr_t selenoidQueue_attributes = {
  .name = "selenoidQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void coordinatorHandler(void *argument);
void selenoidControlerHandler(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t buffer[6];

// General control message
typedef struct{
	uint8_t buffer[6];
	uint8_t id;
} Ctrl_msg;


typedef enum {
	SELENOID = 0,
	PUMP,
	HUMIDITY
} CRT_Type;


// Message format for LED control (LED_Crt_msg)
typedef struct{
	uint8_t frequency;
	uint8_t duration;
	uint8_t id;
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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  HAL_UART_Receive_IT(&huart3, buffer, sizeof(buffer));
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ctrlMsgQueue */
  ctrlMsgQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &ctrlMsgQueue_attributes);

  /* creation of selenoidQueue */
  selenoidQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &selenoidQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of coordinator */
  coordinatorHandle = osThreadNew(coordinatorHandler, NULL, &coordinator_attributes);

  /* creation of selenoidControl */
  selenoidControlHandle = osThreadNew(selenoidControlerHandler, NULL, &selenoidControl_attributes);

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
{  // {cmd: "selenoide", parametros : { "frecuencia" : 10, duracion, "10m"}0000000000"
	Ctrl_msg msg;
	if(huart->Instance == USART3){
		// Receive the USART message.
		HAL_UART_Receive_IT(&huart3, buffer, sizeof(buffer));

		// Copy by value the contents of buffer to the control message.
		strcpy((char *) msg.buffer, (const char *) buffer);

		// Push the control message to the queue. Do not wait.
		osMessageQueuePut(ctrlMsgQueueHandle, &msg, 0U, 0U);
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
	osStatus_t status;
	JSONStatus_t result;
	Ctrl_msg msg;
	char key [] = "cmd";	// command type.
	char * val;
	size_t key_length = sizeof(key) - 1;
	size_t val_length;
	CRT_Type ctrMsg;


	for(;;)
	{
		status = osMessageGet(ctrlMsgQueueHandle,  &msg, NULL, osWaitForever);
		if(status == osOK) {
			size_t msg_length = sizeof(msg.buffer) - 1;
			result = JSON_Validate(msg.buffer, msg_length);
			if (result == JSONSuccess){
				// Evaluate to which controller the message corresponds
				result = JSON_Search(msg.buffer, msg_length, key, key_length, val, val_length);
				if (result == JSONSuccess){
					CRT_Type cmd = atoi(val);

					// Forward the message to the appropriate controller.
					switch(cmd)
					{
						case SELENOID:
							osMessageQueuePut(selenoidQueueHandle, &msg, 0U, 0U);
							break;
						case PUMP:
							break;
						case HUMIDITY:
							break;
						default: // An unrecognized control message.
					}
				}
			}
		}
	}
  /* USER CODE END coordinatorHandler */
}

/* USER CODE BEGIN Header_selenoidControlerHandler */
/**
* @brief Function implementing the selenoidControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_selenoidControlerHandler */
void selenoidControlerHandler(void *argument)
{
	/* USER CODE BEGIN selenoidControlerHandler */
	/* Infinite loop */
	osStatus_t status;
	char key1 [] = "id";
	char key2 [] = "duration";
	char key3 [] = "frequency";
	char * val;
	size_t val_length;
	SELENOID_Ctrl_msg selenoid;

	for(;;)
	{
		status = osMessageGet(selenoidQueueHandle,  &msg, NULL, 0);

		// Control message preparation
		if(status == osOK) {
			size_t msg_length = sizeof(msg.buffer) - 1;
			JSON_Search(msg.buffer, msg_length, key1, sizeof(key1)-1, val, val_length);
			selenoid.id = val;
			JSON_Search(msg.buffer, msg_length, key2, sizeof(key2)-1, val, val_length);
			selenoid.duration = val;
			JSON_Search(msg.buffer, msg_length, key3, sizeof(key3)-1, val, val_length);
			selenoid.frequency = val;
		}

		// Execution
	}
  /* USER CODE END selenoidControlerHandler */
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
