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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"  // For memcpy
#include "stdlib.h"  // For abs
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
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId defaultTaskHandle;
osThreadId consumerTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
void consumerTaskHook(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * 1. Assume that an application transmits a large text file in chunks. These chunks are received via
 *    the fixed size UART rx_buffer which will be smaller than the UART circular buffer.
 *    and the size of the application
 */

#define RX_RB_SZ 5
#define RX_RCB_SZ 10
#define RX_RCB_EMPTY 1


// Receive Circular Buffer
typedef struct RecvCBuffer {
	uint8_t rxb[RX_RB_SZ];     // The receive buffer (or message)
	uint8_t rxcb[RX_RCB_SZ];   // The circular buffer
	int16_t head;
	int16_t tail;
} _RXCB_;


/*
 * 2. The receive circular buffer data structure
 */
_RXCB_ rcb = {.tail = -1, .head = -1};

int16_t drop_msgs = 0;
int16_t read_msgs = 0;
/*
 * 3. Declare the rx receive buffer
 */
//uint8_t rx_buffer[RX_BUFFER_SZ];



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
  //rcb.head = -1;
  //rcb.tail = -1;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rcb.rxb, RX_RB_SZ);
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of consumerTask */
  osThreadDef(consumerTask, consumerTaskHook, osPriorityNormal, 0, 128);
  consumerTaskHandle = osThreadCreate(osThread(consumerTask), NULL);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t rxb_sz){

	if( huart->Instance == USART3) {

		// The is space in front of the buffer
		if( rcb.head > rcb.tail  && rxb_sz <= (RX_RCB_SZ - (rcb.head - rcb.tail )) )
		{
			// write at the free locations at the end of the buffer
			int16_t k = ((RX_RCB_SZ - rcb.head) > rxb_sz)? rxb_sz : RX_RCB_SZ - rcb.head;
			memcpy( (uint8_t *) rcb.rxcb + rcb.head, (uint8_t *) rcb.rxb, k );
			if(  (rxb_sz - k) > 0 )
				memcpy( (uint8_t *) rcb.rxcb, (uint8_t *) rcb.rxb + k, (rxb_sz - k) );
			rcb.head = rcb.head + rxb_sz % RX_RCB_SZ;
		}


		// There is space somewhere in the beginning of the buffer
		if( rcb.head < rcb.tail &&  rxb_sz <= (RX_RCB_SZ - abs(rcb.head - rcb.tail )) )
		{
			memcpy( (uint8_t *) rcb.rxcb + rcb.head, (uint8_t *) rcb.rxb, rxb_sz);
			rcb.head = rcb.head + rxb_sz;
		}


		// If the buffer is full or the incoming message does not fit, then it is drop and the
		// pending task is notified
		if( rcb.head == rcb.tail || rxb_sz > (RX_RCB_SZ - abs(rcb.head - rcb.tail )) ) {
			// must signal the consumer task that the buffer is full.
			drop_msgs++;
		}

		// The circular buffer is empty
		if( rcb.tail == -1 && rcb.head == -1) {
			memcpy( (uint8_t *) rcb.rxcb, (uint8_t *) rcb.rxb, rxb_sz);
			rcb.tail = 0;
			rcb.head = rxb_sz;
		}

		// Reset the UART and disable half transfer mode
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rcb.rxb, RX_RB_SZ);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

	}
}



char poll(uint8_t * rxb, int sz )
{
	// the buffer is empty
	if(rcb.head == rcb.tail)
		return RX_RCB_EMPTY;

	// the buffer has not been initiated
	if(rcb.head == -1 && rcb.tail == -1)
		return RX_RCB_EMPTY;

	// estimate the number of characters in the buffer
	int16_t k = ( rcb.head > rcb.tail )?  rcb.head - rcb.tail : (RX_RCB_SZ - rcb.tail) + rcb.head;

	// elements in the buffer are located at the end of it
	if( rcb.head > rcb.tail )
	{
		memcpy((uint8_t *) rxb, (uint8_t *) rcb.rxcb + rcb.tail, k);
		// no need to reset the buffer, just move the pointers
		rcb.tail = rcb.tail + k;
		// TODO: evaluate if a tailing character must be added to the rxb
	}

	// elements in the buffer are located at the end and at the beginning
	if( rcb.tail > rcb.head )
	{
		// copy elements at the end of the buffer first
		memcpy((uint8_t *) rxb, (uint8_t *) rcb.rxcb + rcb.tail, (RX_RCB_SZ - rcb.tail) );
		// copy elements at the beginning of the second buffer
		memcpy((uint8_t *) rxb + (RX_RCB_SZ - rcb.tail), (uint8_t *) rcb.rxcb, rcb.head );

		// copy the elements at the beginning of the buffer subsequently

	}

	return !RX_RCB_EMPTY;
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_consumerTaskHook */
/**
* @brief Function implementing the consumerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_consumerTaskHook */
void consumerTaskHook(void const * argument)
{
	TickType_t TaskTimeStamp;
	TickType_t DelayTimeMsec = 10000;  // wait 2s then poll

	// We will assume the user gives a finite size buffer (3 characters in this example)
	uint8_t msg[RX_RB_SZ];
	for(;;)
	{
		// wait 2s then try to move data from kernel to user memory space
		osDelayUntil(&TaskTimeStamp, DelayTimeMsec);
		// non-blocking access to the buffer
		if( poll(msg, RX_RB_SZ) == RX_RCB_EMPTY )
			read_msgs++;  // successfully read messages
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
