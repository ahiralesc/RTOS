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
#include <math.h>
#include <stdio.h>
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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

osThreadId OdometryUpdateHandle;
osThreadId MotorControllerHandle;
osThreadId PIDUpdateHandle;
osThreadId ReadIRSensorsHandle;
/* USER CODE BEGIN PV */
//------IR Sensors-------//
volatile uint16_t adcResultsDMA[5];
const int adcChannelCount = sizeof (adcResultsDMA) / sizeof (adcResultsDMA[0]);
volatile int adcConversionComplete = 0;
char message[70] = {'\0'};
int lenMessage;
int CH1, CH2, CH3, CH4, CH5;
//------Motor PD---------//

//Initial Pose
double x = 0;
double y = 0;
float theta = 0;

#define mmPerTick 0.55   //Linear distance per tick of each encoder with the 63 mm wheel
#define L 152   //Distance wheel to wheel (mm)

//const float pi = 3.141592;
//New variables
int RTicks = 0;
int LTicks = 0;
int PrevRTicks = 0;
int PrevLTicks = 0;
int RdtTicks = 0;
int LdtTicks = 0;
float Rdist = 0;
float Ldist = 0;
float Cdist = 0;
float thetaDeg = 0;
float thetaDegError = 0;
//uint8_t message[50] = {'\0'};
//uint8_t message[50] = {'\0'};
int lenMessage;
//Goal variables

//const int gx = 0;
//const int gy = 300;

//PD variables and controller variables
#define Kp 3000
#define Kd 1500
#define errorToleranceReduce 60
#define errorToleranceStop 20
//#define errorToleranceStopRotation 30
#define dutyCycle 50000	//Dutycycle for velocity of the motors Resolution X/65535
#define dutyCycleReduced 30000	//Reduced dutycycle for especific error tolerance
#define dutyCycleRotation 30000

float e = 0;
float gtheta;
float eP = 0;
float eD = 0;
float last_e = 0;
int dutyCyclePD = 0;
float lastTime = 0;
float Time = 0;
int angleRotationControl = 15;

//PathFollower
int CntPathNode = 0;
#define pathSize 5
int XPoints[pathSize] = {1,1,2,2,3};
int YPoints[pathSize] = {0,1,1,0,0};

//-------Dijkstra---------//s

volatile int RTNextNode = 0;

int gx = 0;
int gy = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void StartOdometryUpdate(void const * argument);
void StartMotorController(void const * argument);
void StartPIDUpdate(void const * argument);
void StartReadIRSensors(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  //IR Sensor
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResultsDMA, 5);
  //
  gx = XPoints[CntPathNode]*180;
  gy = YPoints[CntPathNode]*180;

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
  /* definition and creation of OdometryUpdate */
  osThreadDef(OdometryUpdate, StartOdometryUpdate, osPriorityNormal, 0, 128);
  OdometryUpdateHandle = osThreadCreate(osThread(OdometryUpdate), NULL);

  /* definition and creation of MotorController */
  osThreadDef(MotorController, StartMotorController, osPriorityNormal, 0, 128);
  MotorControllerHandle = osThreadCreate(osThread(MotorController), NULL);

  /* definition and creation of PIDUpdate */
  osThreadDef(PIDUpdate, StartPIDUpdate, osPriorityNormal, 0, 128);
  PIDUpdateHandle = osThreadCreate(osThread(PIDUpdate), NULL);

  /* definition and creation of ReadIRSensors */
  osThreadDef(ReadIRSensors, StartReadIRSensors, osPriorityIdle, 0, 128);
  ReadIRSensorsHandle = osThreadCreate(osThread(ReadIRSensors), NULL);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RedLED_Pin|GreenLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RedLED_Pin GreenLED_Pin */
  GPIO_InitStruct.Pin = RedLED_Pin|GreenLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc){
	  CH1 = adcResultsDMA[0];
	  CH2 = adcResultsDMA[1];
	  CH3 = adcResultsDMA[2];
	  CH4 = adcResultsDMA[3];
	  CH5 = adcResultsDMA[4];
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartOdometryUpdate */
/**
  * @brief  Function implementing the OdometryUpdate thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartOdometryUpdate */
void StartOdometryUpdate(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  RTicks = ((TIM2->CNT)>>2);
	  	  LTicks =  ((TIM3->CNT)>>2);

	  	  //CNT registers are 2^14

	  	  if  (RTicks >= 16000 && PrevRTicks <= 1000) {
	  		  RTicks = -(16383 - RTicks);
	  	  }

	  	  if  (LTicks >= 16000 && PrevLTicks <= 1000) {
	  		  LTicks = -(16383 - LTicks);
	  	  }

	  	  if  (RTicks <= 1000 && PrevRTicks >= 16000) {
	  		  RTicks = RTicks + (16383 - PrevRTicks);
	  	  }

	  	  if  (LTicks <= 1000 && PrevLTicks >= 16000) {
	  		  LTicks = LTicks + (16383 - PrevLTicks);
	  	  }

	  	  RdtTicks = RTicks - PrevRTicks;
	  	  LdtTicks = LTicks - PrevLTicks;

	  	  Rdist = mmPerTick*RdtTicks;
	  	  Ldist = 2*mmPerTick*LdtTicks;  //Resolution of left encoder is 1/2 of the right encoder
	  	  Cdist = (Rdist + Ldist)/2;

	  	  x = x + Cdist*cos(theta);
	  	  y = y + Cdist*sin(theta);

	  	  theta = theta + ((Rdist - Ldist)/L);
	  	  theta = atan2(sin(theta), cos(theta));
	  	  thetaDeg = theta*(180/3.1416);

	  	  PrevRTicks = RTicks;
	  	  PrevLTicks = LTicks;

	  	  //Debugging via UART
	  	  int xs, ys, ts, Rs, Ls, Cs;
	  	  xs = x;
	  	  ys = y;
	  	  ts = thetaDeg;
	  	  Rs = Rdist;
	  	  Ls = Ldist;
	  	  Cs = Cdist;

	  	  lenMessage = sprintf(message, "x = %d | y = %d | theta = %d |  gx = %d | cnt = %d \n\r", xs, ys, ts, gx, CntPathNode);
	  	  HAL_UART_Transmit(&huart1, (uint8_t *)message, lenMessage, 15);
  //	  lenMessage = sprintf(message, "Rdist = %d | Ldist = %d | Cdist = %d \n\r", Rs, Ls, Cs);
  //	  HAL_UART_Transmit(&huart1, (uint8_t *)message, lenMessage, 15);
  //	  lenMessage = sprintf(message, "LeftEncoder= %d| RightEncoder= %d \n\r", ((TIM3->CNT)>>2), ((TIM2->CNT)>>2) );
  //	  HAL_UART_Transmit(&huart1, (uint8_t *)message, lenMessage, 15);
	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //Toggle the state of pin PC13
	  	  //------------------------------------------------------//

	  	  osDelay(15);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMotorController */
/**
* @brief Function implementing the MotorController thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorController */
void StartMotorController(void const * argument)
{
  /* USER CODE BEGIN StartMotorController */
  /* Infinite loop */
  for(;;)
  {
	  if ((thetaDegError >= angleRotationControl || thetaDegError <= -angleRotationControl) && (errorToleranceReduce <= sqrt(pow(gx-x, 2)+pow(gy-y, 2)))){
		  angleRotationControl = 10;
		  if (thetaDegError < 0) {
			  TIM4->CCR1 = dutyCycleRotation;
			  TIM4->CCR2 = 0;
			  TIM4->CCR3 = dutyCycleRotation;
			  TIM4->CCR4 = 0;
		  }
		  else {
			  TIM4->CCR1 = 0;
			  TIM4->CCR2 = dutyCycleRotation;
			  TIM4->CCR3 = 0;
			  TIM4->CCR4 = dutyCycleRotation;
		  }
	  }

	  else {
		  angleRotationControl = 15;
		  RTNextNode = 1; //Rotation Flag to avoid dealing with the angle controller ^^ problem while following a straight Coord/Line
		  if (errorToleranceReduce <= sqrt(pow(gx-x, 2)+pow(gy-y, 2))){		  //Error tolerance within the euclidean distance of the robot to goal
			  TIM4->CCR1 = 0;
			  TIM4->CCR2 = dutyCycle-dutyCyclePD;
			  TIM4->CCR3 = dutyCycle+dutyCyclePD;
			  TIM4->CCR4 = 0;
		  }
		  else if (errorToleranceStop <= sqrt(pow(gx-x, 2)+pow(gy-y, 2))){
			  TIM4->CCR1 = 0;
			  TIM4->CCR2 = dutyCycleReduced; //++
			  TIM4->CCR3 = dutyCycleReduced;
			  TIM4->CCR4 = 0;
		  }
		  else {
			  TIM4->CCR1 = 0;
			  TIM4->CCR2 = 0;
			  TIM4->CCR3 = 0;
			  TIM4->CCR4 = 0;

			  RTNextNode = 0; //Rotation Flag Off to avoid dealing with the angle controller ^^ problem while following a straight Coord/Line

			  HAL_Delay(1000);

			  if (pathSize-1 > CntPathNode){
				  CntPathNode++;
				  gx = XPoints[CntPathNode]*180;
				  gy = YPoints[CntPathNode]*180;
//
//				  gtheta = atan2((gy-y), (gx-x));
//				  e = gtheta - theta;
//				  e = atan2(sin(e), cos(e));
//				  thetaDegError = (gtheta-theta)*(180/3.1416);
//
//				  if (thetaDegError >= 45 || thetaDegError <= -45) {
//					  RTNextNode = 1;
//				  }
			  }


		  }
	  }
	  osDelay(15);
  }

  /* USER CODE END StartMotorController */
}

/* USER CODE BEGIN Header_StartPIDUpdate */
/**
* @brief Function implementing the PIDUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPIDUpdate */
void StartPIDUpdate(void const * argument)
{
  /* USER CODE BEGIN StartPIDUpdate */
  /* Infinite loop */
  for(;;)
  {
	  gtheta = atan2((gy-y), (gx-x));
	  e = gtheta - theta;
	  e = atan2(sin(e), cos(e));
	  eP = e*Kp;
	  Time = HAL_GetTick();
	  eD = ((e-last_e)/(Time - lastTime))*Kd;
	  last_e = e;
	  lastTime = Time;
	  dutyCyclePD = eP + eD;

	  thetaDegError = (gtheta-theta)*(180/3.1416);

	  //Debugging via UART
	  int ePs, eDs, gthetaDeg, thes;
	  ePs = eP;
	  eDs = eD;
	  gthetaDeg = gtheta*(180/3.1416);
	  thes = thetaDegError;


	  lenMessage = sprintf(message, "eP = %d | eD = %d | gtheta = %d | thetaError = %d \n\r", ePs, eDs, gthetaDeg, thes);
	  HAL_UART_Transmit(&huart1, (uint8_t *)message, lenMessage, 15);
	  //------------------------------------------------------//
	  osDelay(25);
  }
  /* USER CODE END StartPIDUpdate */
}

/* USER CODE BEGIN Header_StartReadIRSensors */
/**
* @brief Function implementing the ReadIRSensors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadIRSensors */
void StartReadIRSensors(void const * argument)
{
  /* USER CODE BEGIN StartReadIRSensors */
  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResultsDMA, 5);
	  lenMessage = sprintf(message, "CH1 = %d | CH2 = %d | CH3 = %d | CH4 = %d | CH5 = %d \n\r", CH1, CH2, CH3, CH4, CH5);
	  HAL_UART_Transmit(&huart1, (uint8_t *)message, lenMessage, 15);
	  osDelay(100);
  }
  /* USER CODE END StartReadIRSensors */
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
