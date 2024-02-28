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
#include <stdlib.h>
#include <time.h>
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

osThreadId defaultTaskHandle;
osThreadId adjust_flowHandle;
osThreadId light_stateHandle;
osThreadId sys_manageHandle;
osMessageQId traffic_queue_1Handle;
osMessageQId traffic_queue_2Handle;
osMessageQId cars_array_queueHandle;
osMessageQId light_status_queueHandle;
osMessageQId car_timer_queueHandle;
osMessageQId pot_timer_queueHandle;
osMessageQId light_timer_queueHandle;
osTimerId car_movement_timerHandle;
osMutexId cars_array_mutexHandle;
osMutexId traffic_rate_2_mutexHandle;
osMutexId light_status_mutexHandle;
osMutexId traffic_rate_1_mutexHandle;
osMutexId car_timing_mutexHandle;
osMutexId pot_timer_mutexHandle;
osMutexId light_timer_mutexHandle;
/* USER CODE BEGIN PV */
int light_status;
int cars[16];
int traffic_rate;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);
void AdjustFlow(void const * argument);
void LightState(void const * argument);
void SysManage(void const * argument);
void car_movement_callback(void const * argument);

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
	srand(314159);
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of cars_array_mutex */
  osMutexDef(cars_array_mutex);
  cars_array_mutexHandle = osMutexCreate(osMutex(cars_array_mutex));

  /* definition and creation of traffic_rate_2_mutex */
  osMutexDef(traffic_rate_2_mutex);
  traffic_rate_2_mutexHandle = osMutexCreate(osMutex(traffic_rate_2_mutex));

  /* definition and creation of light_status_mutex */
  osMutexDef(light_status_mutex);
  light_status_mutexHandle = osMutexCreate(osMutex(light_status_mutex));

  /* definition and creation of traffic_rate_1_mutex */
  osMutexDef(traffic_rate_1_mutex);
  traffic_rate_1_mutexHandle = osMutexCreate(osMutex(traffic_rate_1_mutex));

  /* definition and creation of car_timing_mutex */
  osMutexDef(car_timing_mutex);
  car_timing_mutexHandle = osMutexCreate(osMutex(car_timing_mutex));

  /* definition and creation of pot_timer_mutex */
  osMutexDef(pot_timer_mutex);
  pot_timer_mutexHandle = osMutexCreate(osMutex(pot_timer_mutex));

  /* definition and creation of light_timer_mutex */
  osMutexDef(light_timer_mutex);
  light_timer_mutexHandle = osMutexCreate(osMutex(light_timer_mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of car_movement_timer */
  osTimerDef(car_movement_timer, car_movement_callback);
  car_movement_timerHandle = osTimerCreate(osTimer(car_movement_timer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of traffic_queue_1 */
  osMessageQDef(traffic_queue_1, 16, uint16_t);
  traffic_queue_1Handle = osMessageCreate(osMessageQ(traffic_queue_1), NULL);

  /* definition and creation of traffic_queue_2 */
  osMessageQDef(traffic_queue_2, 16, uint16_t);
  traffic_queue_2Handle = osMessageCreate(osMessageQ(traffic_queue_2), NULL);

  /* definition and creation of cars_array_queue */
  osMessageQDef(cars_array_queue, 16, uint32_t);
  cars_array_queueHandle = osMessageCreate(osMessageQ(cars_array_queue), NULL);

  /* definition and creation of light_status_queue */
  osMessageQDef(light_status_queue, 16, uint16_t);
  light_status_queueHandle = osMessageCreate(osMessageQ(light_status_queue), NULL);

  /* definition and creation of car_timer_queue */
  osMessageQDef(car_timer_queue, 16, uint16_t);
  car_timer_queueHandle = osMessageCreate(osMessageQ(car_timer_queue), NULL);

  /* definition and creation of pot_timer_queue */
  osMessageQDef(pot_timer_queue, 16, uint16_t);
  pot_timer_queueHandle = osMessageCreate(osMessageQ(pot_timer_queue), NULL);

  /* definition and creation of light_timer_queue */
  osMessageQDef(light_timer_queue, 16, uint16_t);
  light_timer_queueHandle = osMessageCreate(osMessageQ(light_timer_queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of adjust_flow */
  osThreadDef(adjust_flow, AdjustFlow, osPriorityIdle, 0, 128);
  adjust_flowHandle = osThreadCreate(osThread(adjust_flow), NULL);

  /* definition and creation of light_state */
  osThreadDef(light_state, LightState, osPriorityIdle, 0, 128);
  light_stateHandle = osThreadCreate(osThread(light_state), NULL);

  /* definition and creation of sys_manage */
  osThreadDef(sys_manage, SysManage, osPriorityIdle, 0, 128);
  sys_manageHandle = osThreadCreate(osThread(sys_manage), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osTimerStart(car_movement_timerHandle, 500);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sConfig.Channel = ADC_CHANNEL_13;
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
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Red_Light_Pin|Amber_Light_Pin|Green_Light_Pin|Shift_Reg_Data_Pin
                          |Shift_Reg_Clock_Pin|Shift_Reg_Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Red_Light_Pin Amber_Light_Pin Green_Light_Pin Shift_Reg_Data_Pin
                           Shift_Reg_Clock_Pin Shift_Reg_Reset_Pin */
  GPIO_InitStruct.Pin = Red_Light_Pin|Amber_Light_Pin|Green_Light_Pin|Shift_Reg_Data_Pin
                          |Shift_Reg_Clock_Pin|Shift_Reg_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

/* USER CODE BEGIN Header_AdjustFlow */
/**
 * @brief Function implementing the adjust_flow thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_AdjustFlow */
void AdjustFlow(void const * argument)
{
  /* USER CODE BEGIN AdjustFlow */
	uint16_t raw = 0;
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(pot_timer_mutexHandle, osWaitForever);
		osEvent event = osMessageGet(pot_timer_queueHandle, 0);
		osMutexRelease(pot_timer_mutexHandle);
		if(event.status != osEventMessage){
			continue;
		}
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		raw = HAL_ADC_GetValue(&hadc1);
		if(raw > 2400){
			raw = 2400;
		}
		osMutexWait(traffic_rate_1_mutexHandle, osWaitForever);
		osMessageGet(traffic_queue_1Handle, 0);
		osMessagePut(traffic_queue_1Handle, raw, osWaitForever);
		osMutexRelease(traffic_rate_1_mutexHandle);

		osMutexWait(traffic_rate_2_mutexHandle, osWaitForever);
		osMessageGet(traffic_queue_2Handle, 0);
		osMessagePut(traffic_queue_2Handle, raw, osWaitForever);
		osMutexRelease(traffic_rate_2_mutexHandle);
		// osDelay(1);
	}

  /* USER CODE END AdjustFlow */
}

/* USER CODE BEGIN Header_LightState */
/**
 * @brief Function implementing the light_state thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LightState */
void LightState(void const * argument)
{
  /* USER CODE BEGIN LightState */
	uint16_t rate = 0;
	float scaled = 0;
	int scale_cap = 1;
	int scale_count = 0;
	/* Infinite loop */
	for(;;)
	{
		osMutexWait(traffic_rate_2_mutexHandle, osWaitForever);
		// int rate = traffic_rate; // TODO: traffic_queue_1
		osEvent event = osMessageGet(traffic_queue_2Handle, 0);
		if(event.status == osEventMessage){
			rate = event.value.v;
		}
		osMutexRelease(traffic_rate_2_mutexHandle);
		scaled = rate / 2400.0;
		// turn green LED on
		HAL_GPIO_WritePin(GPIOC, Red_Light_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Amber_Light_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Green_Light_Pin, GPIO_PIN_SET);

		osMutexWait(light_status_mutexHandle, osWaitForever);
		osMessagePut(light_status_queueHandle, 2, osWaitForever);
		osMutexRelease(light_status_mutexHandle);
		// light_status = 2; //TODO: light_queue_0
		// modulate traffic rate to 1
		// osDelay((uint32_t)(3000 + 3000 * scaled));
		scale_cap = (int)(3000 + 3000 * scaled)/500;
		osMutexWait(light_timer_mutexHandle, osWaitForever);
		event = osMessageGet(light_timer_queueHandle, 0);
		osMutexRelease(light_timer_mutexHandle);
		while(scale_count < scale_cap){
			if (event.status == osEventMessage){
				scale_count ++;
			}
			osMutexWait(light_timer_mutexHandle, osWaitForever);
			event = osMessageGet(light_timer_queueHandle, 0);
			osMutexRelease(light_timer_mutexHandle);
		}
		scale_count = 0;

		// turn yellow LED on
		HAL_GPIO_WritePin(GPIOC, Red_Light_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Amber_Light_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, Green_Light_Pin, GPIO_PIN_RESET);

		osMutexWait(light_status_mutexHandle, osWaitForever);
		osMessagePut(light_status_queueHandle, 1, osWaitForever);
		osMutexRelease(light_status_mutexHandle);
		// light_status = 1;
		scale_cap = 2;
		osMutexWait(light_timer_mutexHandle, osWaitForever);
		event = osMessageGet(light_timer_queueHandle, 0);
		osMutexRelease(light_timer_mutexHandle);
		while(scale_count < scale_cap){
			if (event.status == osEventMessage){
				scale_count ++;
			}
			osMutexWait(light_timer_mutexHandle, osWaitForever);
			event = osMessageGet(light_timer_queueHandle, 0);
			osMutexRelease(light_timer_mutexHandle);
		}
		scale_count = 0;

		osMutexWait(traffic_rate_2_mutexHandle, osWaitForever);
		event = osMessageGet(traffic_queue_2Handle, 0); //TODO: traffic_queue_1
		if(event.status == osEventMessage){
			rate = event.value.v;
		}
		osMutexRelease(traffic_rate_2_mutexHandle);
		scaled = rate / 2400.0;
		// turn red LED on
		HAL_GPIO_WritePin(GPIOC, Red_Light_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, Amber_Light_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, Green_Light_Pin, GPIO_PIN_RESET);

		osMutexWait(light_status_mutexHandle, osWaitForever);
		osMessagePut(light_status_queueHandle, 0, osWaitForever);
		osMutexRelease(light_status_mutexHandle);
		// light_status = 0;
		// modulate traffic rate to 1
		// osDelay((uint32_t)(3000 + 3000 * (1-scaled)));
		scale_cap = (int)(3000 + 3000 * (1 - scaled))/500;
		osMutexWait(light_timer_mutexHandle, osWaitForever);
		event = osMessageGet(light_timer_queueHandle, 0);
		osMutexRelease(light_timer_mutexHandle);
		while(scale_count < scale_cap){
			if (event.status == osEventMessage){
				scale_count ++;
			}
			osMutexWait(light_timer_mutexHandle, osWaitForever);
			event = osMessageGet(light_timer_queueHandle, 0);
			osMutexRelease(light_timer_mutexHandle);
		}
		scale_count = 0;
	}
  /* USER CODE END LightState */
}

/* USER CODE BEGIN Header_SysManage */
/**
 * @brief Function implementing the sys_manage thread.
 * @param argument: Not used
 * @retval None
 */

int32_t convert_to_integer(int* traffic_array){
	int i;
	int32_t converted_number;
	for (i=0; i<19; i++){
		converted_number |= traffic_array[i] << (18-i);
	}
	return converted_number;
}

int trafficGenerated(){
	uint16_t traffic = 0;
	float scaled = 0;
	osMutexWait(traffic_rate_1_mutexHandle, osWaitForever);
	//int traffic = traffic_rate; //TODO: traffic_queue_0
	osEvent event = osMessageGet(traffic_queue_1Handle, 0);
	if(event.status == osEventMessage){
		traffic = event.value.v;
	}
	osMutexRelease(traffic_rate_1_mutexHandle);
	scaled = traffic / 2400.0;
	// modulate traffic rate from 1 to 10

	float random = (float)(rand() % 10);
	if (random < scaled*10) {
		return 1;
	}
	return 0;
}

/* USER CODE END Header_SysManage */
void SysManage(void const * argument)
{
  /* USER CODE BEGIN SysManage */
	/* Infinite loop */
	int i;
	int cars[19] = {};
	int cars_padded[21] = {};
	int light_colour = 0;
	for(;;)
	{
		osMutexWait(car_timing_mutexHandle, osWaitForever);
		osEvent event = osMessageGet(car_timer_queueHandle, 0);
		osMutexRelease(car_timing_mutexHandle);
		if(event.status != osEventMessage){
			osDelay(1);
			continue;
		}

		osMutexWait(light_status_mutexHandle, osWaitForever);
		event = osMessageGet(light_status_queueHandle, 0); //TODO: light_queue_0
		if(event.status == osEventMessage){
			light_colour = event.value.v;
		}
		osMutexRelease(light_status_mutexHandle);

		// osMutexWait(cars_array_mutexHandle); //TODO: cars_queue_0
		for (i = 18; i>0; i--){
			if (light_colour == 2) { //green
				cars[i] = cars[i-1];
				cars[i - 1] = 0;
			}
			else if (light_colour == 1) { //yellow
				if (i > 8) {
					cars[i] = cars[i-1];
					cars[i-1] = 0;
				}
				else {
					if (!cars[i]){
						cars[i] = cars[i-1];
						cars[i-1] = 0;
					}
				}
			}
			else { //red
				if (i > 8){
					cars[i] = cars[i-1];
					cars[i-1] = 0;
				}
				else if (i < 8){
					if (!cars[i]) {
						cars[i] = cars[i-1];
						cars[i-1] = 0;
					}
				}
			}
//			else {
//				if (i > 8){
//					cars[i] = cars[i-1];
//				}
//				else {
//
//					if (!cars[i]) {
//						cars[i] = cars[i-1];
//						cars[i-1] = 0;
//					}
//				}
//			}
		}
		if (trafficGenerated()){
			cars[0] = 1;
		}
		else {
			cars[0] = 0;
		}
		//int32_t cars_int = convert_to_integer(cars);
		// osMutexRelease(cars_array_mutexHandle);
		//osMutexWait(cars_array_mutexHandle, osWaitForever);
		// int* mail = (int *)osMailAlloc(cars_array_queueHandle, osWaitForever);
		//osMailPut(cars_array_queueHandle, cars);
		//osMessagePut(cars_array_queueHandle, cars_int, osWaitForever);
		//osMutexRelease(cars_array_mutexHandle);

		// Pad the shift registers with extra data in the 8th bit for each.
		for(i = 0; i < 19; i++){
			if(i >= 14){
				cars_padded[i+2] = cars[i];
			} else if(i >= 7){
				cars_padded[i+1] = cars[i];
			} else {
				cars_padded[i] = cars[i];
			}
		}

		HAL_GPIO_WritePin(GPIOC, Shift_Reg_Reset_Pin, GPIO_PIN_RESET);
		osDelay(1);
		HAL_GPIO_WritePin(GPIOC, Shift_Reg_Reset_Pin, GPIO_PIN_SET);
		//osDelay(1);
		for(i =20; i >= 0; i--){

			if(cars_padded[i] != 0){
				HAL_GPIO_WritePin(GPIOC, Shift_Reg_Data_Pin, GPIO_PIN_SET);
			}
			//osDelay(1);
			HAL_GPIO_WritePin(GPIOC, Shift_Reg_Clock_Pin, GPIO_PIN_SET);
			//osDelay(1);
			HAL_GPIO_WritePin(GPIOC, Shift_Reg_Clock_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, Shift_Reg_Data_Pin, GPIO_PIN_RESET);
			//osDelay(1);
		}
		// osDelay(500);
	}
  /* USER CODE END SysManage */
}

/* car_movement_callback function */
void car_movement_callback(void const * argument)
{
  /* USER CODE BEGIN car_movement_callback */
	osMutexWait(car_timing_mutexHandle, osWaitForever);
	osMutexWait(pot_timer_mutexHandle, osWaitForever);
	osMutexWait(light_timer_mutexHandle, osWaitForever);

	osMessageGet(car_timer_queueHandle, 0);
	osMessagePut(car_timer_queueHandle, 1, osWaitForever);
	osMessagePut(pot_timer_queueHandle, 1, osWaitForever);
	osMessagePut(light_timer_queueHandle, 1, osWaitForever);

	osMutexRelease(light_timer_mutexHandle);
	osMutexRelease(pot_timer_mutexHandle);
	osMutexRelease(car_timing_mutexHandle);
  /* USER CODE END car_movement_callback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
