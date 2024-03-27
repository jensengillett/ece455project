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
#include <stdio.h>
#include <stdlib.h>
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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

HCD_HandleTypeDef hhcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
osThreadId deadline_drivenHandle;
osThreadId task_generatorHandle;
osThreadId monitorHandle;
osThreadId red_light_taskHandle;
osThreadId amber_light_tasHandle;
osThreadId green_light_tasHandle;
osMessageQId active_queueHandle;
osMessageQId completed_queueHandle;
osMessageQId dds_task_queueHandle;
osMessageQId overdue_queueHandle;
osMessageQId make_active_queueHandle;
osMessageQId make_completed_queueHandle;
osMessageQId make_overdue_queueHandle;
osMessageQId task_duration_queueHandle;
osMessageQId task_1_time_queueHandle;
osMessageQId task_2_time_queueHandle;
osMessageQId task_3_time_queueHandle;
osTimerId dds_control_timerHandle;
osMutexId active_queue_mutexHandle;
osMutexId completed_queue_mutexHandle;
osMutexId overdue_queue_mutexHandle;
osMutexId dds_task_queue_mutexHandle;
osMutexId make_active_queue_mutexHandle;
osMutexId make_completed_queue_mutexHandle;
osMutexId make_overdue_queue_mutexHandle;
osMutexId task_duration_queue_mutexHandle;
osMutexId task_1_time_queue_mutexHandle;
osMutexId task_2_time_queue_mutexHandle;
osMutexId task_3_time_queue_mutexHandle;
/* USER CODE BEGIN PV */
osTimerId task_1_timerHandle;
osTimerId task_2_timerHandle;
osTimerId task_3_timerHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_OTG_FS_HCD_Init(void);
void StartDefaultTask(void const * argument);
void DeadlineDrivenScheduler(void const * argument);
void TaskGenerator(void const * argument);
void Monitor(void const * argument);
void RedLightTask(void const * argument);
void AmberLightTask(void const * argument);
void GreenLightTask(void const * argument);
void dds_control_callback(void const * argument);

/* USER CODE BEGIN PFP */
void task_1_timer_callback(void const * argument);
void task_2_timer_callback(void const * argument);
void task_3_timer_callback(void const * argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum task_type {PERIODIC, APERIODIC} TASK_TYPE;

typedef struct dd_task {
     osThreadId t_handle;
     TASK_TYPE type;
     uint32_t task_id;
     uint32_t release_time;
     uint32_t absolute_deadline;
     uint32_t completion_time;
     uint32_t execution_time;
} DD_TASK;

typedef struct dd_task_list {
    DD_TASK task;
    struct dd_task_list* next;
} DD_TASK_LIST;

typedef struct time_struct{
	uint32_t period;
	uint32_t execution_time;
}time_struct;
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_HCD_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of active_queue_mutex */
  osMutexDef(active_queue_mutex);
  active_queue_mutexHandle = osMutexCreate(osMutex(active_queue_mutex));

  /* definition and creation of completed_queue_mutex */
  osMutexDef(completed_queue_mutex);
  completed_queue_mutexHandle = osMutexCreate(osMutex(completed_queue_mutex));

  /* definition and creation of overdue_queue_mutex */
  osMutexDef(overdue_queue_mutex);
  overdue_queue_mutexHandle = osMutexCreate(osMutex(overdue_queue_mutex));

  /* definition and creation of dds_task_queue_mutex */
  osMutexDef(dds_task_queue_mutex);
  dds_task_queue_mutexHandle = osMutexCreate(osMutex(dds_task_queue_mutex));

  /* definition and creation of make_active_queue_mutex */
  osMutexDef(make_active_queue_mutex);
  make_active_queue_mutexHandle = osMutexCreate(osMutex(make_active_queue_mutex));

  /* definition and creation of make_completed_queue_mutex */
  osMutexDef(make_completed_queue_mutex);
  make_completed_queue_mutexHandle = osMutexCreate(osMutex(make_completed_queue_mutex));

  /* definition and creation of make_overdue_queue_mutex */
  osMutexDef(make_overdue_queue_mutex);
  make_overdue_queue_mutexHandle = osMutexCreate(osMutex(make_overdue_queue_mutex));

  /* definition and creation of task_duration_queue_mutex */
  osMutexDef(task_duration_queue_mutex);
  task_duration_queue_mutexHandle = osMutexCreate(osMutex(task_duration_queue_mutex));

  /* definition and creation of task_1_time_queue_mutex */
  osMutexDef(task_1_time_queue_mutex);
  task_1_time_queue_mutexHandle = osMutexCreate(osMutex(task_1_time_queue_mutex));

  /* definition and creation of task_2_time_queue_mutex */
  osMutexDef(task_2_time_queue_mutex);
  task_2_time_queue_mutexHandle = osMutexCreate(osMutex(task_2_time_queue_mutex));

  /* definition and creation of task_3_time_queue_mutex */
  osMutexDef(task_3_time_queue_mutex);
  task_3_time_queue_mutexHandle = osMutexCreate(osMutex(task_3_time_queue_mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of dds_control_timer */
  osTimerDef(dds_control_timer, dds_control_callback);
  dds_control_timerHandle = osTimerCreate(osTimer(dds_control_timer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of active_queue */
  osMessageQDef(active_queue, 16, uint32_t);
  active_queueHandle = osMessageCreate(osMessageQ(active_queue), NULL);

  /* definition and creation of completed_queue */
  osMessageQDef(completed_queue, 16, uint32_t);
  completed_queueHandle = osMessageCreate(osMessageQ(completed_queue), NULL);

  /* definition and creation of dds_task_queue */
  osMessageQDef(dds_task_queue, 16, uint32_t);
  dds_task_queueHandle = osMessageCreate(osMessageQ(dds_task_queue), NULL);

  /* definition and creation of overdue_queue */
  osMessageQDef(overdue_queue, 16, uint32_t);
  overdue_queueHandle = osMessageCreate(osMessageQ(overdue_queue), NULL);

  /* definition and creation of make_active_queue */
  osMessageQDef(make_active_queue, 16, uint32_t);
  make_active_queueHandle = osMessageCreate(osMessageQ(make_active_queue), NULL);

  /* definition and creation of make_completed_queue */
  osMessageQDef(make_completed_queue, 16, uint32_t);
  make_completed_queueHandle = osMessageCreate(osMessageQ(make_completed_queue), NULL);

  /* definition and creation of make_overdue_queue */
  osMessageQDef(make_overdue_queue, 16, uint32_t);
  make_overdue_queueHandle = osMessageCreate(osMessageQ(make_overdue_queue), NULL);

  /* definition and creation of task_duration_queue */
  osMessageQDef(task_duration_queue, 16, uint32_t);
  task_duration_queueHandle = osMessageCreate(osMessageQ(task_duration_queue), NULL);

  /* definition and creation of task_1_time_queue */
  osMessageQDef(task_1_time_queue, 16, uint32_t);
  task_1_time_queueHandle = osMessageCreate(osMessageQ(task_1_time_queue), NULL);

  /* definition and creation of task_2_time_queue */
  osMessageQDef(task_2_time_queue, 16, uint32_t);
  task_2_time_queueHandle = osMessageCreate(osMessageQ(task_2_time_queue), NULL);

  /* definition and creation of task_3_time_queue */
  osMessageQDef(task_3_time_queue, 16, uint32_t);
  task_3_time_queueHandle = osMessageCreate(osMessageQ(task_3_time_queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of deadline_driven */
  osThreadDef(deadline_driven, DeadlineDrivenScheduler, osPriorityNormal, 0, 128);
  deadline_drivenHandle = osThreadCreate(osThread(deadline_driven), NULL);

  /* definition and creation of task_generator */
  osThreadDef(task_generator, TaskGenerator, osPriorityAboveNormal, 0, 128);
  task_generatorHandle = osThreadCreate(osThread(task_generator), NULL);

  /* definition and creation of monitor */
  osThreadDef(monitor, Monitor, osPriorityNormal, 0, 128);
  monitorHandle = osThreadCreate(osThread(monitor), NULL);

  /* definition and creation of red_light_task */
  osThreadDef(red_light_task, RedLightTask, osPriorityHigh, 0, 128);
  red_light_taskHandle = osThreadCreate(osThread(red_light_task), NULL);

  /* definition and creation of amber_light_tas */
  osThreadDef(amber_light_tas, AmberLightTask, osPriorityHigh, 0, 128);
  amber_light_tasHandle = osThreadCreate(osThread(amber_light_tas), NULL);

  /* definition and creation of green_light_tas */
  osThreadDef(green_light_tas, GreenLightTask, osPriorityHigh, 0, 128);
  green_light_tasHandle = osThreadCreate(osThread(green_light_tas), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  // We don't actually want the light tasks to run immediately, so end the threads.
  //osThreadTerminate(red_light_taskHandle);
  //osThreadTerminate(amber_light_tasHandle);
  //osThreadTerminate(green_light_tasHandle);

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_HCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hhcd_USB_OTG_FS.Init.Host_channels = 8;
  hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
  hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

DD_TASK_LIST* get_active_dd_task_list(){
	osMutexWait(dds_task_queue_mutexHandle, osWaitForever);
	osMessagePut(dds_task_queueHandle, 0, osWaitForever);
	osMutexRelease(dds_task_queue_mutexHandle);

	osThreadSetPriority(deadline_drivenHandle, osPriorityRealtime);
	osThreadYield();

	osMutexWait(active_queue_mutexHandle, osWaitForever);
	osEvent event = osMessageGet(active_queueHandle, 0);
	osMutexRelease(active_queue_mutexHandle);
	while(event.status != osEventMessage){
		osMutexWait(active_queue_mutexHandle, osWaitForever);
		event = osMessageGet(active_queueHandle, 0);
		osMutexRelease(active_queue_mutexHandle);
	}

	//TODO: this might be a hack, fix if it causes problems
	DD_TASK_LIST* active = (DD_TASK_LIST*)event.value.v;
	return active;
}

DD_TASK_LIST* get_completed_dd_task_list(){
	osMutexWait(dds_task_queue_mutexHandle, osWaitForever);
	osMessagePut(dds_task_queueHandle, 1, osWaitForever);
	osMutexRelease(dds_task_queue_mutexHandle);

	osThreadSetPriority(deadline_drivenHandle, osPriorityRealtime);
	osThreadYield();

	osMutexWait(completed_queue_mutexHandle, osWaitForever);
	osEvent event = osMessageGet(completed_queueHandle, 0);
	osMutexRelease(completed_queue_mutexHandle);
	while(event.status != osEventMessage){
		osMutexWait(completed_queue_mutexHandle, osWaitForever);
		event = osMessageGet(completed_queueHandle, 0);
		osMutexRelease(completed_queue_mutexHandle);
	}

	//TODO: this might be a hack, fix if it causes problems
	DD_TASK_LIST* completed = (DD_TASK_LIST*)event.value.v;
	return completed;
}

DD_TASK_LIST* get_overdue_dd_task_list(){
	osMutexWait(dds_task_queue_mutexHandle, osWaitForever);
	osMessagePut(dds_task_queueHandle, 2, osWaitForever);
	osMutexRelease(dds_task_queue_mutexHandle);

	osThreadSetPriority(deadline_drivenHandle, osPriorityRealtime);
	osThreadYield();


	osMutexWait(overdue_queue_mutexHandle, osWaitForever);
	osEvent event = osMessageGet(overdue_queueHandle, 0);
	osMutexRelease(overdue_queue_mutexHandle);
	while(event.status != osEventMessage){
		osMutexWait(overdue_queue_mutexHandle, osWaitForever);
		event = osMessageGet(overdue_queueHandle, 0);
		osMutexRelease(overdue_queue_mutexHandle);
		osThreadYield();
	}

	//TODO: this might be a hack, fix if it causes problems
	DD_TASK_LIST* overdue = (DD_TASK_LIST*)event.value.v;
	return overdue;
}

void release_dd_task(osThreadId t_handle, TASK_TYPE type, uint32_t task_id,uint32_t absolute_deadline, uint32_t execution_time){
	// malloc here
	uint32_t clock_time = osKernelSysTick();
	DD_TASK_LIST* task = (DD_TASK_LIST*) malloc(sizeof(DD_TASK_LIST));
	task->task.t_handle = t_handle;
	task->task.type = type;
	task->task.task_id = task_id;
	task->task.release_time = 0;
	task->task.absolute_deadline = absolute_deadline;
	task->task.completion_time = 0;
	task->task.execution_time = execution_time;
	task->next = NULL;

	//vTaskPrioritySet(t_handle, osPriorityLow);

	osMutexWait(dds_task_queue_mutexHandle, osWaitForever);
	osMutexWait(make_active_queue_mutexHandle, osWaitForever);
	osMessagePut(dds_task_queueHandle, 3, osWaitForever);
	uint32_t pointer = (uint32_t) task;
	osMessagePut(make_active_queueHandle, pointer, osWaitForever);
	osMutexRelease(make_active_queue_mutexHandle);
	osMutexRelease(dds_task_queue_mutexHandle);
}

void complete_dd_task(osThreadId task_handle){
	DD_TASK_LIST* searching_task = get_active_dd_task_list();
	DD_TASK_LIST* found_task = NULL;
	DD_TASK_LIST* overdue_tasks = NULL;
	uint32_t clock_time = osKernelSysTick(); // 1kHz
	while (searching_task != NULL){
		if (searching_task->task.absolute_deadline < clock_time){
			if (overdue_tasks == NULL){
				overdue_tasks = searching_task;
				searching_task = searching_task->next;
				overdue_tasks->next = NULL;
			}
			else {
				DD_TASK_LIST* overdue_last_task = overdue_tasks;
				while(overdue_last_task->next != NULL){
					continue;
				}
				overdue_last_task = searching_task;
				searching_task = searching_task->next;
				overdue_tasks->next = NULL;
			}
		}
		else {
			if (searching_task->task.t_handle == task_handle && found_task == NULL){
				found_task = searching_task;
			}
			searching_task = searching_task->next;
		}
	}

	osMutexWait(dds_task_queue_mutexHandle, osWaitForever);
	osMutexWait(make_completed_queue_mutexHandle, osWaitForever);
	osMutexWait(make_overdue_queue_mutexHandle, osWaitForever);
	osMessagePut(dds_task_queueHandle, 3, osWaitForever);
	osMessagePut(make_completed_queueHandle, (int)found_task, osWaitForever);
	osMessagePut(make_overdue_queueHandle, (int)overdue_tasks, osWaitForever);
	osMutexRelease(make_overdue_queue_mutexHandle);
	osMutexRelease(make_completed_queue_mutexHandle);
	osMutexRelease(dds_task_queue_mutexHandle);
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
  // die
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_DeadlineDrivenScheduler */
/**
* @brief Function implementing the deadline_driven thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DeadlineDrivenScheduler */
void DeadlineDrivenScheduler(void const * argument)
{
  /* USER CODE BEGIN DeadlineDrivenScheduler */
	int running = 0;
	osEvent event;
	DD_TASK_LIST* active_tasks = NULL;
	DD_TASK_LIST* completed_tasks = NULL;
	DD_TASK_LIST* overdue_tasks = NULL;
	DD_TASK_LIST* counter;
	DD_TASK_LIST* new_task;
	/* Infinite loop */
	for(;;)
	{
	  // Release DD Task
		osMutexWait(make_active_queue_mutexHandle, osWaitForever);
		event = osMessageGet(make_active_queueHandle, 0);
		osMutexRelease(make_active_queue_mutexHandle);

		if(event.status == osEventMessage){
			if (active_tasks == NULL){
				active_tasks = (DD_TASK_LIST*) event.value.v;
			}
			else if (active_tasks->next == NULL){
				new_task = (DD_TASK_LIST*) event.value.v;
				active_tasks->next = new_task;
				new_task->next = NULL;
			}
			else {
				new_task = (DD_TASK_LIST*) event.value.v;
				counter = active_tasks;
				while(counter->next != NULL && counter->task.absolute_deadline <= new_task->task.absolute_deadline){
					counter = counter->next;
				}
				new_task->next = counter->next;
				counter->next = new_task;
			}
			if (running == 0){
				running = 1;
				osMutexWait(task_duration_queue_mutexHandle, osWaitForever);
				osMessagePut(task_duration_queueHandle, active_tasks->task.execution_time, osWaitForever);
				osMutexRelease(task_duration_queue_mutexHandle);
				// set task priority to high
				osThreadSetPriority(active_tasks->task.t_handle, osPriorityNormal);
			}
		}
	  // Complete DD Task
		osMutexWait(make_completed_queue_mutexHandle, osWaitForever);
		event = osMessageGet(make_completed_queueHandle, 0);
		osMutexRelease(make_completed_queue_mutexHandle);

		if(event.status == osEventMessage){
			if (event.value.v != 0){
				if (completed_tasks == NULL){
					completed_tasks = (DD_TASK_LIST*) event.value.v;
				}
				else {
					new_task = (DD_TASK_LIST*) event.value.v;
					counter = completed_tasks;
					while(counter->next != NULL){
						counter = counter->next;
					}
					counter->next = new_task;
				}
			}

//			DD_TASK_LIST* check_task = active_tasks;
			DD_TASK_LIST* prev = NULL;
//			while (check_task != (DD_TASK_LIST*)event.value.v && check_task != NULL){
//				prev = check_task;
//				check_task = check_task->next;
//			}
//			if (check_task != NULL){
//				prev->next = check_task->next;
//				check_task->next = NULL;
//			}
			prev = active_tasks;
			active_tasks = active_tasks->next;
			prev->next = NULL;

			osMutexWait(task_duration_queue_mutexHandle, osWaitForever);
			osMessagePut(task_duration_queueHandle, new_task->task.execution_time, osWaitForever);
			osMutexRelease(task_duration_queue_mutexHandle);
			// set task priority to high
			if(active_tasks->task.t_handle != NULL && active_tasks != NULL){
				osThreadSetPriority(active_tasks->task.t_handle, osPriorityNormal);
			}
			else {
				running = 0;
			}
			// put overdue tasks away

			osMutexWait(make_overdue_queue_mutexHandle, osWaitForever);
			event = osMessageGet(make_overdue_queueHandle, 0);
			osMutexRelease(make_overdue_queue_mutexHandle);

			if (event.status == osEventMessage){
				if (overdue_tasks == NULL){
					overdue_tasks = (DD_TASK_LIST*) event.value.v;
				}
				else {
					new_task = (DD_TASK_LIST*) event.value.v;
					counter = overdue_tasks;
					while(counter->next != NULL){
						counter = counter->next;
					}
					counter->next = new_task;
				}
			}
		}
	  // Get Lists
		osMutexWait(dds_task_queue_mutexHandle, osWaitForever);
		event = osMessageGet(dds_task_queueHandle, 0);
		osMutexRelease(dds_task_queue_mutexHandle);

		if(event.status == osEventMessage){
			if (event.value.v == 0){
				osMutexWait(active_queue_mutexHandle, osWaitForever);
				osMessagePut(active_queueHandle, (int)active_tasks, osWaitForever);
				osMutexRelease(active_queue_mutexHandle);
			}
			else if (event.value.v == 1){
				osMutexWait(completed_queue_mutexHandle, osWaitForever);
				osMessagePut(completed_queueHandle, (int)completed_tasks, osWaitForever);
				osMutexRelease(completed_queue_mutexHandle);
			}
			else if (event.value.v == 2){
				osMutexWait(overdue_queue_mutexHandle, osWaitForever);
				osMessagePut(overdue_queueHandle, (int)overdue_tasks, osWaitForever);
				osMutexRelease(overdue_queue_mutexHandle);
			}
			osThreadSetPriority(deadline_drivenHandle, osPriorityNormal);
		}
		osThreadYield();
  	}
  /* USER CODE END DeadlineDrivenScheduler */
}

/* USER CODE BEGIN Header_TaskGenerator */
/**
* @brief Function implementing the task_generator thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskGenerator */
void TaskGenerator(void const * argument)
{
  /* USER CODE BEGIN TaskGenerator */
	uint8_t active_testbench = 0;  // 0-indexed
	uint32_t testbenches[3][3][2] = {
			{{ 95,500}, {150,500}, {250,750}},
			{{ 95,250}, {150,500}, {250,750}},
			{{100,500}, {200,500}, {200,500}}
	};

	uint32_t task_1_execution_time = 	testbenches[active_testbench][0][0];
	uint32_t task_1_period = 			testbenches[active_testbench][0][1];
	uint32_t task_2_execution_time = 	testbenches[active_testbench][1][0];
	uint32_t task_2_period = 			testbenches[active_testbench][1][1];
	uint32_t task_3_execution_time = 	testbenches[active_testbench][2][0];
	uint32_t task_3_period = 			testbenches[active_testbench][2][1];

	/* definition and creation of task_1_timer */
	osTimerDef(task_1_timer, task_1_timer_callback);
	time_struct* task_1_time = (time_struct*) malloc(sizeof(time_struct));
	task_1_time -> period = task_1_period;
	task_1_time -> execution_time = task_1_execution_time;
	osMutexWait(task_1_time_queue_mutexHandle, osWaitForever);
	osMessagePut(task_1_time_queueHandle, (int)task_1_time, osWaitForever);
	osMutexRelease(task_1_time_queue_mutexHandle);
	task_1_timerHandle = osTimerCreate(osTimer(task_1_timer), osTimerPeriodic, (void*)task_1_time);
	task_1_timer_callback(NULL);

	/* definition and creation of task_2_timer */
	osTimerDef(task_2_timer, task_2_timer_callback);
	time_struct* task_2_time = (time_struct*) malloc(sizeof(time_struct));
	task_2_time -> period = task_2_period;
	task_2_time -> execution_time = task_2_execution_time;
	osMutexWait(task_2_time_queue_mutexHandle, osWaitForever);
	osMessagePut(task_2_time_queueHandle, (int)task_2_time, osWaitForever);
	osMutexRelease(task_2_time_queue_mutexHandle);
	task_2_timerHandle = osTimerCreate(osTimer(task_2_timer), osTimerPeriodic, (void*)task_2_time);
	task_2_timer_callback(NULL);

	/* definition and creation of task_3_timer */
	osTimerDef(task_3_timer, task_3_timer_callback);
	time_struct* task_3_time = (time_struct*) malloc(sizeof(time_struct));
	task_3_time -> period = task_3_period;
	task_3_time -> execution_time = task_3_execution_time;
	osMutexWait(task_3_time_queue_mutexHandle, osWaitForever);
	osMessagePut(task_3_time_queueHandle, (int)task_3_time, osWaitForever);
	osMutexRelease(task_3_time_queue_mutexHandle);
	task_3_timerHandle = osTimerCreate(osTimer(task_3_timer), osTimerPeriodic, (void*)task_3_time);
	task_3_timer_callback(NULL);

	osStatus status = osTimerStart(task_1_timerHandle, task_1_period);
	status = osTimerStart(task_2_timerHandle, task_2_period);
	status = osTimerStart(task_3_timerHandle, task_3_period);

	osThreadTerminate(task_generatorHandle);
  /* USER CODE END TaskGenerator */
}

/* USER CODE BEGIN Header_Monitor */
/**
* @brief Function implementing the monitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Monitor */
void Monitor(void const * argument)
{
  /* USER CODE BEGIN Monitor */
	/* Infinite loop */
	for(;;){
		osDelay(1500);  // every hyper period

		printf("Current timestamp: %ldms\n", osKernelSysTick());

		DD_TASK_LIST* active_task_list = get_active_dd_task_list();
		int number_active_tasks = 0;
		if(active_task_list != NULL){
			number_active_tasks++;
			while(active_task_list->next != NULL){
				number_active_tasks++;
				active_task_list = active_task_list->next;
			}
		}

		printf("Number of active tasks: %d\n", number_active_tasks);

		DD_TASK_LIST* completed_task_list = get_completed_dd_task_list();
		int number_completed_tasks = 0;
		if(completed_task_list != NULL){
			number_completed_tasks++;
			while(completed_task_list->next != NULL){
				number_completed_tasks++;
				completed_task_list = completed_task_list->next;
			}
		}

		printf("Number of completed tasks: %d\n", number_completed_tasks);

		DD_TASK_LIST* overdue_task_list = get_overdue_dd_task_list();
		int number_overdue_tasks = 0;
		if(overdue_task_list != NULL){
			number_overdue_tasks++;
			while(overdue_task_list->next != NULL){
				number_overdue_tasks++;
				overdue_task_list = overdue_task_list->next;
			}
		}

		printf("Number of overdue tasks: %d\n", number_overdue_tasks);

		osThreadYield();
	}
  /* USER CODE END Monitor */
}

/* USER CODE BEGIN Header_RedLightTask */

int _write(int le, char *ptr, int len){
	int DataIdx;
	for(DataIdx = 0; DataIdx < len; DataIdx++){
		ITM_SendChar(*ptr++);
	}
	return len;
}

/* task_1_timer_callback function */
void task_1_timer_callback(void const * argument)
{
	/* USER CODE BEGIN task_1_timer_callback */
	static uint32_t count = 0;
	static time_struct* task_1_time = NULL;
	count++;
	if(task_1_time == NULL){
		osMutexWait(task_1_time_queue_mutexHandle, osWaitForever);
		osEvent event = osMessageGet(task_1_time_queueHandle, 0);
		task_1_time = (time_struct*)event.value.v;
		osMutexRelease(task_1_time_queue_mutexHandle);
	}
	uint32_t period = task_1_time -> period;
	uint32_t execution_time = task_1_time -> execution_time;
	osThreadDef(red_light_task, RedLightTask, osPriorityLow, 0, 128);
	red_light_taskHandle = 0;
	while(red_light_taskHandle == 0){
		red_light_taskHandle = osThreadCreate(osThread(red_light_task), NULL);
		if(red_light_taskHandle == 0){
			printf("Red returned 0, trying again\n");
		}
	}
	release_dd_task(red_light_taskHandle, PERIODIC, count, count * period, execution_time);
	/* USER CODE END task_1_timer_callback */
}

/* task_2_timer_callback function */
void task_2_timer_callback(void const * argument)
{
	/* USER CODE BEGIN task_2_timer_callback */
	static uint32_t count = 0;
	static time_struct* task_2_time = NULL;
	count++;
	if(task_2_time == NULL){
		osMutexWait(task_2_time_queue_mutexHandle, osWaitForever);
		osEvent event = osMessageGet(task_2_time_queueHandle, 0);
		task_2_time = (time_struct*)event.value.v;
		osMutexRelease(task_2_time_queue_mutexHandle);
	}
	uint32_t period = task_2_time -> period;
	uint32_t execution_time = task_2_time -> execution_time;
	osThreadDef(amber_light_tas, AmberLightTask, osPriorityLow, 0, 128);
	amber_light_tasHandle = 0;
	while(amber_light_tasHandle == 0){
		amber_light_tasHandle = osThreadCreate(osThread(amber_light_tas), NULL);
		if(amber_light_tasHandle == 0){
			printf("Amber returned 0, trying again\n");
		}
	}
	release_dd_task(amber_light_tasHandle, PERIODIC, count, count * period, execution_time);
	/* USER CODE END task_2_timer_callback */
}

/* task_3_timer_callback function */
void task_3_timer_callback(void const * argument)
{
	/* USER CODE BEGIN task_3_timer_callback */
	static uint32_t count = 0;
	static time_struct* task_3_time = NULL;
	count++;
	if(task_3_time == NULL){
		osMutexWait(task_3_time_queue_mutexHandle, osWaitForever);
		osEvent event = osMessageGet(task_3_time_queueHandle, 0);
		task_3_time = (time_struct*)event.value.v;
		osMutexRelease(task_3_time_queue_mutexHandle);
	}
	uint32_t period = task_3_time -> period;
	uint32_t execution_time = task_3_time -> execution_time;
	osThreadDef(green_light_tas, GreenLightTask, osPriorityLow, 0, 128);
	green_light_tasHandle = 0;
	while(green_light_tasHandle == 0){
		green_light_tasHandle = osThreadCreate(osThread(green_light_tas), NULL);
		if(green_light_tasHandle == 0){
			printf("Green returned 0, trying again!\n");
		}
	}
	release_dd_task(green_light_tasHandle, PERIODIC, count, count * period, execution_time);
	/* USER CODE END task_3_timer_callback */
}

/**
* @brief Function implementing the red_light_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RedLightTask */
void RedLightTask(void const * argument)
{
  /* USER CODE BEGIN RedLightTask */
	static int firstRun = 1;
	if(firstRun == 1){
		firstRun = 0;
		osThreadTerminate(osThreadGetId());
	}
	osMutexWait(task_duration_queue_mutexHandle, osWaitForever);
	osEvent event = osMessageGet(task_duration_queueHandle, osWaitForever);
	int time = event.value.v;
	osMutexRelease(task_duration_queue_mutexHandle);
	HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
	osDelay(time);
	HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);

	complete_dd_task(osThreadGetId());
	osThreadTerminate(osThreadGetId());
  /* USER CODE END RedLightTask */
}

/* USER CODE BEGIN Header_AmberLightTask */
/**
* @brief Function implementing the amber_light_tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AmberLightTask */
void AmberLightTask(void const * argument)
{
  /* USER CODE BEGIN AmberLightTask */
	static int firstRun = 1;
	if(firstRun == 1){
		firstRun = 0;
		osThreadTerminate(osThreadGetId());
	}
	osMutexWait(task_duration_queue_mutexHandle, osWaitForever);
	osEvent event = osMessageGet(task_duration_queueHandle, osWaitForever);
	int time = event.value.v;
	osMutexRelease(task_duration_queue_mutexHandle);
	HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
	osDelay(time);
	HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);

	complete_dd_task(osThreadGetId());
	osThreadTerminate(osThreadGetId());
  /* USER CODE END AmberLightTask */
}

/* USER CODE BEGIN Header_GreenLightTask */
/**
* @brief Function implementing the green_light_tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GreenLightTask */
void GreenLightTask(void const * argument)
{
  /* USER CODE BEGIN GreenLightTask */
	static int firstRun = 1;
	if(firstRun == 1){
		firstRun = 0;
		osThreadTerminate(osThreadGetId());
	}
	osMutexWait(task_duration_queue_mutexHandle, osWaitForever);
	osEvent event = osMessageGet(task_duration_queueHandle, osWaitForever);
	int time = event.value.v;
	osMutexRelease(task_duration_queue_mutexHandle);
	HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
	osDelay(time);
	HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);

	complete_dd_task(osThreadGetId());
	osThreadTerminate(osThreadGetId());
  /* USER CODE END GreenLightTask */
}

/* dds_control_callback function */
void dds_control_callback(void const * argument)
{
  /* USER CODE BEGIN dds_control_callback */

  /* USER CODE END dds_control_callback */
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
