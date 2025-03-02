/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include "sensor_master.hpp"
extern "C" {
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "main.h"
}
#include "PID.hpp"
#include "controll_master.hpp"
#include "data_types.hpp"
#include "motor_controller.hpp"
#include "parameters.hpp"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define UART_DEBUGG_SENS
// #define UART_DEBUGG_MOTOR
// #define UART_DEBUGG_PID
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for sensorProcessin */
osThreadId_t sensorProcessinHandle;
const osThreadAttr_t sensorProcessin_attributes = {
    .name = "sensorProcessin",
    .stack_size = 300 * 4,
    .priority = static_cast<osPriority_t>(osPriorityNormal3),
};
/* Definitions for motorDriver */
osThreadId_t motorDriverHandle;
const osThreadAttr_t motorDriver_attributes = {
    .name = "motorDriver",
    .stack_size = 300 * 4,
    .priority = static_cast<osPriority_t>(osPriorityNormal5),
};
/* Definitions for controllerTask */
osThreadId_t controllerTaskHandle;
const osThreadAttr_t controllerTask_attributes = {
    .name = "controllerTask",
    .stack_size = 300 * 4,
    .priority = static_cast<osPriority_t>(osPriorityBelowNormal4),
};
/* Definitions for controllMaster */
osThreadId_t controllMasterHandle;
const osThreadAttr_t controllMaster_attributes = {
    .name = "controllMaster",
    .stack_size = 128 * 4,
    .priority = static_cast<osPriority_t>(osPriorityAboveNormal6),
};
/* Definitions for UARTLogger */
osThreadId_t UARTLoggerHandle;
const osThreadAttr_t UARTLogger_attributes = {
    .name = "UARTLogger",
    .stack_size = 128 * 4,
    .priority = static_cast<osPriority_t>(osPriorityBelowNormal),
};
/* Definitions for sensorData */
osMessageQueueId_t sensorDataHandle;
const osMessageQueueAttr_t sensorData_attributes = {.name = "sensorData"};
/* Definitions for controllData */
osMessageQueueId_t controllDataHandle;
const osMessageQueueAttr_t controllData_attributes = {.name = "controllData"};
/* Definitions for controllMasterData */
osMessageQueueId_t controllMasterDataHandle;
const osMessageQueueAttr_t controllMasterData_attributes = {
    .name = "controllMasterData"};
/* Definitions for newSensorDataAvailable */
osEventFlagsId_t newSensorDataAvailableHandle;
const osEventFlagsAttr_t newSensorDataAvailable_attributes = {
    .name = "newSensorDataAvailable"};
/* Definitions for newDataAvailableForController */
osEventFlagsId_t newDataAvailableForControllerHandle;
const osEventFlagsAttr_t newDataAvailableForController_attributes = {
    .name = "newDataAvailableForController"};
/* Definitions for newUARTDataAvailable */
osEventFlagsId_t newUARTDataAvailableHandle;
const osEventFlagsAttr_t newUARTDataAvailable_attributes = {
    .name = "newUARTDataAvailable"};
/* Definitions for debuggUARTisFree */
osEventFlagsId_t debuggUARTisFreeHandle;
const osEventFlagsAttr_t debuggUARTisFree_attributes = {.name =
                                                            "debuggUARTisFree"};
/* Definitions for i2cDataRecieved */
osEventFlagsId_t i2cDataRecievedHandle;
const osEventFlagsAttr_t i2cDataRecieved_attributes = {.name =
                                                           "i2cDataRecieved"};
/* Definitions for i2cDataTransmitted */
osEventFlagsId_t i2cDataTransmittedHandle;
const osEventFlagsAttr_t i2cDataTransmitted_attributes = {
    .name = "i2cDataTransmitted"};
/* Definitions for i2cMemTransmitted */
osEventFlagsId_t i2cMemTransmittedHandle;
const osEventFlagsAttr_t i2cMemTransmitted_attributes = {
    .name = "i2cMemTransmitted"};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM11_Init(void);
void StartSensorProcessingTask(void *argument);
void StartMotorDriverTask(void *argument);
void StartControllerTask(void *argument);
void StartControllMaster(void *argument);
void StartUARTLogger(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM11_Init();
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
  /* creation of sensorData */
  sensorDataHandle =
      osMessageQueueNew(100, sizeof(sensor_data_type), &sensorData_attributes);

  /* creation of controllData */
  controllDataHandle = osMessageQueueNew(100, sizeof(controll_data_type),
                                         &controllData_attributes);

  /* creation of controllMasterData */
  controllMasterDataHandle = osMessageQueueNew(
      100, sizeof(ControllSignal_data_type), &controllMasterData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of sensorProcessin */
  sensorProcessinHandle =
      osThreadNew(StartSensorProcessingTask, NULL, &sensorProcessin_attributes);

  /* creation of motorDriver */
  motorDriverHandle =
      osThreadNew(StartMotorDriverTask, NULL, &motorDriver_attributes);

  /* creation of controllerTask */
  controllerTaskHandle =
      osThreadNew(StartControllerTask, NULL, &controllerTask_attributes);

  /* creation of controllMaster */
  controllMasterHandle =
      osThreadNew(StartControllMaster, NULL, &controllMaster_attributes);

  /* creation of UARTLogger */
  UARTLoggerHandle = osThreadNew(StartUARTLogger, NULL, &UARTLogger_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of newSensorDataAvailable */
  newSensorDataAvailableHandle =
      osEventFlagsNew(&newSensorDataAvailable_attributes);

  /* creation of newDataAvailableForController */
  newDataAvailableForControllerHandle =
      osEventFlagsNew(&newDataAvailableForController_attributes);

  /* creation of newUARTDataAvailable */
  newUARTDataAvailableHandle =
      osEventFlagsNew(&newUARTDataAvailable_attributes);

  /* creation of debuggUARTisFree */
  debuggUARTisFreeHandle = osEventFlagsNew(&debuggUARTisFree_attributes);

  /* creation of i2cDataRecieved */
  i2cDataRecievedHandle = osEventFlagsNew(&i2cDataRecieved_attributes);

  /* creation of i2cDataTransmitted */
  i2cDataTransmittedHandle = osEventFlagsNew(&i2cDataTransmitted_attributes);

  /* creation of i2cMemTransmitted */
  i2cMemTransmittedHandle = osEventFlagsNew(&i2cMemTransmitted_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 839;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU_Data_Ready_Pin */
  GPIO_InitStruct.Pin = MPU_Data_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU_Data_Ready_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (newSensorDataAvailableHandle != NULL) {
    osEventFlagsSet(newSensorDataAvailableHandle, 0x00000001U);
  }
}

static uint32_t GPIO_UART_ALIVE_LED = 0;

uint8_t Rx_data[50];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart6) {
    if (GPIO_UART_ALIVE_LED == 60) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      GPIO_UART_ALIVE_LED = 0;
    } else {
      GPIO_UART_ALIVE_LED++;
    }
    if (newUARTDataAvailableHandle != NULL) {
      osEventFlagsSet(newUARTDataAvailableHandle, 0x00000001U);
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (debuggUARTisFreeHandle != NULL) {
    osEventFlagsSet(debuggUARTisFreeHandle, 0x00000001U);
  }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
  if (i2cDataTransmittedHandle != NULL) {
    osEventFlagsSet(i2cDataTransmittedHandle, 0x00000001U);
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
  if (i2cDataRecievedHandle != NULL) {
    osEventFlagsSet(i2cDataRecievedHandle, 0x00000001U);
  }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
  if (i2cMemTransmittedHandle != NULL) {
    osEventFlagsSet(i2cMemTransmittedHandle, 0x00000001U);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensorProcessingTask */
/**
 * @brief  Function implementing the sensorProcessin thread.
 * @param  argument: Not used
 * @retval None
 */
sensor_data_type servo_angle_for_queue;
/* USER CODE END Header_StartSensorProcessingTask */

void StartSensorProcessingTask(void *argument) {
  /* USER CODE BEGIN 5 */
  sensormaster::SensorProcessing sensor_processor(&hi2c1);
#ifdef UART_DEBUGG_SENS
  UartLogger *Logger = UartLogger::getUartLoggerSingeleton(&huart1);
  Logger->add_data_logger("\n\rDebugger Runs");
#endif

  for (;;) {
    uint16_t newSensorDataReady;
    osEventFlagsWait(newSensorDataAvailableHandle, 0x00000001U, osFlagsWaitAny,
                     osWaitForever);

    servo_angle_for_queue.sampple_time = HAL_GetTick();
    sensor_processor.sensor_data_processing();

    servo_angle_for_queue.angle_pitch =
        SENSOR_OFFSET + sensor_processor.get_filtered_angle_x();
    servo_angle_for_queue.angle_roll =
        SENSOR_OFFSET + sensor_processor.get_filtered_angle_y();

    // TODO Fix the magnetometer config
    //  sensor_processor.magnetometer_sensor_data_processing();
    //  servo_angle_for_queue.angle_yaw =
    //  sensor_processor.get_magnetometer_yaw();

    // to do cleamn up
#ifdef UART_DEBUGG_SENS
    Logger->add_data_logger("\n\rPITCH");
    Logger->add_data_logger(int32_t(servo_angle_for_queue.angle_pitch));
    Logger->add_data_logger("\n\rROLL");
    Logger->add_data_logger(int32_t(servo_angle_for_queue.angle_roll));
    Logger->add_data_logger("\n\rYAW");
    Logger->add_data_logger(int32_t(0));
#endif
    osMessageQueuePut(sensorDataHandle, &servo_angle_for_queue, 0U, 1000);

    if (newDataAvailableForControllerHandle != NULL) {
      osEventFlagsSet(newDataAvailableForControllerHandle,
                      NEW_DATA_FORM_SENSOR);
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMotorDriverTask */
/**
 * @brief Function implementing the motorDriver thread.
 * @param argument: Not used
 * @retval None
 */
uint16_t debug_var = 0;
/* USER CODE END Header_StartMotorDriverTask */
void StartMotorDriverTask(void *argument) {
  /* USER CODE BEGIN StartMotorDriverTask */
  /* Infinite loop */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  motor_controller::Motor_controller motor_calculation_master;
#ifdef UART_DEBUGG_MOTOR
  UartLogger *Logger = UartLogger::getUartLoggerSingeleton(&huart1);
  Logger->add_data_logger("\n\rDebugger Runs");
#endif

  for (;;) {
    controll_data_type controll_Date_from_queue{};
    osMessageQueueGet(controllDataHandle, &controll_Date_from_queue, NULL,
                      1000);

    bldc_driver_data_type bldc_driver =
        motor_calculation_master.update_motor_driver(controll_Date_from_queue);

#ifdef UART_DEBUGG_MOTOR
    Logger->add_data_logger("\n\rM1");
    Logger->add_data_logger(int32_t(MOTOR_IDDLE_TICK + bldc_driver.motor_1));
    Logger->add_data_logger("\n\rM2");
    Logger->add_data_logger(int32_t(MOTOR_IDDLE_TICK + bldc_driver.motor_2));
    Logger->add_data_logger("\n\rM3");
    Logger->add_data_logger(int32_t(MOTOR_IDDLE_TICK + bldc_driver.motor_3));
    Logger->add_data_logger("\n\rM4");
    Logger->add_data_logger(int32_t(MOTOR_IDDLE_TICK + bldc_driver.motor_4));
#endif

    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1,
                         MOTOR_IDDLE_TICK + bldc_driver.motor_1);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2,
                         MOTOR_IDDLE_TICK + bldc_driver.motor_2);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3,
                         MOTOR_IDDLE_TICK + bldc_driver.motor_3);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4,
                         MOTOR_IDDLE_TICK + bldc_driver.motor_4);
  }
  /* USER CODE END StartMotorDriverTask */
}

/* USER CODE BEGIN Header_StartControllerTask */
/**
 * @brief Function implementing the controllerTask thread.
 * @param argument: Not used
 * @retval None
 */
controll_data_type contrull_Data_for_queue;
ControllSignal_data_type decoded_UART_message_from_queue;
sensor_data_type servo_angle_from_queue_buffer;
sensor_data_type servo_angle_from_queue;
float sampple_time_buffer;
/* USER CODE END Header_StartControllerTask */
void StartControllerTask(void *argument) {
  /* USER CODE BEGIN StartcontrollerTask */
  PID_controller PID_pitch(PITCH_P, PITCH_I, PITCH_D, HAL_GetTick());
  PID_controller PID_roll(ROLL_P, ROLL_I, ROLL_D, HAL_GetTick());
  PID_controller_yaw PID_yaw(YAW_P, YAW_I, YAW_D, HAL_GetTick());
#ifdef UART_DEBUGG_PID
  UartLogger *Logger = UartLogger::getUartLoggerSingeleton(&huart1);
  Logger->add_data_logger("\n\rDebugger Runs");
#endif
  /* Infinite loop */
  for (;;) {

    uint32_t message_status =
        osEventFlagsWait(newDataAvailableForControllerHandle,
                         NEW_DATA_FORM_SENSOR | NEW_DATA_FORM_CONTROLL,
                         osFlagsWaitAny, osWaitForever);

    if (message_status | NEW_DATA_FORM_SENSOR) {
      osMessageQueueGet(sensorDataHandle, &servo_angle_from_queue_buffer, NULL,
                        0);
      servo_angle_from_queue = servo_angle_from_queue_buffer;
    }

    if (message_status | NEW_DATA_FORM_CONTROLL) {
      osMessageQueueGet(controllMasterDataHandle,
                        &decoded_UART_message_from_queue, NULL, 0);
      contrull_Data_for_queue.PID_controll_state =
          decoded_UART_message_from_queue.state;
    }

    if (sampple_time_buffer != servo_angle_from_queue.sampple_time) {

      contrull_Data_for_queue.angle_pitch = PID_pitch.get_PID_controll_signal(
          decoded_UART_message_from_queue.angle_pitch,
          servo_angle_from_queue.angle_pitch,
          servo_angle_from_queue.sampple_time,
          decoded_UART_message_from_queue.state,
          decoded_UART_message_from_queue.acceleration);
      contrull_Data_for_queue.angle_roll = PID_roll.get_PID_controll_signal(
          decoded_UART_message_from_queue.angle_roll,
          servo_angle_from_queue.angle_roll,
          servo_angle_from_queue.sampple_time,
          decoded_UART_message_from_queue.state,
          decoded_UART_message_from_queue.acceleration);
      contrull_Data_for_queue.angle_yaw = PID_yaw.get_PID_controll_signal(
          decoded_UART_message_from_queue.angle_yaw,
          servo_angle_from_queue.angle_yaw, servo_angle_from_queue.sampple_time,
          decoded_UART_message_from_queue.state,
          decoded_UART_message_from_queue.acceleration);
    }
    sampple_time_buffer = servo_angle_from_queue.sampple_time;
    contrull_Data_for_queue.acceleration =
        decoded_UART_message_from_queue.acceleration;

#ifdef UART_DEBUGG_PID
    Logger->add_data_logger("\n\rPi");
    Logger->add_data_logger(
        int32_t(servo_angle_from_queue.angle_pitch * LOGGING_MULTIPLIER));
    Logger->add_data_logger("\n\rRi");
    Logger->add_data_logger(
        int32_t(servo_angle_from_queue.angle_roll * LOGGING_MULTIPLIER));

    // Logger->add_data_logger("\n\rYi");
    // Logger->add_data_logger(int32_t
    // (servo_angle_from_queue.angle_yaw*LOGGING_MULTIPLIER));

    Logger->add_data_logger("\n\rPo");
    Logger->add_data_logger(
        int32_t(contrull_Data_for_queue.angle_pitch * LOGGING_MULTIPLIER));
    Logger->add_data_logger("\n\rRo");
    Logger->add_data_logger(
        int32_t(contrull_Data_for_queue.angle_roll * LOGGING_MULTIPLIER));

    // Logger->add_data_logger("\n\rYo");
    // Logger->add_data_logger(int32_t
    // (contrull_Data_for_queue.angle_yaw*LOGGING_MULTIPLIER));
#endif

    osMessageQueuePut(controllDataHandle, &contrull_Data_for_queue, 0U, 1000);
  }
  /* USER CODE END StartControllerTask */
}

/* USER CODE BEGIN Header_StartControllMaster */
/**
 * @brief Function implementing the controllMaster thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControllMaster */
void StartControllMaster(void *argument) {

  ControllMessageManeger UART_controll_master;
  /* USER CODE BEGIN StartControllMaster */
  HAL_UART_Receive_DMA(&huart6, Rx_data, IBUS_LENGTH);
  /* Infinite loop */
  for (;;) {

    osEventFlagsWait(newUARTDataAvailableHandle, 0x00000001U, osFlagsWaitAny,
                     osWaitForever);
    ControllSignal_data_type decoded_UART_message =
        UART_controll_master.decode_new_message(Rx_data, IBUS_LENGTH);

    HAL_UART_Receive_DMA(&huart6, Rx_data, IBUS_LENGTH);
    osMessageQueuePut(controllMasterDataHandle, &decoded_UART_message, 0U,
                      1000);

    if (newDataAvailableForControllerHandle != NULL) {
      osEventFlagsSet(newDataAvailableForControllerHandle,
                      NEW_DATA_FORM_CONTROLL);
    }
  }
  /* USER CODE END StartControllMaster */
}
extern unsigned long ulHighFrequencyTimerTicks;
extern "C" {
void configureTimerForRunTimeStats(void) {
  ulHighFrequencyTimerTicks = 0;
  HAL_TIM_Base_Start_IT(&htim11);
}

unsigned long getRunTimeCounterValue(void) { return ulHighFrequencyTimerTicks; }
}
/* USER CODE BEGIN Header_StartUARTLogger */
/**
 * @brief Function implementing the UARTLogger thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUARTLogger */
void StartUARTLogger(void *argument) {
  /* USER CODE BEGIN StartUARTLogger */
  /* Infinite loop */
  UartLogger *debugger_to_be_triggered =
      UartLogger::getUartLoggerSingeleton(&huart1);
  for (;;) {
    osEventFlagsWait(debuggUARTisFreeHandle, 0x00000001U, osFlagsWaitAny,
                     osWaitForever);
    debugger_to_be_triggered->dataTransferFinished();
  }
  /* USER CODE END StartUARTLogger */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM10 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
