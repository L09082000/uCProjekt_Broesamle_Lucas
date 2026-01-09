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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "../Sensors/AccelerationSensor/lsm6dsl.h"
#include "../Sensors/AirPressureSensor/lps22hb.h"
#include "../Sensors/HumiditySensor/hts221.h"
#include "../Sensors/MagnetometerSensor/lis3mdl.h"
#include "../Sensors/ToFSensor/vl53l0x.h"
#include "../Filter/filter.h"

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
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MagnetoTask */
osThreadId_t MagnetoTaskHandle;
const osThreadAttr_t MagnetoTask_attributes = {
  .name = "MagnetoTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ToFTask */
osThreadId_t ToFTaskHandle;
const osThreadAttr_t ToFTask_attributes = {
  .name = "ToFTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BaroTask */
osThreadId_t BaroTaskHandle;
const osThreadAttr_t BaroTask_attributes = {
  .name = "BaroTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for HumidityTask */
osThreadId_t HumidityTaskHandle;
const osThreadAttr_t HumidityTask_attributes = {
  .name = "HumidityTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for InitTask */
osThreadId_t InitTaskHandle;
const osThreadAttr_t InitTask_attributes = {
  .name = "InitTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for DataTransmitTas */
osThreadId_t DataTransmitTasHandle;
const osThreadAttr_t DataTransmitTas_attributes = {
  .name = "DataTransmitTas",
  .stack_size = 265 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for DataFilterTask */
osThreadId_t DataFilterTaskHandle;
const osThreadAttr_t DataFilterTask_attributes = {
  .name = "DataFilterTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for I2C2available */
osSemaphoreId_t I2C2availableHandle;
const osSemaphoreAttr_t I2C2available_attributes = {
  .name = "I2C2available"
};
/* Definitions for UART1available */
osSemaphoreId_t UART1availableHandle;
const osSemaphoreAttr_t UART1available_attributes = {
  .name = "UART1available"
};
/* USER CODE BEGIN PV */
uint8_t ucHeap[ configTOTAL_HEAP_SIZE ] __attribute__((section(".FreeRTOSHeap")));

uint32_t max_cycles_acc = 0;
uint32_t max_cycles_mag = 0;
float runtime_us_acc = 0.0f;
float runtime_us_mag = 0.0f;

TRANSMIT_DATA transmit_data ={.delimiter = 0xDEADBEEF};

extern LSM6DSL_VALUES lsm6dsl_values;
extern LIS3MDL_VALUES lis3mdl_values;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void IMU_Task(void *argument);
void Magneto_Task(void *argument);
void ToF_Task(void *argument);
void Baro_Task(void *argument);
void Humidity_Task(void *argument);
void Init_Task(void *argument);
void Data_Transmit_Task(void *argument);
void Data_Filter_Task(void *argument);

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
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Debug Trace aktivieren
  DWT->CYCCNT = 0;                                 // Zähler zurücksetzen
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Zähler starten
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of I2C2available */
  I2C2availableHandle = osSemaphoreNew(1, 1, &I2C2available_attributes);

  /* creation of UART1available */
  UART1availableHandle = osSemaphoreNew(1, 1, &UART1available_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(IMU_Task, NULL, &IMUTask_attributes);

  /* creation of MagnetoTask */
  MagnetoTaskHandle = osThreadNew(Magneto_Task, NULL, &MagnetoTask_attributes);

  /* creation of ToFTask */
  ToFTaskHandle = osThreadNew(ToF_Task, NULL, &ToFTask_attributes);

  /* creation of BaroTask */
  BaroTaskHandle = osThreadNew(Baro_Task, NULL, &BaroTask_attributes);

  /* creation of HumidityTask */
  HumidityTaskHandle = osThreadNew(Humidity_Task, NULL, &HumidityTask_attributes);

  /* creation of InitTask */
  InitTaskHandle = osThreadNew(Init_Task, NULL, &InitTask_attributes);

  /* creation of DataTransmitTas */
  DataTransmitTasHandle = osThreadNew(Data_Transmit_Task, NULL, &DataTransmitTas_attributes);

  /* creation of DataFilterTask */
  DataFilterTaskHandle = osThreadNew(Data_Filter_Task, NULL, &DataFilterTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* terminate the default Task because it just exists because of the STM23CubeIDE configuration set it mandatory - that is not needed because
   * a idle task is created by FreeRTOS anyway   */
  osThreadTerminate(defaultTaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_Start();
  SEGGER_SYSVIEW_PrintfHost("START");
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10B21F61;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VL53L0X_XSHUT_GPIO_Port, VL53L0X_XSHUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUTTON_EXTI13_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI10_Pin LSM6DSL_INT1_EXTI11_Pin HTS221_DRDY_EXTI15_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI10_Pin|LSM6DSL_INT1_EXTI11_Pin|HTS221_DRDY_EXTI15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VL53L0X_XSHUT_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VL53L0X_XSHUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VL53L0X_GPIO1_EXTI7_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VL53L0X_GPIO1_EXTI7_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 10, 0);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 0);

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

/* USER CODE BEGIN Header_IMU_Task */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Task */
void IMU_Task(void *argument)
{
  /* USER CODE BEGIN IMU_Task */
  HAL_StatusTypeDef HAL_status = HAL_OK;
  /* Infinite loop */
  for(;;)
  {
	  // Auf Aktivierungsflag warten
	  osThreadFlagsWait(IMU_SENSOR_THREAD_ACTIVATE_FLAG, osFlagsWaitAll, osWaitForever);

	  // I2C-Bus sichern
	  osSemaphoreAcquire(I2C2availableHandle, osWaitForever);

	  // Prüfen, ob neue IMU-Daten vorliegen
	  HAL_status = LSM6DSL_data_ready();
	  osSemaphoreRelease(I2C2availableHandle);

	  // Flagge: neue Sensorwerte vorhanden
	  osThreadFlagsSet(DataFilterTaskHandle, RAW_IMU_DATA_READY_FLAG);
  }
  /* USER CODE END IMU_Task */
}

/* USER CODE BEGIN Header_Magneto_Task */
/**
* @brief Function implementing the MagnetoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Magneto_Task */
void Magneto_Task(void *argument)
{
  /* USER CODE BEGIN Magneto_Task */
  HAL_StatusTypeDef HAL_status = HAL_OK;
  /* Infinite loop */
  for(;;)
  {
	  // Auf Aktivierungsflag warten
	  osThreadFlagsWait(MAG_SENSOR_THREAD_ACTIVATE_FLAG, osFlagsWaitAll, osWaitForever);

	  // I2C-Bus sichern
	  osSemaphoreAcquire(I2C2availableHandle, osWaitForever);

	  // Prüfen, ob neue Magnetometer-Daten vorliegen
	  HAL_status = LIS3MDL_data_ready();
	  osSemaphoreRelease(I2C2availableHandle);

	  // Flagge: neue Sensorwerte vorhanden
	  osThreadFlagsSet(DataFilterTaskHandle, RAW_MAG_DATA_READY_FLAG);
  }
  /* USER CODE END Magneto_Task */
}

/* USER CODE BEGIN Header_ToF_Task */
/**
* @brief Function implementing the ToFTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ToF_Task */
void ToF_Task(void *argument)
{
  /* USER CODE BEGIN ToF_Task */
  HAL_StatusTypeDef HAL_status = HAL_OK;
  /* Infinite loop */
  for(;;)
  {
	  // Auf Aktivierungsflag warten
	  osThreadFlagsWait(TOF_SENSOR_THREAD_ACTIVATE_FLAG, osFlagsWaitAll, osWaitForever);

	  // I2C-Bus sichern
	  osSemaphoreAcquire(I2C2availableHandle, osWaitForever);

	  // Prüfen, ob neue Magnetometer-Daten vorliegen
	  HAL_status = VL53L0X_data_ready();
	  osSemaphoreRelease(I2C2availableHandle);
  }
  /* USER CODE END ToF_Task */
}

/* USER CODE BEGIN Header_Baro_Task */
/**
* @brief Function implementing the BaroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Baro_Task */
void Baro_Task(void *argument)
{
  /* USER CODE BEGIN Baro_Task */
  HAL_StatusTypeDef HAL_status = HAL_OK;
  /* Infinite loop */
  for(;;)
  {
	  // Auf Aktivierungsflag warten
	  osThreadFlagsWait(BARO_SENSOR_THREAD_ACTIVATE_FLAG, osFlagsWaitAll, osWaitForever);

	  // I2C-Bus sichern
	  osSemaphoreAcquire(I2C2availableHandle, osWaitForever);

	  // Prüfen, ob neue Magnetometer-Daten vorliegen
	  HAL_status = LPS22HB_data_ready();
	  osSemaphoreRelease(I2C2availableHandle);
  }
  /* USER CODE END Baro_Task */
}

/* USER CODE BEGIN Header_Humidity_Task */
/**
* @brief Function implementing the HumidityTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Humidity_Task */
void Humidity_Task(void *argument)
{
  /* USER CODE BEGIN Humidity_Task */
  HAL_StatusTypeDef HAL_status = HAL_OK;
  /* Infinite loop */
  for(;;)
  {
	  // Auf Aktivierungsflag warten
	  osThreadFlagsWait(HUMIDITY_SENSOR_THREAD_ACTIVATE_FLAG, osFlagsWaitAll, osWaitForever);

	  // I2C-Bus sichern
	  osSemaphoreAcquire(I2C2availableHandle, osWaitForever);
	  // Prüfen, ob neue Magnetometer-Daten vorliegen
	  HAL_status = HTS221_data_ready();
	  osSemaphoreRelease(I2C2availableHandle);
  }
  /* USER CODE END Humidity_Task */
}

/* USER CODE BEGIN Header_Init_Task */
/**
* @brief Function implementing the InitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Init_Task */
void Init_Task(void *argument)
{
  /* USER CODE BEGIN Init_Task */
  vl53l0x_initialize(); /* ToF */
  HTS221_initialize();  /* Humidity */
  LPS22HB_initialize(); /* Barometer */
  LSM6DSL_initialize(); /* IMU */
  LIS3MDL_initialize(); /* Magnetometer */

  /* initialisation mainly done -> clear interrupts in case they already are active and enable them */
  HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* Initialise Filter */
  LSM6DSL_Filter_Init();
  LIS3MDL_Filter_Init();

  /* initialisation finally done --> terminate this task */
  osThreadTerminate(InitTaskHandle);

  /* USER CODE END Init_Task */
}

/* USER CODE BEGIN Header_Data_Transmit_Task */
/**
* @brief Function implementing the DataTransmitTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Data_Transmit_Task */
void Data_Transmit_Task(void *argument)
{
  /* USER CODE BEGIN Data_Transmit_Task */
  /* Infinite loop */
  for(;;)
      {
		osThreadFlagsWait(FILTERED_DATA_READY_FLAG, osFlagsWaitAny, osWaitForever);
		SEGGER_SYSVIEW_PrintfHost("DataTransmit");

		// UART-Semaphore sichern
		osSemaphoreAcquire(UART1availableHandle, osWaitForever);

		// Rohdaten kopieren (IMU + Magnetometer)
		memcpy(&transmit_data.acc_x, &lsm6dsl_values.acc_x, 6 * sizeof(float));  // acc + gyro
		memcpy(&transmit_data.mag_x, &lis3mdl_values.x, 3 * sizeof(float));

		// Gefilterte Werte kopieren
		memcpy(&transmit_data.acc_x_filtered, &lsm6dsl_filtered_values.acc_x, 6 * sizeof(float));
		memcpy(&transmit_data.mag_x_filtered, &lis3mdl_filtered_values.x, 3 * sizeof(float));

		// Daten per DMA senden
		HAL_UART_Transmit_DMA(&huart1, (const uint8_t *)&transmit_data, sizeof(transmit_data));

		// Halb-Transmit Interrupt deaktivieren (nicht benötigt)
		__HAL_DMA_DISABLE_IT(huart1.hdmatx, DMA_IT_HT);
	  }
  /* USER CODE END Data_Transmit_Task */
}

/* USER CODE BEGIN Header_Data_Filter_Task */
/**
* @brief Function implementing the DataFilterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Data_Filter_Task */
void Data_Filter_Task(void *argument)
{
  /* USER CODE BEGIN Data_Filter_Task */
  uint32_t start, stop;

  for (;;)
  {
    uint32_t flags = osThreadFlagsWait(RAW_IMU_DATA_READY_FLAG | RAW_MAG_DATA_READY_FLAG, osFlagsWaitAny, osWaitForever);

    if (flags & RAW_IMU_DATA_READY_FLAG)
    {
      __disable_irq();              // Interrupts kurz deaktivieren
      start = DWT->CYCCNT;

      LSM6DSL_Filter_Update(lsm6dsl_values);

      stop = DWT->CYCCNT;
      __enable_irq();

      uint32_t cycles = stop - start;
      if(cycles > max_cycles_acc) max_cycles_acc = cycles;

      // Laufzeit in µs berechnen und global speichern
      runtime_us_acc = (float)cycles / SystemCoreClock * 1e6;
      SEGGER_SYSVIEW_PrintfHost("LSM6DSL Filter: %lu cycles = %.2f us", cycles, runtime_us_acc);
    }

    if (flags & RAW_MAG_DATA_READY_FLAG)
    {
      __disable_irq();
      start = DWT->CYCCNT;

      LIS3MDL_Filter_Update(lis3mdl_values);

      stop = DWT->CYCCNT;
      __enable_irq();

      uint32_t cycles = stop - start;
      if(cycles > max_cycles_mag) max_cycles_mag = cycles;

      // Laufzeit in µs berechnen und global speichern
      runtime_us_mag = (float)cycles / SystemCoreClock * 1e6;
      SEGGER_SYSVIEW_PrintfHost("LIS3MDL Filter: %lu cycles = %.2f us", cycles, runtime_us_mag);
    }

    osThreadFlagsSet(DataTransmitTasHandle, FILTERED_DATA_READY_FLAG);
  }
  /* USER CODE END Data_Filter_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
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
  /* User can add his own implementation to report the HAL error return state
   */
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        osSemaphoreRelease(UART1availableHandle);
    }
}
