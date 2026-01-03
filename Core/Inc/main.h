/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_EXTI13_Pin GPIO_PIN_13
#define BUTTON_EXTI13_GPIO_Port GPIOC
#define BUTTON_EXTI13_EXTI_IRQn EXTI15_10_IRQn
#define INTERNAL_I2C2_SCL_Pin GPIO_PIN_10
#define INTERNAL_I2C2_SCL_GPIO_Port GPIOB
#define INTERNAL_I2C2_SDA_Pin GPIO_PIN_11
#define INTERNAL_I2C2_SDA_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define LPS22HB_INT_DRDY_EXTI10_Pin GPIO_PIN_10
#define LPS22HB_INT_DRDY_EXTI10_GPIO_Port GPIOD
#define LPS22HB_INT_DRDY_EXTI10_EXTI_IRQn EXTI15_10_IRQn
#define LSM6DSL_INT1_EXTI11_Pin GPIO_PIN_11
#define LSM6DSL_INT1_EXTI11_GPIO_Port GPIOD
#define LSM6DSL_INT1_EXTI11_EXTI_IRQn EXTI15_10_IRQn
#define HTS221_DRDY_EXTI15_Pin GPIO_PIN_15
#define HTS221_DRDY_EXTI15_GPIO_Port GPIOD
#define HTS221_DRDY_EXTI15_EXTI_IRQn EXTI15_10_IRQn
#define VL53L0X_XSHUT_Pin GPIO_PIN_6
#define VL53L0X_XSHUT_GPIO_Port GPIOC
#define VL53L0X_GPIO1_EXTI7_Pin GPIO_PIN_7
#define VL53L0X_GPIO1_EXTI7_GPIO_Port GPIOC
#define VL53L0X_GPIO1_EXTI7_EXTI_IRQn EXTI9_5_IRQn
#define LSM3MDL_DRDY_EXTI8_Pin GPIO_PIN_8
#define LSM3MDL_DRDY_EXTI8_GPIO_Port GPIOC
#define LSM3MDL_DRDY_EXTI8_EXTI_IRQn EXTI9_5_IRQn
#define SYS_JTMS_SWDIO_Pin GPIO_PIN_13
#define SYS_JTMS_SWDIO_GPIO_Port GPIOA
#define SYS_JTCK_SWCLK_Pin GPIO_PIN_14
#define SYS_JTCK_SWCLK_GPIO_Port GPIOA
#define SYS_JTD0_SWO_Pin GPIO_PIN_3
#define SYS_JTD0_SWO_GPIO_Port GPIOB
#define ST_LINK_UART1_TX_Pin GPIO_PIN_6
#define ST_LINK_UART1_TX_GPIO_Port GPIOB
#define ST_LINK_UART1_RX_Pin GPIO_PIN_7
#define ST_LINK_UART1_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define max(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })

#define min(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b; })

/* ========== Sensor-Interrupt → Sensor-Tasks ========== */
#define IMU_SENSOR_THREAD_ACTIVATE_FLAG      (1U << 0)   // 0x01
#define MAG_SENSOR_THREAD_ACTIVATE_FLAG      (1U << 1)   // 0x02
#define BARO_SENSOR_THREAD_ACTIVATE_FLAG     (1U << 2)   // 0x04
#define HUMIDITY_SENSOR_THREAD_ACTIVATE_FLAG (1U << 3)   // 0x08
#define TOF_SENSOR_THREAD_ACTIVATE_FLAG      (1U << 4)   // 0x10

/* ========== Sensor-Tasks → Filter-Task ========== */
#define RAW_IMU_DATA_READY_FLAG              (1U << 5)   // 0x20
#define RAW_MAG_DATA_READY_FLAG              (1U << 6)   // 0x40

/* ========== Filter-Task → Transmit-Task ========== */
#define FILTERED_DATA_READY_FLAG             (1U << 7)   // 0x80

typedef struct {
	float	delimiter;
	// LSM6DSL Rohdaten
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;

	// LSM6DSL gefilterte Werte
	float acc_x_filtered;
	float acc_y_filtered;
	float acc_z_filtered;
	float gyro_x_filtered;
	float gyro_y_filtered;
	float gyro_z_filtered;

	// LIS3MDL Rohdaten
	float mag_x;
	float mag_y;
	float mag_z;

	// LIS3MDL gefilterte Werte
	float mag_x_filtered;
	float mag_y_filtered;
	float mag_z_filtered;
}__attribute__((packed)) TRANSMIT_DATA;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
