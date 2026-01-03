/*
 * LSM6DSL.c
 *
 *  Created on: May 26, 2021
 *      Author: MarkusKrug
 */

#include "lsm6dsl.h"
#include "main.h"
#include <limits.h>
#include <stdbool.h>
#include <string.h>
#include "cmsis_os2.h"

extern osSemaphoreId_t I2C2availableHandle;

/* variables used within this file as global ones */
static LSM6DSL_RAW_VALUES lsm6dsl_raw_values;
LSM6DSL_VALUES lsm6dsl_values;
static LSM6DSL_CTRL1 lsm6dsl_ctrl = {
    .ctrl1.reg_value = 0x54,  /* CTRL1_XL: ODR 104 Hz, FS ±16g */
    .ctrl2.reg_value = 0x5C,  /* CTRL2_G: ODR 104 Hz, FS ±2000°/s */
    .ctrl3.reg_value = 0x44,  /* BDU and autoincrement active, all other values set to defaults */
    .ctrl4.reg_value = 0x28,  /* DRDY_MASK = 1, all interrupts to pin int1, all other values set to defaults */
    .ctrl5.reg_value = 0x00,  /* all values set to defaults */
    .ctrl6.reg_value = 0x00,  /* all values set to defaults */
    .ctrl7.reg_value = 0x00,  /* all values set to defaults */
    .ctrl8.reg_value = 0x00,  /* all values set to defaults */
    .ctrl9.reg_value = 0x00,  /* all values set to defaults */
    .ctrl10.reg_value = 0x0   /* all values set to defaults */
};

static LSM6DSLINTCFG lsm6dsl_intcfg = {
    .intpulsectrl.reg_value = 0x80, /* reset the interrupt pin after 75usec */
    .int1ctrl.reg_value = 0x02,  /* raise interrupt when gyro has new data,
    								that assumes that with the same ODR for
    								acceleration and gyro we will have the data available at the
    								same point in time. This can be checked in the status register
    								of the sensor (STATUS_REG, 0x1E) */
    .int2ctrl.reg_value = 0x00}; /* no interrupts on int2 pin because this one is not connected on the board */

extern I2C_HandleTypeDef hi2c2;

/*
 * Initialize the LSM6DSL
 */
HAL_StatusTypeDef LSM6DSL_initialize(void) {
  HAL_StatusTypeDef HAL_status;
  uint8_t buffer;

  /* check if the sensor is reachable */
  HAL_status =
      HAL_I2C_Mem_Read_DMA(&hi2c2, (LSM6DSL_I2C_ADDR << 1), LSM6DSL_WHO_AM_I,
                           I2C_MEMADD_SIZE_8BIT, &buffer, sizeof(buffer));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;
  if (buffer != LSM6DSL_WHO_AM_I_VALUE)
    return HAL_ERROR;

  /* set the sensor configuration */
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (LSM6DSL_I2C_ADDR << 1), LSM6DSL_CTRL1_XL, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lsm6dsl_ctrl, sizeof(lsm6dsl_ctrl));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  /* read back the sensor configuration to do a basic check */
  memset(&lsm6dsl_ctrl, 0xFF, sizeof(lsm6dsl_ctrl));
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LSM6DSL_I2C_ADDR << 1), LSM6DSL_CTRL1_XL, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lsm6dsl_ctrl, sizeof(lsm6dsl_ctrl));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  /* set the interrupt behavior */
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (LSM6DSL_I2C_ADDR << 1), LSM6DSL_DRDY_PULSE_CFG_G, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lsm6dsl_intcfg, sizeof(lsm6dsl_intcfg));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  /* dummy read to make sure the DRDY signal is reset */
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LSM6DSL_I2C_ADDR << 1), LSM6DSL_OUT_TEMP_L, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lsm6dsl_raw_values, sizeof(lsm6dsl_raw_values));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  return HAL_status;
}

/**
 * @brief  EXTI line detection callback.
 * @param  None
 * @retval None
 */
HAL_StatusTypeDef LSM6DSL_data_ready(void)
{
  HAL_StatusTypeDef HAL_status;

  /* start reading the acc-gyro sensor value */
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LSM6DSL_I2C_ADDR << 1), LSM6DSL_OUT_TEMP_L, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lsm6dsl_raw_values, (sizeof(lsm6dsl_raw_values)));

  if (HAL_status != HAL_OK)
  {
    return HAL_status;
  }
  else
  {
	  while(hi2c2.State != HAL_I2C_STATE_READY);
  }
  /* release here to allow other tasks to access the I2C2 as early as possible */
  osSemaphoreRelease(I2C2availableHandle);
  // todo magic numbers entfernen
  lsm6dsl_values.temperature =
      lsm6dsl_raw_values.raw_temperature / 256.0 + 25.0;
  lsm6dsl_values.acc_x =
      lsm6dsl_raw_values.raw_acc_x * (16.0 / USHRT_MAX) * GRAVITY;
  lsm6dsl_values.acc_y =
      lsm6dsl_raw_values.raw_acc_y * (16.0 / USHRT_MAX) * GRAVITY;
  lsm6dsl_values.acc_z =
      lsm6dsl_raw_values.raw_acc_z * (16.0 / USHRT_MAX) * GRAVITY;

  lsm6dsl_values.gyro_x = lsm6dsl_raw_values.raw_gyro_x * (2000.0 / USHRT_MAX);
  lsm6dsl_values.gyro_y = lsm6dsl_raw_values.raw_gyro_y * (2000.0 / USHRT_MAX);
  lsm6dsl_values.gyro_z = lsm6dsl_raw_values.raw_gyro_z * (2000.0 / USHRT_MAX);

  return HAL_status;
}

LSM6DSL_VALUES LSM6DSL_get_values(void)
{
    return lsm6dsl_values;
}
