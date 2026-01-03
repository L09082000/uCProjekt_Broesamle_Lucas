/*
 * LIS3MDL.c
 *
 *  Created on: May 26, 2021
 *      Author: MarkusKrug
 */
#include <stdbool.h>
#include <string.h>

#include "lis3mdl.h"
#include "main.h"

#include "cmsis_os2.h"

extern osSemaphoreId_t I2C2availableHandle;

/* variables used within this file as global ones */
static LIS3MDL_RAW_VALUES lis3mdl_raw_values;
LIS3MDL_VALUES lis3mdl_values;
static LIS3MDL_CTRL lis3mdl_ctrl = {
    .ctrl1.reg_value = 0xF4,  /* temperature sensor enabled, ultra high performance mode for x/y axis, ODR=80 Hz, all other values set to defaults */
    .ctrl2.reg_value = 0x11,  /* all values set to default, FS = +/- 16gauss */
    .ctrl3.reg_value = 0x00,  /* continuous conversion, all other values set to defaults */
    .ctrl4.reg_value = 0x0C,  /* ultra high performance mode for z axis, all other values set to defaults */
    .ctrl5.reg_value = 0x40   /* BDU set on, all other values set to defaults */
};

/* note interrupt behavior is configured but not uses because the corresponding
 * pin of the sensor is not connected to the µC on the discovery board that is
 * used */
static LIS3MDLINTCFG lis3mdl_intcfg = {
    .intcfg.reg_value =
        0x27, /* interrupt when axis raise an interrupt, interrupt enabled with
                 high active and latched interrupt pin */
    .intsrc.reg_value = 0x00, /* read only register */
    .intths.reg_value = 0x6AE7}; /* set the threshold to just one bit below the maximum */

extern I2C_HandleTypeDef hi2c2;

/*
 * Initialize the LIS3MDL
 */
HAL_StatusTypeDef LIS3MDL_initialize(void) {
  HAL_StatusTypeDef HAL_status;
  uint8_t buffer;

  /* check if the sensor is reachable */
  HAL_status =
      HAL_I2C_Mem_Read_DMA(&hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_WHO_AM_I,
                           I2C_MEMADD_SIZE_8BIT, &buffer, sizeof(buffer));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;
  if (buffer != LIS3MDL_WHO_AM_I_VALUE)
    return HAL_ERROR;

  /* set the sensor configuration */
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_CTRL_REG1, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_ctrl, sizeof(lis3mdl_ctrl));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  /* read back the sensor configuration to do a basic check */
  memset(&lis3mdl_ctrl, 0xFF, sizeof(lis3mdl_ctrl));
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_CTRL_REG1, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_ctrl, sizeof(lis3mdl_ctrl));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  /* set the interrupt behavior */
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_INT_CFG, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_intcfg.intcfg, sizeof(lis3mdl_intcfg.intcfg));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_INT_THS_L, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_intcfg.intths, sizeof(lis3mdl_intcfg.intths));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  /* dummy read to make sure the DRDY signal is reset */
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_OUT_X_L, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_raw_values.raw_x, sizeof(int16_t));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  return HAL_status;
}

HAL_StatusTypeDef LIS3MDL_data_ready(void)
{
  HAL_StatusTypeDef HAL_status;

  /* start reading the magnetometer sensor value */
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_OUT_X_L, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_raw_values.raw_x, (3 * sizeof(int16_t)));


  if (HAL_status != HAL_OK)
  {
    return HAL_status;
  }
  else
  {
	  while(hi2c2.State != HAL_I2C_STATE_READY);
  }

  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_TEMP_OUT_L, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_raw_values.raw_temperature, sizeof(int16_t));
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }
  if (HAL_status != HAL_OK)
  {
    return HAL_status;
  }

  /* release here to allow other tasks to access the I2C2 as early as possible */
  osSemaphoreRelease(I2C2availableHandle);

  /*Converting the raw sensor values into physical units (Gauss) correctly. */
  lis3mdl_values.temperature = lis3mdl_raw_values.raw_temperature / 8.0 + 25.0;
  lis3mdl_values.x = lis3mdl_raw_values.raw_x / 1711.0;
  lis3mdl_values.y = lis3mdl_raw_values.raw_y / 1711.0;
  lis3mdl_values.z = lis3mdl_raw_values.raw_z / 1711.0;

  return HAL_status;
}

LIS3MDL_VALUES LIS3MDL_get_values(void)
{
    return lis3mdl_values;
}

