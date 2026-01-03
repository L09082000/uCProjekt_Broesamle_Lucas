/*
 * LIS3MDL.c
 *
 *  Created on: May 26, 2021
 *      Author: MarkusKrug
 */

#include <limits.h>
#include <stdbool.h>
#include <string.h>

#include "lps22hb.h"
#include "main.h"

#include "cmsis_os2.h"

extern osSemaphoreId_t I2C2availableHandle;

/* variables used within this file as global ones */

static LPS22HB_RAW_VALUES lps22hb_raw_values;
static LPS22HB_VALUES lps22hb_values;
static LPS22HB_CTRL lps22hb_ctrl = {
    .ctrl1.reg_value = 0x12,  /* ODR=1Hz, BDU enabled, all other values set to defaults */
    .ctrl2.reg_value = 0x10,  /* enable autoincrement for I2C adresses, all values set to default */
    .ctrl3.reg_value = 0x04,  /* DRDY interrupt enabled, all other values set to defaults */
    .fifoctrl.reg_value = 0x0 /* all values set to defaults, FIFO bypass mode */
};

/* note interrupt behavior is configured but not uses because the corresponding
 * pin of the sensor is not connected to the µC on the discovery board that is
 * used */
static uint8_t lps22hb_intcfg =
    0x08; /* enable interrupts */

extern I2C_HandleTypeDef hi2c2;

/*
 * Initialize the LPS22HB
 */
HAL_StatusTypeDef LPS22HB_initialize(void) {
  volatile HAL_StatusTypeDef HAL_status;

  uint8_t buffer = 0;

  /* check if the sensor is reachable */
  HAL_status =
      HAL_I2C_Mem_Read_DMA(&hi2c2, (LPS22HB_I2C_ADDR << 1), LPS22HB_WHO_AM_I,
                           I2C_MEMADD_SIZE_8BIT, &buffer, sizeof(buffer));
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }
  if (buffer != LPS22HB_WHO_AM_I_VALUE) {
    return HAL_ERROR;
  }

  /* set the sensor configuration */
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (LPS22HB_I2C_ADDR << 1), LPS22HB_CTRL_REG2, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lps22hb_ctrl.ctrl2, sizeof(uint8_t));
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (LPS22HB_I2C_ADDR << 1), LPS22HB_CTRL_REG1, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lps22hb_ctrl, sizeof(lps22hb_ctrl));
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }

  //	/* read back the sensor configuration to do a basic check */
  //	memset(&lps22hb_ctrl,0xFF,sizeof(lps22hb_ctrl));
  //	HAL_status = HAL_I2C_Mem_Read_DMA(&hi2c2, (LPS22HB_I2C_ADDR<<1),
  // LPS22HB_CTRL_REG1, 			I2C_MEMADD_SIZE_8BIT, (uint8_t
  // *)&lps22hb_ctrl, sizeof(lps22hb_ctrl)); 	while(hi2c2.State !=
  // HAL_I2C_STATE_READY);

  /* set the interrupt behavior */
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (LPS22HB_I2C_ADDR << 1), LPS22HB_INTERRUPT_CFG,
      I2C_MEMADD_SIZE_8BIT, (uint8_t *)&lps22hb_intcfg, sizeof(lps22hb_intcfg));
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }

  /* dummy read to make sure the DRDY signal is reset */
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LPS22HB_I2C_ADDR << 1), LPS22HB_INT_SOURCE, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lps22hb_raw_values, sizeof(lps22hb_raw_values));
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }

  return HAL_status;
}

HAL_StatusTypeDef LPS22HB_data_ready(void)
{

  HAL_StatusTypeDef HAL_status;

  /* start reading the humidity sensor value */
  /* this needs to be done in a do-while loop because the sensor
   * is not responding anymore if previous data has not been read */
  do
  {
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LPS22HB_I2C_ADDR << 1), LPS22HB_INT_SOURCE, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lps22hb_raw_values, sizeof(lps22hb_raw_values));
  } while(HAL_status != HAL_OK);

  while(hi2c2.State != HAL_I2C_STATE_READY);


  /* release here to allow other tasks to access the I2C2 as early as possible */
  osSemaphoreRelease(I2C2availableHandle);

  /* Convert raw pressure to hPa */
  if (lps22hb_raw_values.raw_p_t_data[2] > SCHAR_MAX) {
    lps22hb_values.pressure =
        -1.0 *
        (float)((uint32_t)(lps22hb_raw_values.raw_p_t_data[0] |
                           (lps22hb_raw_values.raw_p_t_data[1] << 8) |
                           (lps22hb_raw_values.raw_p_t_data[2] << 16))) /
        4096;
  } else {
    lps22hb_values.pressure =
        (float)((uint32_t)(lps22hb_raw_values.raw_p_t_data[0] |
                           (lps22hb_raw_values.raw_p_t_data[1] << 8) |
                           (lps22hb_raw_values.raw_p_t_data[2] << 16))) /
        4096;
  }

  lps22hb_values.temperature =
      ((int16_t)((lps22hb_raw_values.raw_p_t_data[3]) |
                 (lps22hb_raw_values.raw_p_t_data[4] << 8))) /
      100.0;

  return HAL_status;
}
