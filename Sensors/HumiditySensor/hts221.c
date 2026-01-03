/*
 * HTS221.c
 *
 *  Created on: May 25, 2021
 *      Author: MarkusKrug
 */

#include <stdbool.h>
#include <string.h>

#include "hts221.h"
#include "main.h"
#include "cmsis_os2.h"

/* variables used within this file as global ones */
static HTS221_CAL_VALUES hts221_cal_values;
static HTS221_VALUES hts221_values;
static HTS221_CTRL hts221_ctrl = {
		.ctrl1.reg_value = 0x81,  /* ODR = 1 Hz (01), BDU enabled, power on, rest default */
		.ctrl2.reg_value = 0x00,  /* enable autoincrement for I2C addresses, defaults */
		.ctrl3.reg_value = 0x04   /* DRDY interrupt enabled */
};

extern I2C_HandleTypeDef hi2c2;
extern osSemaphoreId_t I2C2availableHandle;

/*
 * Initialize the HTS221
 */
HAL_StatusTypeDef HTS221_initialize(void) {
  volatile HAL_StatusTypeDef HAL_status;
  uint8_t buffer;

  /* check if the sensor is reachable */
  HAL_status =
      HAL_I2C_Mem_Read_DMA(&hi2c2, (HTS221_I2C_ADDR << 1), HTS221_WHO_AM_I,
                           I2C_MEMADD_SIZE_8BIT, &buffer, sizeof(buffer));
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }
  if (buffer != HTS221_WHO_AM_I_VALUE)
    return HAL_ERROR;

  /* set the sensor configuration */
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (HTS221_I2C_ADDR << 1), HTS221_CTRL_REG1, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&hts221_ctrl, sizeof(hts221_ctrl));
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }

  /* read back the sensor configuration to do a basic check */
  memset(&hts221_ctrl, 0xFF, sizeof(hts221_ctrl));
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (HTS221_I2C_ADDR << 1), HTS221_CTRL_REG1, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&hts221_ctrl, sizeof(hts221_ctrl));
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }

  HAL_status = HTS221_Get_Calibration_Values(&hts221_cal_values);

  /* do a dummy read to reset the DRDY and Status signal */
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (HTS221_I2C_ADDR << 1), HTS221_HUMIDITY_OUT_L,
      I2C_MEMADD_SIZE_8BIT, (uint8_t *)&hts221_values, (2 * sizeof(uint16_t)));
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }
  return HAL_status;
}

/*
 * Read the HTS221 calibration values. These values are used for
 * calculating the relative humidity and temperature
 */
HAL_StatusTypeDef HTS221_Get_Calibration_Values(HTS221_CAL_VALUES *cal_values) {
  uint8_t buffer[13];
  HAL_StatusTypeDef HAL_status;
  uint16_t temp;

  HAL_status =
      HAL_I2C_Mem_Read_DMA(&hi2c2, (HTS221_I2C_ADDR << 1), HTS221_H0_rH_x2_REG,
                           I2C_MEMADD_SIZE_8BIT, buffer, 4);
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }
  HAL_status =
      HAL_I2C_Mem_Read_DMA(&hi2c2, (HTS221_I2C_ADDR << 1), HTS221_T1T0_msb_REG,
                           I2C_MEMADD_SIZE_8BIT, &buffer[4], 3);
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }
  HAL_status =
      HAL_I2C_Mem_Read_DMA(&hi2c2, (HTS221_I2C_ADDR << 1), HTS221_H1_T0_OUT_REG,
                           I2C_MEMADD_SIZE_8BIT, &buffer[7], 6);
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }
  cal_values->H0_rH = buffer[0] >> 1;
  cal_values->H1_rH = buffer[1] >> 1;
  cal_values->T0_degC = buffer[2];
  cal_values->T1_degC = buffer[3];
  temp = (cal_values->T0_degC) | ((buffer[4] & 0x03) << 8);
  cal_values->T0_degC = temp >> 3;
  temp = (cal_values->T1_degC) | ((buffer[4] & 0x0C) << 6);
  cal_values->T1_degC = temp >> 3;
  memcpy(&cal_values->H0_T0_OUT, &buffer[5], sizeof(int16_t));
  memcpy(&cal_values->H1_T0_OUT, &buffer[7], sizeof(int16_t));
  memcpy(&cal_values->T0_OUT, &buffer[9], sizeof(int16_t));
  memcpy(&cal_values->T1_OUT, &buffer[11], sizeof(int16_t));
  return HAL_status;
}

/*
 * Read the HTS221 relative humidity raw sensor value and calculate
 * the relative humidity by using the calibration values stored in the
 * sensor
 */
float HTS221_Get_Humidity(void) {
  float value;
  int tmp;

  tmp = ((int32_t)((int32_t)hts221_values.raw_humidity -
                   (int32_t)hts221_cal_values.H0_T0_OUT)) *
        ((int32_t)((int32_t)(hts221_cal_values.H1_rH -
                             (int32_t)hts221_cal_values.H0_rH)));
  value = (tmp / ((int32_t)((int32_t)hts221_cal_values.H1_T0_OUT) +
                  (int32_t)hts221_cal_values.H0_T0_OUT) +
           ((int32_t)((int32_t)hts221_cal_values.H0_rH)));

  if (value > 1000)
    return value = 1000;
  else
    return value;
}

/*
 * Read the HTS221 relative temperature raw sensor value and calculate
 * the temperature by using the calibration values stored in the
 * sensor
 */
float HTS221_Get_Temperature(void) {
  float value;

  /* Compute the temperature value by linear interpolation */
  value = (float)(hts221_values.raw_temperature - hts221_cal_values.T0_OUT) *
              (float)(hts221_cal_values.T1_degC - hts221_cal_values.T0_degC) /
              (float)(hts221_cal_values.T1_OUT - hts221_cal_values.T0_OUT) +
          hts221_cal_values.T0_degC;

  return value;
}

HAL_StatusTypeDef HTS221_data_ready(void)
{
  HAL_StatusTypeDef HAL_status;
  /* start reading the humidity sensor value */
  /* this needs to be done in a do-while loop because the sensor
   * is not responding anymore if previous data has not been read */
  do
  {
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (HTS221_I2C_ADDR << 1), HTS221_HUMIDITY_OUT_L,
      I2C_MEMADD_SIZE_8BIT, (uint8_t *)&hts221_values, (2 * sizeof(int16_t)));
  }while(HAL_status != HAL_OK);


  while(hi2c2.State != HAL_I2C_STATE_READY);

  /* release here to allow other tasks to access the I2C2 as early as possible */
  osSemaphoreRelease(I2C2availableHandle);


  hts221_values.humidity = HTS221_Get_Humidity();
  hts221_values.temperature = HTS221_Get_Temperature();

  return HAL_status;
}
