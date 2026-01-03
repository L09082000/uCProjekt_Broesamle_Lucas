/*
 * HTS221.h
 *
 *  Created on: May 25, 2021
 *      Author: MarkusKrug
 */

#ifndef INC_HTS221_H_
#define INC_HTS221_H_

#include "main.h"

/* the I2C Address defines are given as 7Bit value
 * according to the discovery kit user manual. To
 * be used they have to be shifted one bit to the left
 * and bit 0 defines if read ('0') or write ('1') accesss
 * is used */

// temp humidity
#define HTS221_I2C_ADDR 0x5F // Seite 29 um2153  //BE/2  <- i2c2 P47 um2153

/* constant values of HTS221 register */
#define HTS221_WHO_AM_I_VALUE 0xBC

/* sensor internal register addresses */
/* Note: to use the auto-increment all upper
 * nibbles of the register address has an offset
 * of 8 (see HTS221 datasheet page 15 */
#define HTS221_WHO_AM_I 0x8F
#define HTS221_AV_CONF 0x90
#define HTS221_CTRL_REG1 0xA0
#define HTS221_CTRL_REG2 0xA1
#define HTS221_CTRL_REG3 0xA2
#define HTS221_HUMIDITY_OUT_L 0xA8
#define HTS221_HUMIDITY_OUT_H 0xA9
#define HTS221_TEMP_OUT_L 0xAA
#define HTS221_TEMP_OUT_H 0xAB

#define HTS221_H0_rH_x2_REG 0xB0
#define HTS221_H1_rH_x2_REG 0xB1
#define HTS221_T0_degC_x8_REG 0xB2
#define HTS221_T1_degC_x8_REG 0xB3
#define HTS221_T1T0_msb_REG 0xB5

#define HTS221_H0_T0_OUT_REG 0xB6
#define HTS221_H1_T0_OUT_REG 0xBA
#define HTS221_T0_OUT_REG 0xBC
#define HTS221_T1_OUT_REG 0xBE

/* typedefs for the HTS221 */
typedef union {
  struct {
    uint8_t ODR : 2;
    uint8_t BDU : 1;
    uint8_t res : 4;
    uint8_t PD : 1;
  } bits;
  uint8_t reg_value;
} HTS221_CTRL1_REG;

typedef union {
  struct {
    uint8_t ONE_SHOT : 1;
    uint8_t heater : 1;
    uint8_t res : 5;
    uint8_t BOOT : 1;
  } bits;
  uint8_t reg_value;
} HTS221_CTRL2_REG;

typedef union {
  struct {
    uint8_t res1 : 2;
    uint8_t DRDY : 1;
    uint8_t res2 : 3;
    uint8_t PP_OD : 1;
    uint8_t DRDY_H_L : 1;
  } bits;
  uint8_t reg_value;
} HTS221_CTRL3_REG;

typedef struct {
  HTS221_CTRL1_REG ctrl1;
  HTS221_CTRL2_REG ctrl2;
  HTS221_CTRL3_REG ctrl3;
} __attribute__((packed)) HTS221_CTRL;

typedef struct {
  uint8_t H0_rH;
  uint8_t H1_rH;
  int16_t H0_T0_OUT;
  int16_t H1_T0_OUT;
  uint8_t T0_degC;
  uint8_t T1_degC;
  int16_t T0_OUT;
  int16_t T1_OUT;
} HTS221_CAL_VALUES;

typedef struct {
  int16_t raw_humidity;
  int16_t raw_temperature;
  float humidity;
  float temperature;
} __attribute__((packed)) HTS221_VALUES;

/* function prototypes */
float HTS221_Get_Humidity(void);
float HTS221_Get_Temperature(void);
HAL_StatusTypeDef HTS221_Get_Calibration_Values(HTS221_CAL_VALUES *);
HAL_StatusTypeDef HTS221_initialize(void);
HAL_StatusTypeDef HTS221_data_ready();

#endif /* INC_HTS221_H_ */
