/*
 * LIS3MDL.h
 *
 *  Created on: May 26, 2021
 *      Author: MarkusKrug
 */

#ifndef INC_LIS3MDL_H_
#define INC_LIS3MDL_H_

#include "main.h"

/* the I2C Address defines are given as 7Bit value
 * according to the discovery kit user manual. To
 * be used they have to be shifted one bit to the left
 * and bit 0 defines if read ('0') or write ('1') accesss
 * is used */

// magnetometer
#define LIS3MDL_I2C_ADDR 0x1E //<- i2c2 P47 um2153

/* constant values of HTS221 register */
#define LIS3MDL_WHO_AM_I_VALUE 0x3D

/* sensor internal register addresses */
/* Note: to use the auto-increment all upper
 * nibbles of the register address has an offset
 * of 8 (see LIS3MDL datasheet page 17 */
#define LIS3MDL_OFFSET_X_REG_L_M 0x85
#define LIS3MDL_OFFSET_X_REG_H_M 0x86
#define LIS3MDL_OFFSET_Y_REG_L_M 0x87
#define LIS3MDL_OFFSET_Y_REG_H_M 0x88
#define LIS3MDL_OFFSET_Z_REG_L_M 0x89
#define LIS3MDL_OFFSET_Z_REG_H_M 0x8A

#define LIS3MDL_WHO_AM_I 0x8F
#define LIS3MDL_CTRL_REG1 0xA0
#define LIS3MDL_CTRL_REG2 0xA1
#define LIS3MDL_CTRL_REG3 0xA2
#define LIS3MDL_CTRL_REG4 0xA3
#define LIS3MDL_CTRL_REG5 0xA4

#define LIS3MDL_STATUS_REG 0xA7
#define LIS3MDL_OUT_X_L 0xA8
#define LIS3MDL_OUT_X_H 0xA9
#define LIS3MDL_OUT_Y_L 0xAA
#define LIS3MDL_OUT_Y_H 0xAB
#define LIS3MDL_OUT_Z_L 0xAC
#define LIS3MDL_OUT_Z_H 0xAD
#define LIS3MDL_TEMP_OUT_L 0xAE
#define LIS3MDL_TEMP_OUT_H 0xAF
#define LIS3MDL_INT_CFG 0xB0
#define LIS3MDL_INT_SRC 0xB1
#define LIS3MDL_INT_THS_L 0xB2
#define LIS3MDL_INT_THS_H 0xB3

/* typedefs for the LIS3MDL */
typedef union {
  struct {
    uint8_t ST : 1;
    uint8_t FAST_ODR : 1;
    uint8_t DO : 3;
    uint8_t OM : 2;
    uint8_t TEMP_EN : 1;
  } bits;
  uint8_t reg_value;
} LIS3MDL_CTRL1_REG;

typedef union {
  struct {
    uint8_t res1 : 2;
    uint8_t SOFT_RST : 1;
    uint8_t REBOOT : 1;
    uint8_t res2 : 1;
    uint8_t FS : 2;
    uint8_t res3 : 1;
  } bits;
  uint8_t reg_value;
} LIS3MDL_CTRL2_REG;

typedef union {
  struct {
    uint8_t MD : 2;
    uint8_t SIM : 1;
    uint8_t res1 : 2;
    uint8_t LP : 1;
    uint8_t res2 : 2;
  } bits;
  uint8_t reg_value;
} LIS3MDL_CTRL3_REG;

typedef union {
  struct {
    uint8_t res1 : 1;
    uint8_t BLE : 1;
    uint8_t OMZ : 2;
    uint8_t res2 : 4;
  } bits;
  uint8_t reg_value;
} LIS3MDL_CTRL4_REG;

typedef union {
  struct {
    uint8_t res1 : 6;
    uint8_t BDU : 1;
    uint8_t FAST_READ : 1;
  } bits;
  uint8_t reg_value;
} LIS3MDL_CTRL5_REG;

typedef union {
  struct {
    uint8_t XDA : 1;
    uint8_t YDA : 1;
    uint8_t ZDA : 1;
    uint8_t ZYXDA : 1;
    uint8_t XOR : 1;
    uint8_t YOR : 1;
    uint8_t ZOR : 1;
    uint8_t ZYXOR : 1;
  } bits;
  uint8_t reg_value;
} LIS3MDL_STATUSREG;

typedef union {
  struct {
    uint8_t IEN : 1;
    uint8_t LIR : 1;
    uint8_t IEA : 1;
    uint8_t res : 2;
    uint8_t ZIEN : 1;
    uint8_t YIEN : 1;
    uint8_t XIEN : 1;
  } bits;
  uint8_t reg_value;
} LIS3MDL_INTCFG;

typedef union {
  struct {
    uint8_t INT : 1;
    uint8_t MROI : 1;
    uint8_t NTH_Z : 1;
    uint8_t NTH_Y : 1;
    uint8_t NTH_X : 1;
    uint8_t PTH_Z : 1;
    uint8_t PTH_Y : 1;
    uint8_t PTH_X : 1;
  } bits;
  uint8_t reg_value;
} LIS3MDL_INTSRC;

typedef union {
  struct {
    uint16_t THS : 15;
    uint16_t res : 1;
  } bits;
  uint16_t reg_value;
} LIS3MDL_INTTHS;

typedef struct {
  LIS3MDL_CTRL1_REG ctrl1;
  LIS3MDL_CTRL2_REG ctrl2;
  LIS3MDL_CTRL2_REG ctrl3;
  LIS3MDL_CTRL4_REG ctrl4;
  LIS3MDL_CTRL5_REG ctrl5;
} __attribute__((packed)) LIS3MDL_CTRL;

typedef struct {
  LIS3MDL_INTCFG intcfg;
  LIS3MDL_INTSRC intsrc;
  LIS3MDL_INTTHS intths;
} __attribute__((packed)) LIS3MDLINTCFG;

typedef struct {
  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;
  int16_t raw_temperature;
} __attribute__((packed)) LIS3MDL_RAW_VALUES;

typedef struct {
  float x;
  float y;
  float z;
  float temperature;
} LIS3MDL_VALUES;

/* function prototypes */
HAL_StatusTypeDef LIS3MDL_initialize(void);
HAL_StatusTypeDef LIS3MDL_data_ready(void);
LIS3MDL_VALUES LIS3MDL_get_values(void);
#endif /* INC_LIS3MDL_H_ */
