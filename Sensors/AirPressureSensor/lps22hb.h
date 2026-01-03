/*
 * LIS3MDL.h
 *
 *  Created on: May 26, 2021
 *      Author: MarkusKrug
 */

#ifndef INC_LPS22HB_H_
#define INC_LPS22HB_H_

#include "main.h"

/* the I2C Address defines are given as 7Bit value
 * according to the discovery kit user manual. To
 * be used they have to be shifted one bit to the left
 * and bit 0 defines if read ('0') or write ('1') accesss
 * is used */
// pressure   // Seite 29 um2153  0xba  /2
// falsch#define LPS22HB_I2C_ADDR 0x3D
#define LPS22HB_I2C_ADDR 0x5D //< -i2c2 P47 um2153

/* constant values of HTS221 register */
#define LPS22HB_WHO_AM_I_VALUE 0xB1

/* sensor internal register addresses */
#define LPS22HB_INTERRUPT_CFG 0x0B
#define LPS22HB_THS_P_L 0x0C
#define LPS22HB_THS_P_H 0x0D

#define LPS22HB_WHO_AM_I 0x0F
#define LPS22HB_CTRL_REG1 0x10
#define LPS22HB_CTRL_REG2 0x11
#define LPS22HB_CTRL_REG3 0x12

#define LPS22HB_FIFO_CTRL 0x14
#define LPS22HB_REF_P_XL 0x15
#define LPS22HB_REF_P_L 0x16
#define LPS22HB_REF_P_H 0x17
#define LPS22HB_RPDS_L 0x18

#define LPS22HB_RPDS_H 0x19
#define LPS22HB_RES_CONF 0x1A

#define LPS22HB_INT_SOURCE 0x25
#define LPS22HB_FIFO_STATUS 0x26
#define LPS22HB_STATUS 0x27
#define LPS22HB_PRESS_OUT_XL 0x28
#define LPS22HB_PRESS_OUT_L 0x29
#define LPS22HB_PRESS_OUT_H 0x2A
#define LPS22HB_TEMP_OUT_L 0x2B
#define LPS22HB_TEMP_OUT_H 0x2C

#define LPS22HB_LPFP_RES 0x33

/* typedefs for the LPS22HB */
typedef union {
  struct {
    uint8_t SIM : 1;
    uint8_t BDU : 1;
    uint8_t LPFP_CFG : 1;
    uint8_t EN_LPFP : 1;
    uint8_t ODR : 3;
    uint8_t res : 1;
  } bits;
  uint8_t reg_value;
} LPS22HB_CTRLREG1;

typedef union {
  struct {
    uint8_t ONE_SHOT : 1;
    uint8_t res : 1;
    uint8_t SWRESET : 1;
    uint8_t I2C_DIS : 1;
    uint8_t IF_ADD_INC : 1;
    uint8_t STOP_ON_FTH : 1;
    uint8_t FIFO_EN : 1;
    uint8_t BOOT : 1;
  } bits;
  uint8_t reg_value;
} LPS22HB_CTRLREG2;

typedef union {
  struct {
    uint8_t INT_S1 : 2;
    uint8_t DRDY : 1;
    uint8_t F_OVR : 1;
    uint8_t F_FTH : 1;
    uint8_t F_FSS5 : 1;
    uint8_t PP_OD : 1;
    uint8_t INT_H_L : 1;
  } bits;
  uint8_t reg_value;
} LPS22HB_CTRLREG3;

typedef union {
  struct {
    uint8_t WTM : 5;
    uint8_t F_MODE : 3;
  } bits;
  uint8_t reg_value;
} LPS22HB_FIFOCTRL;

typedef struct {
  LPS22HB_CTRLREG1 ctrl1;
  LPS22HB_CTRLREG2 ctrl2;
  LPS22HB_CTRLREG3 ctrl3;
  LPS22HB_FIFOCTRL fifoctrl;
} __attribute__((packed)) LPS22HB_CTRL;

typedef struct {
  uint8_t int_source;
  uint8_t fifo_status;
  uint8_t status;
  uint8_t raw_p_t_data[5];
} __attribute__((packed)) LPS22HB_RAW_VALUES;

typedef struct {
  float pressure;
  float temperature;
} LPS22HB_VALUES;

/* function prototypes */
HAL_StatusTypeDef LPS22HB_initialize(void);
HAL_StatusTypeDef LPS22HB_data_ready(void);

#endif /* INC_LPS22HB_H_ */
