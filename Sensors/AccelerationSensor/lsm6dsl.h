/*
 * LIS3MDL.h
 *
 *  Created on: May 26, 2021
 *      Author: MarkusKrug
 */

#ifndef INC_LSM6DSL_H_
#define INC_LSM6DSL_H_

#include "main.h"

/* the I2C Address defines are given as 7Bit value
 * according to the discovery kit user manual. To
 * be used they have to be shifted one bit to the left
 * and bit 0 defines if read ('0') or write ('1') accesss
 * is used */

// acceleration
#define LSM6DSL_I2C_ADDR 0x6A // Seite 29 um2153 <- i2c2 P47 um2153

#define GRAVITY 9.81
/* constant values of LSM6DSL register */
#define LSM6DSL_WHO_AM_I_VALUE 0x6A

/* sensor internal register addresses */
#define LSM6DSL_FUNC_CFG_ACCESS 0x01
#define LSM6DSL_SENSOR_SYNC_TIME_FRAME 0x04
#define LSM6DSL_SENSOR_SYNC_RES_RATIO 0x05
#define LSM6DSL_FIFO_CTRL1 0x06
#define LSM6DSL_FIFO_CTRL2 0x07
#define LSM6DSL_FIFO_CTRL3 0x08
#define LSM6DSL_FIFO_CTRL4 0x09
#define LSM6DSL_FIFO_CTRL5 0x0A
#define LSM6DSL_DRDY_PULSE_CFG_G 0x0B

#define LSM6DSL_INT1_CTRL 0x0D
#define LSM6DSL_INT2_CTRL 0x0E
#define LSM6DSL_WHO_AM_I 0x0F
#define LSM6DSL_CTRL1_XL 0x10
#define LSM6DSL_CTRL2_G 0x11
#define LSM6DSL_CTRL3_C 0x12
#define LSM6DSL_CTRL4_C 0x13
#define LSM6DSL_CTRL5_C 0x14
#define LSM6DSL_CTRL6_C 0x15
#define LSM6DSL_CTRL7_G 0x16
#define LSM6DSL_CTRL8_XL 0x17
#define LSM6DSL_CTRL9_XL 0x18
#define LSM6DSL_CTRL10_C 0x19
#define LSM6DSL_MASTER_CONFIG 0x1A

#define LSM6DSL_WAKE_UP_SRC 0x1B
#define LSM6DSL_TAP_SRC 0x1C
#define LSM6DSL_D6D_SRC 0x1D
#define LSM6DSL_STATUS_REG 0x1E

#define LSM6DSL_OUT_TEMP_L 0x20
#define LSM6DSL_OUT_TEMP_H 0x21
#define LSM6DSL_OUTX_L_G 0x22
#define LSM6DSL_OUTX_H_G 0x23
#define LSM6DSL_OUTY_L_G 0x24
#define LSM6DSL_OUTY_H_G 0x25
#define LSM6DSL_OUTZ_L_G 0x26
#define LSM6DSL_OUTZ_H_G 0x27
#define LSM6DSL_OUTX_L_XL 0x28
#define LSM6DSL_OUTX_H_XL 0x29
#define LSM6DSL_OUTY_L_XL 0x2A
#define LSM6DSL_OUTY_H_XL 0x2B
#define LSM6DSL_OUTZ_L_XL 0x2C
#define LSM6DSL_OUTZ_H_XL 0x2D

#define LSM6DSL_FIFO_STATUS1 0x3A
#define LSM6DSL_FIFO_STATUS2 0x3B
#define LSM6DSL_FIFO_STATUS3 0x3C
#define LSM6DSL_FIFO_STATUS4 0x3D
#define LSM6DSL_FIFO_DATA_OUT_L 0x3E
#define LSM6DSL_FIFO_DATA_OUT_H 0x3F
#define LSM6DSL_TIMESTAMP0_REG 0x40
#define LSM6DSL_TIMESTAMP1_REG 0x41
#define LSM6DSL_TIMESTAMP2_REG 0x42

#define LSM6DSL_STEP_TIMESTAMP_L 0x49
#define LSM6DSL_STEP_TIMESTAMP_H 0x4A
#define LSM6DSL_STEP_COUNTER_L 0x4B
#define LSM6DSL_STEP_COUNTER_H 0x4C
#define LSM6DSL_FUNC_SRC1 0x53
#define LSM6DSL_FUNC_SRC2 0x54
#define LSM6DSL_WRIST_TILT_IA 0x55

#define LSM6DSL_TAP_CFG 0x58
#define LSM6DSL_TAP_THS_6D 0x59
#define LSM6DSL_INT_DUR2 0x5A
#define LSM6DSL_WAKE_UP_THS 0x5B
#define LSM6DSL_WAKE_UP_DUR 0x5C
#define LSM6DSL_FREE_FALL 0x5D
#define LSM6DSL_MD1_CFG 0x5E
#define LSM6DSL_MD2_CFG 0x5F

#define LSM6DSL_X_OFS_USR 0x73
#define LSM6DSL_Y_OFS_USR 0x74
#define LSM6DSL_Z_OFS_USR 0x75

#define LSM6DSL_CONFIG_PEDO_THS_MIN 0x0F
#define LSM6DSL_SM_THS 0x13
#define LSM6DSL_PEDO_DEB_REG 0x14
#define LSM6DSL_STEP_COUNT_DELTA 0x15

#define LSM6DSL_A_WRIST_TILT_LAT 0x50
#define LSM6DSL_A_WRIST_TILT_THS 0x54
#define LSM6DSL_A_WRIST_TILT_Mask 0x59

/* typedefs for the LSM6DSL */
typedef union {
  struct {
    uint8_t FTH0_7 : 8;
  } bits;
  uint8_t reg_value;
} LSM6DSL_FIFOCTRL1;

typedef union {
  struct {
    uint8_t FTH8_10 : 3;
    uint8_t FIFO_TMP_EN : 1;
    uint8_t res : 2;
    uint8_t TIMER_PEDO_FIFO_DRDY : 1;
    uint8_t TIMER_PEDO_FIFO_EN : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_FIFOCTRL2;

typedef union {
  struct {
    uint8_t DEC_FIFO_XL : 3;
    uint8_t DEC_FIFO_GYRO : 3;
    uint8_t res : 2;
  } bits;
  uint8_t reg_value;
} LSM6DSL_FIFOCTRL3;

typedef union {
  struct {
    uint8_t DEC_DS3_FIFO : 3;
    uint8_t DEC_DS4_FIFO : 3;
    uint8_t ONLY_HIGH_DATA : 1;
    uint8_t STOP_ON_FTH : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_FIFOCTRL4;

typedef union {
  struct {
    uint8_t FIFO_MODE : 3;
    uint8_t ODR_FIFO : 4;
    uint8_t res : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_FIFOCTRL5;

typedef union {
  struct {
    uint8_t INT2_WRIST_TILT : 1;
    uint8_t res : 6;
    uint8_t DRDY_PULSED : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_DRDY_PULSE_CFGG;

typedef union {
  struct {
    uint8_t INT1_DRDY_XL : 1;
    uint8_t INT1_DRDY_G : 1;
    uint8_t INT1_BOOT : 1;
    uint8_t INT1_FTH : 1;
    uint8_t INT1_FIFO_OVR : 1;
    uint8_t INT1_FULL_FLAG : 1;
    uint8_t INT1_SIGN_MOT : 1;
    uint8_t INT1_STEP_DETECTOR : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_INT1CTRL;

typedef union {
  struct {
    uint8_t INT2_DRDY_XL : 1;
    uint8_t INT2_DRDY_G : 1;
    uint8_t INT2_DRDY_TEMP : 1;
    uint8_t INT2_FTH : 1;
    uint8_t INT2_FIFO_OVR : 1;
    uint8_t INT2_FULL_FLAG : 1;
    uint8_t INT2_STEP_COUNT_OV : 1;
    uint8_t INT2_STEP_DELTA : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_INT2CTRL;

typedef union {
  struct {
    uint8_t BW0_XL : 1;
    uint8_t LPF1_BW_SEL : 1;
    uint8_t FS_XL : 2;
    uint8_t ODR_XL : 4;
  } bits;
  uint8_t reg_value;
} LSM6DSL_CTRL1XL;

typedef union {
  struct {
    uint8_t res : 1;
    uint8_t FS_125 : 1;
    uint8_t FS_G : 2;
    uint8_t ODR_G : 4;
  } bits;
  uint8_t reg_value;
} LSM6DSL_CTRL2G;

typedef union {
  struct {
    uint8_t SW_RESET : 1;
    uint8_t BLE : 1;
    uint8_t IF_INC : 1;
    uint8_t SIM : 1;
    uint8_t PP_OD : 1;
    uint8_t H_LACTIVE : 1;
    uint8_t BDU : 1;
    uint8_t BOOT : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_CTRL3C;

typedef union {
  struct {
    uint8_t res : 1;
    uint8_t LPF1_SEL_G : 1;
    uint8_t I2C_disable : 1;
    uint8_t DRDY_MASK : 1;
    uint8_t INT2_on_INT1 : 1;
    uint8_t DEN_DRDY_INT1 : 1;
    uint8_t SLEEP : 1;
    uint8_t DEN_XL_EN : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_CTRL4C;

typedef union {
  struct {
    uint8_t ST_XL : 2;
    uint8_t ST_G : 2;
    uint8_t DEN_LH : 1;
    uint8_t ROUNDING : 3;
  } bits;
  uint8_t reg_value;
} LSM6DSL_CTRL5C;

typedef union {
  struct {
    uint8_t FTYPE : 2;
    uint8_t res : 1;
    uint8_t USR_OFF_W : 1;
    uint8_t XL_HM_MODE : 1;
    uint8_t LVL2_EN : 1;
    uint8_t LVL_EN : 1;
    uint8_t TRIG_EN : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_CTRL6C;

typedef union {
  struct {
    uint8_t res1 : 2;
    uint8_t ROUNDING_STATUS : 1;
    uint8_t res2 : 1;
    uint8_t HPM_G : 2;
    uint8_t HP_EN_G : 1;
    uint8_t G_HM_MODE : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_CTRL7G;

typedef union {
  struct {
    uint8_t LOW_PASS_ON_6D : 1;
    uint8_t res : 1;
    uint8_t HP_SLOPE_XL_EN : 1;
    uint8_t INPUT_COMPOSITE : 1;
    uint8_t HP_REF_MODE : 1;
    uint8_t HPCF_XL : 2;
    uint8_t LPF2_XL_EN : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_CTRL8XL;

typedef union {
  struct {
    uint8_t res1 : 2;
    uint8_t SOFT_EN : 1;
    uint8_t res2 : 1;
    uint8_t DEN_XL_G : 1;
    uint8_t DEN_Z : 1;
    uint8_t DEN_Y : 1;
    uint8_t DEN_X : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_CTRL9XL;

typedef union {
  struct {
    uint8_t SIGN_MOTION_EN : 1;
    uint8_t PEDO_RST_STEP : 1;
    uint8_t FUNC_EN : 1;
    uint8_t TILT_EN : 1;
    uint8_t PEDO_EN : 1;
    uint8_t TIMER_EN : 1;
    uint8_t res : 1;
    uint8_t WRIST_TILT_EN : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_CTRL10C;

typedef union {
  struct {
    uint8_t MASTER_ON : 1;
    uint8_t IRON_EN : 1;
    uint8_t PASS_THROUGH_MODE : 1;
    uint8_t PULL_UP_EN : 1;
    uint8_t START_CONFIG : 1;
    uint8_t res : 1;
    uint8_t DATA_VALID_SEL_FIFO : 1;
    uint8_t DRDY_ON_INT1 : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_MASTERCONFIG;

typedef union {
  struct {
    uint8_t Z_WU : 1;
    uint8_t Y_WU : 1;
    uint8_t X_WU : 1;
    uint8_t WU_IA : 1;
    uint8_t SLEEP_STATE_IA : 1;
    uint8_t FF_IA : 1;
    uint8_t res : 2;
  } bits;
  uint8_t reg_value;
} LSM6DSL_WAKE_UPSRC;

typedef union {
  struct {
    uint8_t Z_TAP : 1;
    uint8_t Y_TAP : 1;
    uint8_t X_TAP : 1;
    uint8_t TAP_SIGN : 1;
    uint8_t DOUBLE_TAP : 1;
    uint8_t SINGLE_TAP : 1;
    uint8_t TAP_IA : 1;
    uint8_t res : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_TAPSRC;

typedef union {
  struct {
    uint8_t XL : 1;
    uint8_t XH : 1;
    uint8_t YL : 1;
    uint8_t YH : 1;
    uint8_t ZL : 1;
    uint8_t ZH : 1;
    uint8_t D6D_IA : 1;
    uint8_t DEN_DRDY : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_D6DSRC;

typedef union {
  struct {
    uint8_t XLDA : 1;
    uint8_t GDA : 1;
    uint8_t TDA : 1;
    uint8_t res : 5;
  } bits;
  uint8_t reg_value;
} LSM6DSL_STATUSREG;

typedef union {
  struct {
    uint8_t DIFF_FIFO_0_7 : 8;
  } bits;
  uint8_t reg_value;
} LSM6DSL_FIFOSTATUS1;

typedef union {
  struct {
    uint8_t DIFF_FIFO_8_10 : 3;
    uint8_t res : 1;
    uint8_t FIFO_EMPTY : 1;
    uint8_t FIFO_FULL_SMART : 1;
    uint8_t OVER_RUN : 1;
    uint8_t WaterM : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_FIFOSTATUS2;

typedef union {
  struct {
    uint8_t FIFO_PATTERN_0_7 : 8;
  } bits;
  uint8_t reg_value;
} LSM6DSL_FIFOSTATUS3;

typedef union {
  struct {
    uint8_t FIFO_PATTERN_8_9 : 2;
    uint8_t res : 6;
  } bits;
  uint8_t reg_value;
} LSM6DSL_FIFOSTATUS4;

typedef union {
  struct {
    uint8_t SENSORHUB_END_OP : 1;
    uint8_t SI_END_OP : 1;
    uint8_t HI_FAIL : 1;
    uint8_t STEP_OVERFLOW : 1;
    uint8_t STEP_DETECTED : 1;
    uint8_t TILT_IA : 1;
    uint8_t SIGN_MOTION_IA : 1;
    uint8_t STEP_COUNT_DELTA_IA : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_FUNCSRC1;

typedef union {
  struct {
    uint8_t WRIST_TILT_IA : 1;
    uint8_t res1 : 2;
    uint8_t SLAVE0_NACK : 1;
    uint8_t SLAVE1_NACK : 1;
    uint8_t SLAVE2_NACK : 1;
    uint8_t SLAVE3_NACK : 1;
    uint8_t res2 : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_FUNCSRC2;

typedef union {
  struct {
    uint8_t res : 2;
    uint8_t WRIST_TILT_IA_Zneg : 1;
    uint8_t WRIST_TILT_IA_Zpos : 1;
    uint8_t WRIST_TILT_IA_Yneg : 1;
    uint8_t WRIST_TILT_IA_Ypos : 1;
    uint8_t WRIST_TILT_IA_Xneg : 1;
    uint8_t WRIST_TILT_IA_Xpos : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_WRIST_TILTIA;

typedef union {
  struct {
    uint8_t LIR : 1;
    uint8_t TAP_Z_EN : 1;
    uint8_t TAP_Y_EN : 1;
    uint8_t TAP_X_EN : 1;
    uint8_t SLOPE_FDS : 1;
    uint8_t NACT_EN : 2;
    uint8_t INTERRUPTS_ENABLE : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_TAPCFG;

typedef union {
  struct {
    uint8_t TAP_THS : 5;
    uint8_t SIXD_THS : 2;
    uint8_t D4D_EN : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_TAP_THS6D;

typedef union {
  struct {
    uint8_t SHOCK : 2;
    uint8_t QUIET : 2;
    uint8_t DUR : 4;
  } bits;
  uint8_t reg_value;
} LSM6DSL_INTDUR2;

typedef union {
  struct {
    uint8_t WK_THS : 6;
    uint8_t res : 1;
    uint8_t SINGLE_DOUBLE_TAP : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_WAKEUP_THS;

typedef union {
  struct {
    uint8_t SLEEP_DUR : 4;
    uint8_t TIMER_HR : 1;
    uint8_t WAKE_DUR : 2;
    uint8_t FF_DUR_5 : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_WAKEUP_DUR;

typedef union {
  struct {
    uint8_t FF_THS : 3;
    uint8_t FF_DUR_0_4 : 5;
  } bits;
  uint8_t reg_value;
} LSM6DSL_FREEFALL;

typedef union {
  struct {
    uint8_t INT1_TIMER : 1;
    uint8_t INT1_TILT : 1;
    uint8_t INT1_6D : 1;
    uint8_t INT1_DOUBLE_TAP : 1;
    uint8_t INT1_FF : 1;
    uint8_t INT1_WU : 1;
    uint8_t INT1_SINGLE_TAP : 1;
    uint8_t INT1_INACT_STATE : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_MD1CFG;

typedef union {
  struct {
    uint8_t INT2_IRON : 1;
    uint8_t INT2_TILT : 1;
    uint8_t INT2_6D : 1;
    uint8_t INT2_DOUBLE_TAP : 1;
    uint8_t INT2_FF : 1;
    uint8_t INT2_WU : 1;
    uint8_t INT2_SINGLE_TAP : 1;
    uint8_t INT2_INACT_STATE : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_MD2CFG;

typedef union {
  struct {
    uint8_t X_OFS_USR_0_7 : 8;
  } bits;
  uint8_t reg_value;
} LSM6DSL_X_OFSUSR;

typedef union {
  struct {
    uint8_t Y_OFS_USR_0_7 : 8;
  } bits;
  uint8_t reg_value;
} LSM6DSL_Y_OFSUSR;

typedef union {
  struct {
    uint8_t Z_OFS_USR_0_7 : 8;
  } bits;
  uint8_t reg_value;
} LSM6DSL_Z_OFSUSR;

typedef union {
  struct {
    uint8_t hs_min : 5;
    uint8_t res : 2;
    uint8_t PEDO_FS : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_CONFIG_PEDO_THSMIN;

typedef union {
  struct {
    uint8_t SM_THS : 8;
  } bits;
  uint8_t reg_value;
} LSM6DSL_SMTHS;

typedef union {
  struct {
    uint8_t DEB_STEP : 3;
    uint8_t DEB_TIME : 5;
  } bits;
  uint8_t reg_value;
} LSM6DSL_PEDO_DEBREG;

typedef union {
  struct {
    uint8_t SC_DELTA : 8;
  } bits;
  uint8_t reg_value;
} LSM6DSL_STEP_COUNTDELTA;

typedef union {
  struct {
    uint8_t WRIST_TILT_TIMER : 8;
  } bits;
  uint8_t reg_value;
} LSM6DSL_A_WRIST_TILTLAT;

typedef union {
  struct {
    uint8_t WRIST_TILT_THS : 8;
  } bits;
  uint8_t reg_value;
} LSM6DSL_A_WRIST_TILTTHS;

typedef union {
  struct {
    uint8_t res : 2;
    uint8_t WRIST_TILT_MASK_Zneg : 1;
    uint8_t WRIST_TILT_MASK_Zpos : 1;
    uint8_t WRIST_TILT_MASK_Yneg : 1;
    uint8_t WRIST_TILT_MASK_Ypos : 1;
    uint8_t WRIST_TILT_MASK_Xneg : 1;
    uint8_t WRIST_TILT_MASK_Xpos : 1;
  } bits;
  uint8_t reg_value;
} LSM6DSL_A_WRIST_TILTMask;

typedef struct {
  LSM6DSL_CTRL1XL ctrl1;
  LSM6DSL_CTRL2G ctrl2;
  LSM6DSL_CTRL3C ctrl3;
  LSM6DSL_CTRL4C ctrl4;
  LSM6DSL_CTRL5C ctrl5;
  LSM6DSL_CTRL6C ctrl6;
  LSM6DSL_CTRL7G ctrl7;
  LSM6DSL_CTRL8XL ctrl8;
  LSM6DSL_CTRL9XL ctrl9;
  LSM6DSL_CTRL10C ctrl10;
} __attribute__((packed)) LSM6DSL_CTRL1;

typedef struct {
  LSM6DSL_DRDY_PULSE_CFGG intpulsectrl;
  uint8_t reserved;
  LSM6DSL_INT1CTRL int1ctrl;
  LSM6DSL_INT2CTRL int2ctrl;
} __attribute__((packed)) LSM6DSLINTCFG;

typedef struct {
  int16_t raw_temperature;
  int16_t raw_gyro_x;
  int16_t raw_gyro_y;
  int16_t raw_gyro_z;
  int16_t raw_acc_x;
  int16_t raw_acc_y;
  int16_t raw_acc_z;
} __attribute__((packed)) LSM6DSL_RAW_VALUES;

typedef struct {
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float temperature;
} LSM6DSL_VALUES;

/* function prototypes */
HAL_StatusTypeDef LSM6DSL_initialize(void);
HAL_StatusTypeDef LSM6DSL_data_ready(void);
LSM6DSL_VALUES LSM6DSL_get_values(void);
#endif /* INC_LSM6DSL_H_ */
