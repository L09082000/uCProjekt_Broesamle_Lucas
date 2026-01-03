// Copyright [2022] <Fabian Steger>

#ifndef DRIVERS_SENSORS_RANGESENSOR_VL53L0X_READ_H_
#define DRIVERS_SENSORS_RANGESENSOR_VL53L0X_READ_H_

#include "main.h"
#include "./vl53l0x/vl53l0x_def.h"
#include "vl53l0x/vl53l0x_tof.h"


VL53L0X_Error vl53l0x_initialize(void);
HAL_StatusTypeDef VL53L0X_data_ready(void);
VL53L0X_Error SetupContinousIntMeasurement(VL53L0X_Dev_t *, int);

#endif
