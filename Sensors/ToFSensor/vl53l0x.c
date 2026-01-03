#include "main.h"
#include "vl53l0x.h"
#include "vl53l0x/vl53l0x_api.h"
#include "vl53l0x/vl53l0x_api_core.h"

#include "cmsis_os2.h"

extern I2C_HandleTypeDef hi2c2; // initialisiert in der main
extern osSemaphoreId_t I2C2availableHandle;

volatile float vl53l0x_last_reading_distance__meters;
volatile uint32_t vl53l0x_last_reading_time__ticks;
volatile uint32_t vl53l0x_last_reading_duration__ticks;

#define VL53L0X_I2C_ADDR 0x29 // Seite 29 um2153 <- i2c2 P47 um2153
#define VL53L0X_ID ((uint16_t)0xEEAA)

VL53L0X_Dev_t  Dev;
VL53L0X_RangingMeasurementData_t vl53l0x_values;

/**
 * @brief  VL53L0X proximity sensor Initialization.
 */
VL53L0X_Error vl53l0x_initialize(void)
{

  /* end sensor shutdown */
  HAL_GPIO_WritePin(VL53L0X_XSHUT_GPIO_Port, VL53L0X_XSHUT_Pin,
                    GPIO_PIN_SET); /* Xshutdown pin, active low */

  HAL_Delay(2); /* tboot after Xshut is 1.2 ms max - DS p 14 */

  uint16_t vl53l0x_id = 0;
  VL53L0X_Error ret_value = 1; /* all error codes are defined <=0, so set it to something that is not defined */
  VL53L0X_DeviceInfo_t VL53L0X_DeviceInfo;
  VL53L0X_Dev_t  *pDev = &Dev;

  Dev.I2cHandle = &hi2c2;
  Dev.I2cDevAddr = ((uint16_t)(VL53L0X_I2C_ADDR)) << 1;

  ret_value = VL53L0X_GetDeviceInfo(pDev, &VL53L0X_DeviceInfo);
  if (ret_value == VL53L0X_ERROR_NONE)
  {
	  ret_value = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &vl53l0x_id);
	  if (ret_value == VL53L0X_ERROR_NONE)
	  {
    	if (vl53l0x_id == VL53L0X_ID)
    	{
    	  ret_value = VL53L0X_DataInit(pDev);
    	  if (ret_value == VL53L0X_ERROR_NONE)
    	  {
    		  Dev.Present = 1;
    		  SetupContinousIntMeasurement(pDev,200000); /* period given in usec ->200msec that is required for 'performance mode' */
    	  }
    	}
	  }
  }
  return ret_value;
}



/**
 *  Setup sensor for continuous measurement defined by the parameter period that is raising an interrupt
 *  after measurement, setting is considering the high-accuracy mode that has a min period of 200msec
 */
VL53L0X_Error SetupContinousIntMeasurement(VL53L0X_Dev_t *pDev, int period)
{
	VL53L0X_Error ret_value = 1;
	FixPoint1616_t LimitCheckCurrent;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;

	ret_value = VL53L0X_StaticInit(pDev); // Device Initialization
	if (ret_value != VL53L0X_ERROR_NONE) {
		return ret_value;
	}

	ret_value = VL53L0X_PerformRefCalibration(pDev, &VhvSettings, &PhaseCal); // Device Initialization
	if (ret_value != VL53L0X_ERROR_NONE) {
		return ret_value;
	}

	ret_value = VL53L0X_SetDeviceMode(pDev,
	VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in continuous mode ranging mode
	if (ret_value != VL53L0X_ERROR_NONE) {
		return ret_value;
	}

	ret_value = VL53L0X_SetLimitCheckEnable(pDev,
	VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	if (ret_value != VL53L0X_ERROR_NONE) {
		return ret_value;
	}

	ret_value = VL53L0X_SetLimitCheckEnable(pDev,
	VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	if (ret_value != VL53L0X_ERROR_NONE) {
		return ret_value;
	}

	ret_value = VL53L0X_SetLimitCheckValue(pDev,
	VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
			(FixPoint1616_t) (0.25 * 65536));
	if (ret_value != VL53L0X_ERROR_NONE) {
		return ret_value;
	}

	ret_value = VL53L0X_SetLimitCheckValue(pDev,
	VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t) (18 * 65536));
	if (ret_value != VL53L0X_ERROR_NONE) {
		return ret_value;
	}

	ret_value = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDev,
			max(200000, period));
	if (ret_value != VL53L0X_ERROR_NONE) {
		return ret_value;
	}

	ret_value = VL53L0X_StartMeasurement(pDev);

	return ret_value;
}

HAL_StatusTypeDef VL53L0X_data_ready(void)
{
	volatile VL53L0X_Error ret_value = 1;
	ret_value = VL53L0X_GetRangingMeasurementData(&Dev, &vl53l0x_values);
    if (ret_value == VL53L0X_ERROR_NONE)
    {
    	ret_value = VL53L0X_ClearInterruptMask(&Dev, 0);
    }

    osSemaphoreRelease(I2C2availableHandle);
    if (ret_value == VL53L0X_ERROR_NONE)
    {
    	return HAL_OK;
    }
    else
    {
    	return HAL_BUSY;
    }
}
