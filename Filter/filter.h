#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include "../Sensors/AccelerationSensor/lsm6dsl.h"  // für LSM6DSL_VALUES
#include "../Sensors/MagnetometerSensor/lis3mdl.h"   // für LIS3MDL_VALUES

/* --- Struktur für gefilterte LSM6DSL-IMU-Werte --- */
typedef struct
{
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} Biquad;

/* ============================
 * Filtered value structs
 * ============================ */
typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} LSM6DSL_FILTERED_VALUES;

typedef struct
{
    float x;
    float y;
    float z;
} LIS3MDL_FILTERED_VALUES;

/* ============================
 * API
 * ============================ */
void LSM6DSL_Filter_Init(void);
void LSM6DSL_Filter_Update(LSM6DSL_VALUES current);

void LIS3MDL_Filter_Init(void);
void LIS3MDL_Filter_Update(LIS3MDL_VALUES current);

/* ============================
 * Global filtered values
 * ============================ */
extern LSM6DSL_FILTERED_VALUES lsm6dsl_filtered_values;
extern LIS3MDL_FILTERED_VALUES lis3mdl_filtered_values;

#endif
