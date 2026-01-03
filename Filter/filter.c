#include "filter.h"
#include <string.h>

/* ============================================================
 * GLOBAL FILTERED VALUES (echte Definition!)
 * ============================================================ */
LSM6DSL_FILTERED_VALUES lsm6dsl_filtered_values = {0};
LIS3MDL_FILTERED_VALUES lis3mdl_filtered_values = {0};

/* ============================================================
 * 5th Order Butterworth (SOS)
 * fs = 100 Hz, fc = 10 Hz
 * ============================================================ */
static const float sos[3][6] =
{
    {0.0013f, 0.0013f, 0.0000f, 1.0000f, -0.5095f,  0.0000f},
    {1.0000f, 2.0016f, 1.0016f, 1.0000f, -1.0966f,  0.3554f},
    {1.0000f, 1.9994f, 0.9994f, 1.0000f, -1.3693f,  0.6926f}
};

/* ============================================================
 * BIQUAD CHAINS – JEDER KANAL EIGEN!
 * ============================================================ */
static Biquad acc_x_biquads[3];
static Biquad acc_y_biquads[3];
static Biquad acc_z_biquads[3];

static Biquad gyro_x_biquads[3];
static Biquad gyro_y_biquads[3];
static Biquad gyro_z_biquads[3];

static Biquad mag_x_biquads[3];
static Biquad mag_y_biquads[3];
static Biquad mag_z_biquads[3];

/* ============================================================
 * Single biquad (DF2 Transposed)
 * ============================================================ */
static inline float Biquad_Process(Biquad *bq, float x)
{
    float w = x - bq->a1 * bq->z1 - bq->a2 * bq->z2;
    float y = bq->b0 * w + bq->b1 * bq->z1 + bq->b2 * bq->z2;

    bq->z2 = bq->z1;
    bq->z1 = w;

    return y;
}

/* ============================================================
 * Process through 3 SOS sections
 * ============================================================ */
static inline float Filter_Process(Biquad *chain, float x)
{
    float y = x;
    for (int i = 0; i < 3; i++)
    {
        y = Biquad_Process(&chain[i], y);
    }
    return y;
}

/* ============================================================
 * Initialize one biquad chain
 * ============================================================ */
static void Init_Biquad_Chain(Biquad *chain)
{
    memset(chain, 0, 3 * sizeof(Biquad));
    for (int i = 0; i < 3; i++)
    {
        chain[i].b0 = sos[i][0];
        chain[i].b1 = sos[i][1];
        chain[i].b2 = sos[i][2];
        chain[i].a1 = sos[i][4];
        chain[i].a2 = sos[i][5];
    }
}

/* ============================================================
 * LSM6DSL FILTER INIT
 * ============================================================ */
void LSM6DSL_Filter_Init(void)
{
    Init_Biquad_Chain(acc_x_biquads);
    Init_Biquad_Chain(acc_y_biquads);
    Init_Biquad_Chain(acc_z_biquads);

    Init_Biquad_Chain(gyro_x_biquads);
    Init_Biquad_Chain(gyro_y_biquads);
    Init_Biquad_Chain(gyro_z_biquads);
}

/* ============================================================
 * LSM6DSL FILTER UPDATE
 * ============================================================ */
void LSM6DSL_Filter_Update(LSM6DSL_VALUES current)
{
	lsm6dsl_filtered_values.acc_x = Filter_Process(acc_x_biquads, current.acc_x);
	lsm6dsl_filtered_values.acc_y = Filter_Process(acc_y_biquads, current.acc_y);
	lsm6dsl_filtered_values.acc_z = Filter_Process(acc_z_biquads, current.acc_z);
	lsm6dsl_filtered_values.gyro_x = Filter_Process(gyro_x_biquads, current.gyro_x);
	lsm6dsl_filtered_values.gyro_y = Filter_Process(gyro_y_biquads, current.gyro_y);
	lsm6dsl_filtered_values.gyro_z = Filter_Process(gyro_z_biquads, current.gyro_z);
}

/* ============================================================
 * LIS3MDL FILTER INIT
 * ============================================================ */
void LIS3MDL_Filter_Init(void)
{
    Init_Biquad_Chain(mag_x_biquads);
    Init_Biquad_Chain(mag_y_biquads);
    Init_Biquad_Chain(mag_z_biquads);
}

/* ============================================================
 * LIS3MDL FILTER UPDATE
 * ============================================================ */
void LIS3MDL_Filter_Update(LIS3MDL_VALUES current)
{
    lis3mdl_filtered_values.x = Filter_Process(mag_x_biquads, current.x);
    lis3mdl_filtered_values.y = Filter_Process(mag_y_biquads, current.y);
    lis3mdl_filtered_values.z = Filter_Process(mag_z_biquads, current.z);
}
