/*
 * mpu6050.h
 *
 *  Created on: 24 Ara 2022
 *      Author: Can Guveren
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "mpu6050_regmap.h"
#include <math.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

#define MPU6050_ADDR			(0x68 << 1)
#define I2C_TIMEOUT				1000

//@defgroup Gyro_range
#define GYRO_RANGE_250			0
#define GYRO_RANGE_500			1
#define GYRO_RANGE_1000			2
#define GYRO_RANGE_2000			3

//@defgroup Accel_range
#define ACCEL_RANGE_2			0
#define ACCEL_RANGE_4			1
#define ACCEL_RANGE_8			2
#define ACCEL_RANGE_16			3

//@defgroup Standby_mode_axis
#define STBY_XA 0x20
#define STBY_YA 0x10
#define STBY_ZA 0x08
#define STBY_XG	0x04
#define STBY_YG 0x02
#define STBY_ZG 0x01

//@defgroup Low_power_mode_freq
#define LP_WAKE_CTRL_1_25HZ 0x00
#define LP_WAKE_CTRL_5HZ	0x01
#define LP_WAKE_CTRL_20HZ	0x02
#define LP_WAKE_CTRL_40HZ	0x03

typedef enum
{
	ON,
	OFF
}MPU6050_Status;


typedef struct
{
	double Axis_X;
	double Axis_Y;
	double Axis_Z;

}MPU6050_DataAxis;

typedef struct
{
	double Roll;
	double Pitch;
	double Yaw;

	double previousTime;
	double currentTime;
	double elapsedTime;

    double accelAngle_X;
    double accelAngle_Y;
    double gyroAngle_X;
    double gyroAngle_Y;

}MPU6050_CFTypeDef;

typedef struct
{
	I2C_HandleTypeDef *MPU6050_I2C_Handle;  //I2C_HandleTypeDef
	uint8_t SampleRateDivider;				//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)   Gyroscope Output Rate = 8kHz
	uint8_t GyroRange;						//@defgroup Accel_range
	uint8_t AccelRange;						//@defgroup Accel_range

}MPU6050_InitTypeDef;

/* Init Function */
HAL_StatusTypeDef MPU6050_Init(MPU6050_InitTypeDef *MPU6050Config);

/* Gyro Functions */
void MPU6050_readGyro(MPU6050_DataAxis *Data);
int16_t MPU6050_readGyroX();
int16_t MPU6050_readGyroY();
int16_t MPU6050_readGyroZ();

/* Accel Functions */
void MPU6050_readAccel(MPU6050_DataAxis *Data);
int16_t MPU6050_readAccelX();
int16_t MPU6050_readAccelY();
int16_t MPU6050_readAccelZ();

/* Temperature Funtions */
void MPU6050_TemperatureSensor(MPU6050_Status status);
float MPU6050_readTemperature();

/* Mode Funtions */
void MPU6050_SleepMode(MPU6050_Status status);
void MPU6050_StandbyMode(MPU6050_Status status, uint8_t StandbyAxis);
void MPU6050_CycleMode(MPU6050_Status status);
void MPU6050_CycleMode_Freq(uint8_t freq);

void MPU6050_ComplemantryFilter(MPU6050_DataAxis *GyroData, MPU6050_DataAxis *AccelData,MPU6050_CFTypeDef *CFilter);

#endif /* MPU6050_H_ */
