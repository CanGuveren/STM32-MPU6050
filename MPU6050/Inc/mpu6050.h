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


#define GYRO_RANGE_250			0
#define GYRO_RANGE_500			1
#define GYRO_RANGE_1000			2
#define GYRO_RANGE_2000			3

#define ACCEL_RANGE_2			0
#define ACCEL_RANGE_4			1
#define ACCEL_RANGE_8			2
#define ACCEL_RANGE_16			3

typedef enum
{
	ON,
	OFF
}MPU6050_SS;


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
	I2C_HandleTypeDef *MPU6050_I2C_Handle;
	uint8_t SampleRateDivider;
	uint8_t GyroRange;
	uint8_t AccelRange;

}MPU6050_InitTypeDef;



HAL_StatusTypeDef regWrite(uint8_t reg, uint8_t val);
HAL_StatusTypeDef regRead(uint8_t regAddr, uint8_t *buff, uint16_t size);

HAL_StatusTypeDef MPU6050_Init(MPU6050_InitTypeDef *MPU6050Config);

void MPU6050_readGyro(MPU6050_DataAxis *Data);
int16_t MPU6050_readGyroX();
int16_t MPU6050_readGyroY();
int16_t MPU6050_readGyroZ();

void MPU6050_readAccel(MPU6050_DataAxis *Data);
int16_t MPU6050_readAccelX();
int16_t MPU6050_readAccelY();
int16_t MPU6050_readAccelZ();

void MPU6050_ComplemantryFilter(MPU6050_DataAxis *GyroData, MPU6050_DataAxis *AccelData,MPU6050_CFTypeDef *CFilter);

HAL_StatusTypeDef MPU6050_TempSensor(MPU6050_SS Status);
float MPU6050_readTemp();

#endif /* MPU6050_H_ */
