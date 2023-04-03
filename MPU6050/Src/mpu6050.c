/*
 * mpu6050.c
 *
 *  Created on: 24 Ara 2022
 *      Author: Can Guveren
 */

#include "mpu6050.h"

/*I2C Handle*/
I2C_HandleTypeDef *MPU6050_I2CHandler;

float GyroScaleFactor;
float AccelScaleFactor;

HAL_StatusTypeDef regWrite(uint8_t regAddr, uint8_t val)
{
	HAL_StatusTypeDef checkFunc;

	checkFunc = HAL_I2C_Mem_Write(MPU6050_I2CHandler, MPU6050_ADDR, regAddr, 1, &val, 1, HAL_MAX_DELAY);

	return checkFunc;
}


HAL_StatusTypeDef regRead(uint8_t regAddr, uint8_t *buff, uint16_t size)
{

	HAL_StatusTypeDef checkFunc;

	checkFunc = HAL_I2C_Mem_Read(MPU6050_I2CHandler, MPU6050_ADDR, regAddr, 1, buff, size, HAL_MAX_DELAY);

	return checkFunc;
}


HAL_StatusTypeDef MPU6050_Init(MPU6050_InitTypeDef *MPU6050Config)
{
	MPU6050_I2CHandler = MPU6050Config->MPU6050_I2C_Handle;

	uint8_t who_am_i;

	regRead(WHO_AM_I, &who_am_i, 1);

	if(who_am_i == 0x68)
	{
		uint8_t regVal;

		/* MPU6050 wakes up from sleep mode*/
		regRead(PWR_MGMT_1, &regVal, 1);
		regVal &= ~(1 << 6);
		regWrite(PWR_MGMT_1, regVal);

		/* Gyroscope full scale range configuration */
		regWrite(SMPLRT_DIV, MPU6050Config->SampleRateDivider);

		/* Gyroscope full scale range configuration */
		regRead(GYRO_CONFIG, &regVal, 1);
		regVal |= (MPU6050Config->GyroRange << 3);
		regWrite(GYRO_CONFIG, regVal);
		GyroScaleFactor = (32767 / (250 * pow(2, MPU6050Config->GyroRange)));

		/* Accelerometer full scale range configuration */
		regRead(ACCEL_CONFIG, &regVal, 1);
		regVal |= (MPU6050Config->AccelRange << 3);
		regWrite(ACCEL_CONFIG, regVal);
		AccelScaleFactor = (32767 / (2 * pow(2, MPU6050Config->AccelRange)));

		return HAL_OK;
	}

	return HAL_ERROR;
}

/***************************************** GYRO DATA *****************************************/
void MPU6050_readGyro(MPU6050_DataAxis *Data)
{
	uint8_t dataBuffer[6];
	int16_t rawX = 0, rawY = 0, rawZ = 0;

	regRead(GYRO_XOUT_H, dataBuffer, 6);

	rawX = (int16_t)((dataBuffer[0] << 8) | dataBuffer[1]);
	rawY = (int16_t)((dataBuffer[2] << 8) | dataBuffer[3]);
	rawZ = (int16_t)((dataBuffer[4] << 8) | dataBuffer[5]);

	Data->Axis_X = (double)(rawX / GyroScaleFactor);
	Data->Axis_Y = (double)(rawY / GyroScaleFactor);
	Data->Axis_Z = (double)(rawZ / GyroScaleFactor);
}

int16_t MPU6050_readGyroX()
{
	uint8_t dataBuffer[2];

	regRead(GYRO_XOUT_H, dataBuffer, 2);

	return (((int16_t)((dataBuffer[0] << 8) | dataBuffer[1])) / GyroScaleFactor);
}

int16_t MPU6050_readGyroY()
{
	uint8_t dataBuffer[2];

	regRead(GYRO_YOUT_H, dataBuffer, 2);

	return (((int16_t)((dataBuffer[0] << 8) | dataBuffer[1])) / GyroScaleFactor);
}

int16_t MPU6050_readGyroZ()
{
	uint8_t dataBuffer[2];

	regRead(GYRO_ZOUT_H, dataBuffer, 2);

	return (((int16_t)((dataBuffer[0] << 8) | dataBuffer[1])) / GyroScaleFactor);
}

/***************************************** ACCEL DATA *****************************************/
void MPU6050_readAccel(MPU6050_DataAxis *Data)
{
	uint8_t dataBuffer[6];
	int16_t rawX = 0, rawY = 0, rawZ = 0;

	regRead(ACCEL_XOUT_H, dataBuffer, 6);

	rawX = (int16_t)((dataBuffer[0] << 8) | dataBuffer[1]);
	rawY = (int16_t)((dataBuffer[2] << 8) | dataBuffer[3]);
	rawZ = (int16_t)((dataBuffer[4] << 8) | dataBuffer[5]);

	Data->Axis_X = (double)(rawX / AccelScaleFactor);
	Data->Axis_Y = (double)(rawY / AccelScaleFactor);
	Data->Axis_Z = (double)(rawZ /  AccelScaleFactor);
}

int16_t MPU6050_readAccelX()
{
	uint8_t dataBuffer[2];

	regRead(ACCEL_XOUT_H, dataBuffer, 2);

	return (((int16_t)((dataBuffer[0] << 8) | dataBuffer[1])) / AccelScaleFactor);
}

int16_t MPU6050_readAccelY()
{
	uint8_t dataBuffer[2];

	regRead(ACCEL_YOUT_H, dataBuffer, 2);

	return (((int16_t)((dataBuffer[0] << 8) | dataBuffer[1])) / AccelScaleFactor);
}

int16_t MPU6050_readAccelZ()
{
	uint8_t dataBuffer[2];

	regRead(ACCEL_ZOUT_H, dataBuffer, 2);

	return (((int16_t)((dataBuffer[0] << 8) | dataBuffer[1])) / AccelScaleFactor);
}

/***************************************** FILTERS *****************************************/
void MPU6050_ComplemantryFilter(MPU6050_DataAxis *GyroData, MPU6050_DataAxis *AccelData,MPU6050_CFTypeDef *CFilter)
{

	MPU6050_readAccel(AccelData);
	MPU6050_readGyro(GyroData);
	CFilter->accelAngle_X = atan(AccelData->Axis_Y/ sqrt(pow(AccelData->Axis_X, 2) + pow(AccelData->Axis_Z, 2))) * RAD_TO_DEG;
	CFilter->accelAngle_Y = atan(-1 * AccelData->Axis_X / sqrt(pow(AccelData->Axis_Y, 2) + pow(AccelData->Axis_Z, 2))) * RAD_TO_DEG;


	CFilter->previousTime = CFilter->currentTime;
	CFilter->currentTime = HAL_GetTick();
	CFilter->elapsedTime = (double)(CFilter->currentTime - CFilter->previousTime) / 1000;

	CFilter->gyroAngle_X = CFilter->gyroAngle_X + GyroData->Axis_X * CFilter->elapsedTime;
	CFilter->gyroAngle_Y = CFilter->gyroAngle_Y + GyroData->Axis_Y * CFilter->elapsedTime;
	CFilter->Yaw = CFilter->Yaw + GyroData->Axis_Z * CFilter->elapsedTime;

	//CFilter->Roll = 0.96 * CFilter->gyroAngle_X + 0.04 * CFilter->accelAngle_X;
	//CFilter->Pitch = 0.96 * CFilter->gyroAngle_Y + 0.04 * CFilter->accelAngle_Y;
	CFilter->Roll = 0.96 * (CFilter->Roll - GyroData->Axis_Y * CFilter->elapsedTime) + 0.04 * CFilter->accelAngle_Y;
	CFilter->Pitch = 0.96 * (CFilter->Pitch - GyroData->Axis_X * CFilter->elapsedTime) + 0.04 * CFilter->accelAngle_X;
}


/***************************************** TEMPERATURE DATA *****************************************/
HAL_StatusTypeDef MPU6050_TempSensor(MPU6050_SS Status)
{

	uint8_t regVal;
	HAL_StatusTypeDef checkFunc;

	checkFunc = regRead(PWR_MGMT_1, &regVal, 1);
	regVal &= ~(1 << 3);
	checkFunc = regWrite(PWR_MGMT_1, regVal);

	return checkFunc;
}

float MPU6050_readTemp()
{
	uint8_t dataBuffer[2];
	float tempVal;
	regRead(TEMP_OUT_H, dataBuffer, 2);

	tempVal = ((int16_t)((dataBuffer[0] << 8) | dataBuffer[1]));
	tempVal = (float)((tempVal / 340) + 36.53);
	return tempVal;
}



/*
HAL_StatusTypeDef MPU6050_SleepMode(MPU6050_SS Status)
{

}

HAL_StatusTypeDef MPU6050_TempSensorReset()
{

}
*/
