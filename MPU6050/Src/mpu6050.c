/*
 * mpu6050.c
 *
 *  Created on: 24 Ara 2022
 *      Author: Can Guveren
 */

#include "mpu6050.h"

/* I2C Handle */
I2C_HandleTypeDef *MPU6050_I2CHandler;

float GyroScaleFactor;
float AccelScaleFactor;

/**
 * @brief regWrite, Writes to MPU6050 register using I2C
 * @param regAddr = MPU6050 Register Address
 * @param val = This value is written to the ADXL345 Register
 * @retval void
 */
static void regWrite(uint8_t regAddr, uint8_t val)
{
	HAL_I2C_Mem_Write(MPU6050_I2CHandler, MPU6050_ADDR, regAddr, 1, &val, 1, HAL_MAX_DELAY);

}

/**
 * @brief regWrite, Writes to MPU6050 register using I2C
 * @param regAddr = MPU6050 register address
 * @param buff = The value read from the register is assigned to this variable.
 * @param size = Byte length to be read
 * @retval void
 */
static void regRead(uint8_t regAddr, uint8_t *buff, uint16_t size)
{
	HAL_I2C_Mem_Read(MPU6050_I2CHandler, MPU6050_ADDR, regAddr, 1, buff, size, HAL_MAX_DELAY);
}

/**
 * @brief MPU6050_Init, configures MPU6050
 * @param MPU6050Config = include I2C_Handler and some settings
 * @retval HAL_StatusTypeDef
 */
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
		GyroScaleFactor = (32767 / (250 * pow(2, MPU6050Config->GyroRange))); //Gyro scale factor calculation

		/* Accelerometer full scale range configuration */
		regRead(ACCEL_CONFIG, &regVal, 1);
		regVal |= (MPU6050Config->AccelRange << 3);
		regWrite(ACCEL_CONFIG, regVal);
		AccelScaleFactor = (32767 / (2 * pow(2, MPU6050Config->AccelRange)));  //Accel scale factor calculation

		return HAL_OK;
	}

	return HAL_ERROR;
}

/**
 * @brief read gyro value in the register
 * @param Data = read values are assigned to this buffer / enum MPU6050_DataAxis
 * @retval void
 */
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

/**
 * @brief read gyro x axis value
 * @retval int16_t
 */
int16_t MPU6050_readGyroX()
{
	uint8_t dataBuffer[2];

	regRead(GYRO_XOUT_H, dataBuffer, 2);

	return (((int16_t)((dataBuffer[0] << 8) | dataBuffer[1])) / GyroScaleFactor);
}

/**
 * @brief read gyro y axis value
 * @retval int16_t
 */
int16_t MPU6050_readGyroY()
{
	uint8_t dataBuffer[2];

	regRead(GYRO_YOUT_H, dataBuffer, 2);

	return (((int16_t)((dataBuffer[0] << 8) | dataBuffer[1])) / GyroScaleFactor);
}

/**
 * @brief read gyro z axis value
 * @retval int16_t
 */
int16_t MPU6050_readGyroZ()
{
	uint8_t dataBuffer[2];

	regRead(GYRO_ZOUT_H, dataBuffer, 2);

	return (((int16_t)((dataBuffer[0] << 8) | dataBuffer[1])) / GyroScaleFactor);
}

/**
 * @brief read acceleration value in the register
 * @param Data = read values are assigned to this buffer / enum MPU6050_DataAxis
 * @retval void
 */
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

/**
 * @brief read acceleration x axis value
 * @retval int16_t
 */
int16_t MPU6050_readAccelX()
{
	uint8_t dataBuffer[2];

	regRead(ACCEL_XOUT_H, dataBuffer, 2);

	return (((int16_t)((dataBuffer[0] << 8) | dataBuffer[1])) / AccelScaleFactor);
}

/**
 * @brief read acceleration y axis value
 * @retval int16_t
 */
int16_t MPU6050_readAccelY()
{
	uint8_t dataBuffer[2];

	regRead(ACCEL_YOUT_H, dataBuffer, 2);

	return (((int16_t)((dataBuffer[0] << 8) | dataBuffer[1])) / AccelScaleFactor);
}

/**
 * @brief read acceleration z axis value
 * @retval int16_t
 */
int16_t MPU6050_readAccelZ()
{
	uint8_t dataBuffer[2];

	regRead(ACCEL_ZOUT_H, dataBuffer, 2);

	return (((int16_t)((dataBuffer[0] << 8) | dataBuffer[1])) / AccelScaleFactor);
}

/**
 * @brief read temperature value
 * @retval int16_t
 */
float MPU6050_readTemperature()
{
	uint8_t dataBuffer[2];
	float tempVal;
	regRead(TEMP_OUT_H, dataBuffer, 2);

	tempVal = ((int16_t)((dataBuffer[0] << 8) | dataBuffer[1]));
	tempVal = (float)((tempVal / 340) + 36.53);
	return tempVal;
}


/**
 * @brief temperature sensor on or off
 * @param ON or OFF enum MPU6050_Status
 * @retval void
 */
void MPU6050_TemperatureSensor(MPU6050_Status status)
{

	uint8_t regVal;

	regRead(PWR_MGMT_1, &regVal, 1);
	status == ON ? (regVal |= (1 << 3)) : (regVal &= ~(1 << 3));
	regWrite(PWR_MGMT_1, regVal);
}

/**
 * @brief sleep mode on or off
 * @param ON or OFF enum MPU6050_Status
 * @retval void
 */
void MPU6050_SleepMode(MPU6050_Status status)
{
	uint8_t regVal;

	regRead(PWR_MGMT_1, &regVal, 1);
	status == ON ? (regVal |= (1 << 6)) : (regVal &= ~(1 << 6));
	regWrite(PWR_MGMT_1, regVal);

}

/**
 * @brief standby mode on or off
 * @param ON or OFF enum MPU6050_Status
 * @param StandbyAxis = @defgroup Standby_mode_axis select standby mode axis
 * @retval void
 */
void MPU6050_StandbyMode(MPU6050_Status status, uint8_t StandbyAxis)
{
	uint8_t regVal;

	regRead(PWR_MGMT_2, &regVal, 1);
	status == ON ? (regVal |= StandbyAxis) : (regVal &= ~(StandbyAxis));
	regWrite(PWR_MGMT_2, regVal);
}

/**
 * @brief cycle mode on or off
 * @param ON or OFF enum MPU6050_Status
 * @retval void
 */
void MPU6050_CycleMode(MPU6050_Status status)
{
	uint8_t regVal;

	regRead(PWR_MGMT_1, &regVal, 1); 					  //Set CYCLE bit to 1
	status == ON ? (regVal |= (1 << 5)) : (regVal &= ~(1 << 5));
	regWrite(PWR_MGMT_1, regVal);

	MPU6050_SleepMode(OFF); 						  		// Set SLEEP bit to 0
	MPU6050_TemperatureSensor(OFF); 				  		// Set TEMP_DIS bit to 1
	MPU6050_StandbyMode(STBY_XG | STBY_YG | STBY_ZG, ON); 	// Set STBY_XG, STBY_YG, STBY_ZG bits to 1
}

/**
 * @brief cycle mode set frequency
 * @param freq = @defgroup Low_power_mode_freq select accelerometer only low power mode frequency
 * @retval void
 */
void MPU6050_CycleMode_Freq(uint8_t freq)
{
	uint8_t regVal;

	regRead(PWR_MGMT_2, &regVal, 1);
	regVal |= (freq << 6);
	regWrite(PWR_MGMT_2, regVal);
}


/*
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
*/
