/*
 * mpu6500.h
 *
 *  Created on: May 10, 2025
 *      Author: ngovi
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#include <math.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define ADDR_DEVICE				0x68

#define PWR_MGMT1_REG			0x6B
#define CONFIG_REG				0x1A
#define SMPLRT_REG				0x19
#define GYRO_CONFIG_REG			0x1B
#define ACCEL_CONFIG_REG		0x1C

#define ACCEL_MEASUR  			0x3B
#define GYRO_MEASUR				0x43

#define INT_CLK					0x00
#define PLL_CLK_X				0x01

#define FS_GYRO_250				0x00
#define FS_GYRO_500				0x08
#define FS_GYRO_1000			0x10
#define FS_GYRO_2000			0x18

#define FS_ACCEL_2G				0x00
#define FS_ACCEL_4G				0x08
#define FS_ACCEL_8G				0x10
#define FS_ACCEL_16G			0x18

#define LSB_GYRO_250			131.0
#define LSB_GYRO_500			65.5
#define LSB_GYRO_1000			32.8
#define LSB_GYRO_2000			16.4

#define LSB_ACCEL_2G			16384.0
#define LSB_ACCEL_4G			8192.0
#define LSB_ACCEL_8G			4096.0
#define LSB_ACCEL_16G			2048.0

typedef struct{
	I2C_HandleTypeDef *hi2c;
	float gx, gy, gz, ax, ay, az;
	float offset_gx, offset_gy, offset_gz, offset_ax, offset_ay, offset_az;
	float roll, pitch, yaw;
}MPU6500_t;

//typedef struct Raw_Data{
//	int16_t gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw;
//}raw_data;
//
//typedef struct Sensor_Data{
//	float gx, gy, gz, ax, ay, az;
//}data_sensor;
//
//typedef struct Real_Angle{
//	float pitch, roll, yaw;
//}angle;
//
//typedef struct Data_offset{
//	float offset_gx, offset_gy, offset_gz, offset_ax, offset_ay, offset_az;
//}offset;


void MPU6500_Init(I2C_HandleTypeDef *hi2c);

void MPU6500_Raw_Gyro(I2C_HandleTypeDef *hi2c, int16_t* raw_gyro);

void MPU6500_Raw_Accel(I2C_HandleTypeDef *hi2c, int16_t* raw_accel);

void MPU6500_Calibrate(I2C_HandleTypeDef *hi2c, MPU6500_t* mpu, uint16_t num_sample);

void MPU6500_Read_Data(I2C_HandleTypeDef *hi2c, MPU6500_t* mpu);

void Angle_CompFilter(MPU6500_t* mpu, float dt, float alpha);

void Read_Angel(MPU6500_t* mpu, volatile float* pitch, volatile float*roll, volatile float *yaw);



#endif /* INC_MPU6500_H_ */
