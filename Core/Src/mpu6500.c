/*
 * mpu6500.c
 *
 *  Created on: May 10, 2025
 *      Author: ngovi
 */

#include "mpu6500.h"
#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_i2c.h"



void MPU6500_Init(I2C_HandleTypeDef *hi2c){
	uint8_t data_clksel = INT_CLK;
//	uint8_t data_dlpf = 0x03;
	uint8_t data_smprt = 0x13;			//sample_rate: 50Hz
	uint8_t data_accel = FS_ACCEL_2G;
	uint8_t data_gyro = FS_GYRO_250;

	HAL_StatusTypeDef check = HAL_I2C_IsDeviceReady(hi2c, (ADDR_DEVICE << 1), 10, 100);
	if(check == HAL_OK){
		HAL_I2C_Mem_Write(hi2c, (ADDR_DEVICE << 1), PWR_MGMT1_REG, 1, &data_clksel, 1, 100);
//		HAL_I2C_Mem_Write(hi2c1, (ADDR_DEVICE << 1), CONFIG_REG, 1, &data_dlpf, 1, 100);
		HAL_I2C_Mem_Write(hi2c, (ADDR_DEVICE << 1), SMPLRT_REG, 1, &data_smprt, 1, 100);
		HAL_I2C_Mem_Write(hi2c, (ADDR_DEVICE << 1), ACCEL_CONFIG_REG, 1, &data_accel, 1, 100);
		HAL_I2C_Mem_Write(hi2c, (ADDR_DEVICE << 1), GYRO_CONFIG_REG, 1, &data_gyro, 1, 100);
	}
}


void MPU6500_Raw_Gyro(I2C_HandleTypeDef *hi2c, int16_t* raw_gyro){
	uint8_t Gyro_buffer[6];
	HAL_I2C_Mem_Read(hi2c, (ADDR_DEVICE << 1), GYRO_MEASUR, 1, Gyro_buffer, 6, 100);
	raw_gyro[0] = (int16_t)((Gyro_buffer[0] << 8) | Gyro_buffer[1]);
	raw_gyro[1] = (int16_t)((Gyro_buffer[2] << 8) | Gyro_buffer[3]);
	raw_gyro[2] = (int16_t)((Gyro_buffer[4] << 8) | Gyro_buffer[5]);

}

void MPU6500_Raw_Accel(I2C_HandleTypeDef *hi2c, int16_t* raw_accel){
	uint8_t Accel_buffer[6];
	HAL_I2C_Mem_Read(hi2c, (ADDR_DEVICE << 1), ACCEL_MEASUR, 1, Accel_buffer, 6, 100);
	raw_accel[0] = (int16_t)((Accel_buffer[0] << 8) | Accel_buffer[1]);
	raw_accel[1] = (int16_t)((Accel_buffer[2] << 8) | Accel_buffer[3]);
	raw_accel[2]= (int16_t)((Accel_buffer[4] << 8) | Accel_buffer[5]);

}

void MPU6500_Calibrate(I2C_HandleTypeDef *hi2c, MPU6500_t* mpu, uint16_t num_sample){
	int32_t total_gx = 0, total_gy = 0, total_gz = 0, total_ax = 0, total_ay = 0, total_az = 0;
	int16_t raw_gyro[3], raw_accel[3];
	for(int i = 0; i < num_sample; i++){
		MPU6500_Raw_Gyro(hi2c, raw_gyro);
		MPU6500_Raw_Accel(hi2c, raw_accel);

		total_gx += raw_gyro[0];
		total_gy += raw_gyro[1];
		total_gz += raw_gyro[2];

		total_ax += raw_accel[0];
		total_ay += raw_accel[1];
		total_az += raw_accel[2];
	}
	mpu->offset_gx = (float)total_gx/num_sample;
	mpu->offset_gy = (float)total_gy/num_sample;
	mpu->offset_gz = (float)total_gz/num_sample;

	mpu->offset_ax = (float)total_ax/num_sample;
	mpu->offset_ay = (float)total_ay/num_sample;
	mpu->offset_az = (float)total_az/num_sample + 16384.0;
}

void MPU6500_Read_Data(I2C_HandleTypeDef *hi2c, MPU6500_t* mpu){
	int16_t raw_gyro[3], raw_accel[3];
	MPU6500_Raw_Gyro(hi2c, raw_gyro);
	MPU6500_Raw_Accel(hi2c, raw_accel);

	mpu->gx = ((float)raw_gyro[0] - mpu->offset_gx)/131.0;
	mpu->gy = ((float)raw_gyro[1] - mpu->offset_gy)/131.0;
	mpu->gz = ((float)raw_gyro[2] - mpu->offset_gz)/131.0;
	mpu->ax = ((float)raw_accel[0] - mpu->offset_ax)/16384.0;
	mpu->ay = ((float)raw_accel[1] - mpu->offset_ay)/16384.0;
	mpu->az = ((float)raw_accel[2] - mpu->offset_az)/16384.0;
}


void Angle_CompFilter(MPU6500_t* mpu, float dt, float alpha){
	float gx = mpu->gx;
	float gy = mpu->gy;
	float gz = mpu->gz;
	float ax = mpu->ax;
	float ay = mpu->ay;
	float az = mpu->az;

	float accel_pitch = (atan2f(ax, sqrt(az*az + ay*ay)))*180.0/3.14;
	float accel_roll  = (atan2f(-ay, sqrt(az*az + ax*ax)))*180.0/3.14;
//	float accel_roll  = (atan2f(ay, az))*180.0/3.14;

	mpu->pitch = alpha*(mpu->pitch + gy*dt) + (1-alpha)*accel_pitch;
	mpu->roll = alpha*(mpu->roll + gx*dt) + (1-alpha)*accel_roll;
	mpu->yaw += gz*dt*(180.0f / 3.14);
}

void Read_Angel(MPU6500_t* mpu, volatile float* pitch, volatile float*roll, volatile float *yaw){

	*pitch = mpu->pitch;
	*roll = mpu->roll;
	*yaw = mpu->yaw;
}

