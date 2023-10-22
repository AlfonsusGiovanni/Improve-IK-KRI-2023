/*AUTHOR: ALFONSUS GIOVANNI*/

#ifndef MPU9250_lib_H
#define MPU9250_lib_H

#include "main.h"
#include "math.h"

typedef struct
{
	int16_t
	Acc_X_Raw,
	Acc_Y_Raw,
	Acc_Z_Raw,
	Gyro_X_Raw,
	Gyro_Y_Raw,
	Gyro_Z_Raw,
	Mag_X_Raw,
	Mag_Y_Raw,
	Mag_Z_Raw,
	Temp_Raw;

	double
	Ax, Ay, Az,
	Gx, Gy, Gz,
	Call_Gx, Call_Gy, Call_Gz,
	MagX, MagY, MagZ,
	Call_MagX, Call_MagY, Call_MagZ;
	
	double 
	KalmanAngleX,
	KalmanAngleY;
	
}MPU9250_t;

typedef struct{
	int8_t gyro_stat;
	int8_t mag_stat;
	int8_t mpu_who_am_i;
	int8_t ak8963_who_am_i;
}MPU_status_t;

typedef enum{
	INITIALIZE_GYRO_OK = 0x01U,
	INITIALIZE_GYRO_FAILED = 0x02U,
}MPU9250_status_t;

typedef struct
{
	double Q_angle;
	double Q_bias;
	double R_measure;
	double angle;
	double bias;
	double P[2][2];
}Kalman_t;

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS     // 0.15 mG per LSB
};

uint8_t CHECK_MPU_WHO_AM_I(I2C_HandleTypeDef *I2Cx, MPU_status_t *get_status);
uint8_t CHECK_AK8963_WHO_AM_I(I2C_HandleTypeDef *I2Cx, MPU_status_t *get_status);
uint8_t MPU9250_Init(I2C_HandleTypeDef *I2Cx, MPU_status_t *get_status);
void Write_AK8963_Reg(I2C_HandleTypeDef *I2Cx, uint8_t sub_address, uint8_t data);
void Read_AK8963_Reg(I2C_HandleTypeDef *I2Cx, uint8_t sub_address, uint8_t count, uint8_t* data_store);
void MPU9250_Read_RawData(I2C_HandleTypeDef *I2Cx, MPU9250_t *DataStruct);
void MPU9250_Read_Data(I2C_HandleTypeDef *I2Cx, MPU9250_t *DataStruct);
void MPU9250_Callibrate_Gyro(I2C_HandleTypeDef *I2Cx, MPU9250_t *DataStruct, uint16_t numCallPoints);
void MPU9250_Read_CallMagData(I2C_HandleTypeDef *I2Cx, MPU9250_t *DataStruct);
void MPU9250_Read_MagData(I2C_HandleTypeDef *I2Cx, MPU9250_t *DataStruct);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
#endif
