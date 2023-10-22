/*AUTHOR: ALFONSUS GIOVANNI*/

#include "MPU9250_lib.h"

#define RAD_TO_DEG 57.295779513082320876798154814105
#define PI 3,142857142857143

#define MPU9250_fdbckA 	115
#define MPU9250_fdbckB	113
#define MPU6050_fdbck 	104
#define MPU_MAG_fdbck		72

//MPU REGISTER SET
#define MPU_ADDR_LOW				0xD0
#define MPU_ADDR_HIGH				0xD1
#define WHO_AM_I 						0X75
#define USER_CTRL						0x6A
#define PWR_MGMT_1					0x6B
#define PWR_MGMT_2					0x6C
#define CONFIG_REG					0x1A
#define GYRO_CONFIG_REG			0x1B
#define ACCEL_CONFIG_REG		0x1C
#define ACCEL_CONFIG_REG2		0x1D
#define EXT_SENS_DATA_00    0x49
#define GYRO_XOUT_H					0x43
#define ACCEL_XOUT_H				0x3B
#define SMPLRT_DIV_REG			0x19
#define PWR_RESET						0x80
#define WHO_AM_I_RST 				0X68
#define CLCK_SEL						0x01
#define ENABLE_ALL					0x00
#define I2C_MST_EN					0x20
#define I2C_MST_CLK 				0x0D
#define I2C_MST_CTRL				0x24
#define I2C_SLV0_ADDR				0x25
#define I2C_SLV0_REG				0x26
#define I2C_SLV0_DO					0x63
#define I2C_SLV0_CTRL				0x27
#define I2C_SLV0_EN					0x80
#define I2C_READ_FLAG				0x80
#define I2C_SLV0_DO					0x63
#define GYRO_FS_SEL_250			0x00
#define GYRO_FS_SEL_500			0x08
#define GYRO_FS_SEL_1000    0x10
#define GYRO_FS_SEL_2000		0x18
#define ACCEL_FS_SEL_2G			0x00
#define ACCEL_FS_SEL_4G			0x08
#define ACCEL_FS_SEL_8G			0x10
#define ACCEL_FS_SEL_16G		0x18
#define SMPLRT_DIV_0				0x00
#define DLPS_460HZ					0x00
#define DLPS_180HZ					0x0B
#define DLPS_92HZ						0x0C
#define DLPS_41HZ						0x0D

//AK8963 REGISTER SET
#define AK8963_I2C_ADDR			0x0C
#define AK8963_WHO_AM_I			0x00
#define AK8963_INFO      		0x01
#define AK8963_ST1       		0x02  \
#define AK8963_ST2					0x09
#define AK8963_XOUT_L    		0x03  
#define AK8963_XOUT_H    		0x04
#define AK8963_YOUT_L    		0x05
#define AK8963_YOUT_H    		0x06
#define AK8963_ZOUT_L    		0x07
#define AK8963_ZOUT_H    		0x08
#define AK8963_CNTL1      	0x0A
#define AK8963_CNTL2				0x0B
#define AK8963_RST					0x01
#define AK8963_ST2       		0x09  
#define AK8963_PWR_DWN			0x00  
#define AK8963_SINGLW_MES		0x01  
#define AK8963_SLF_TST			0x08  
#define AK8963_FUSE_ROM			0x0F
#define AK8963_ASTC      		0x0C  
#define AK8963_I2CDIS    		0x0F  
#define AK8963_ASA       		0x10  
#define AK8963_CNT_MEAS1 		0x12
#define AK8963_CNT_MEAS2 		0x16

#define aScaleFactor_2G			16384.0
#define aScaleFactor_4G			8192.0
#define aScaleFactor_8G			4096.0
#define aScaleFactor_16G		2048.0

#define gScaleFactor_250		131.0
#define gScaleFactor_500		65.5
#define gScaleFactor_1000		32.8
#define gScaleFactor_2000		16.4

#define ACCEL_Z_CORRECTOR		14418.0

uint32_t 
I2C_Timout = 100,
timer;

uint8_t 
Magscale = MFS_16BITS,
MagMode = 0x02,
Mag_Adjust[3],
buffer[21],
mag_buffer[6];

Kalman_t KalmanX = 
{
	.Q_angle 		= 0.001f,
	.Q_bias 		= 0.003f,
	.R_measure 	=	0.03f
};

Kalman_t KalmanY =
{
	.Q_angle 		= 0.001f,
  .Q_bias 		= 0.003f,
  .R_measure 	= 0.03f
};

void Write_AK8963_Reg(I2C_HandleTypeDef *I2Cx, uint8_t sub_address, uint8_t data)
{
	uint8_t sub_reg;
	
	//SET SLAVE 0 TO AK89633, SET FOR WRITE
	sub_reg = AK8963_I2C_ADDR;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, I2C_SLV0_ADDR, 1, &sub_reg, 1, I2C_Timout);
	
	//SET REGISTER TO AK89633 SUB ADDRESS
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, I2C_SLV0_REG, 1, &sub_address, 1, I2C_Timout);
	
	//STORE DATA FOR WRITE
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, I2C_SLV0_DO, 1, &data, 1, I2C_Timout);

	//ENABLE I2C
	sub_reg = I2C_SLV0_EN | ((uint8_t)1);
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, I2C_SLV0_CTRL, 1, &sub_reg, 1, I2C_Timout);
}

void Read_AK8963_Reg(I2C_HandleTypeDef *I2Cx, uint8_t sub_address, uint8_t count, uint8_t* data_store)
{
	uint8_t sub_reg;
	
	//SET SLAVE 0 TO AK89633, SET FOR WRITE
	sub_reg = AK8963_I2C_ADDR | I2C_READ_FLAG;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, I2C_SLV0_ADDR, 1, &sub_reg, 1, I2C_Timout);
	
	//SET REGISTER TO AK89633 SUB ADDRESS
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, I2C_SLV0_REG, 1, &sub_address, 1, I2C_Timout);
	
	//ENABLE 12C
	sub_reg = I2C_SLV0_EN | count;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, I2C_SLV0_CTRL, 1, &sub_reg, 1, I2C_Timout);
	
	HAL_Delay(1);
	
	//READ BYTES OFF EXT SENS DATA
	HAL_I2C_Mem_Read(I2Cx, MPU_ADDR_LOW, EXT_SENS_DATA_00, 1, data_store, count,I2C_Timout);
}

//CEK NILAI PADA REGISTER WHO AM I MPU
uint8_t CHECK_MPU_WHO_AM_I(I2C_HandleTypeDef *I2Cx, MPU_status_t *get_status)
{
	uint8_t check;
	
	HAL_I2C_Mem_Read(I2Cx, MPU_ADDR_LOW, WHO_AM_I, 1,  &check, 1, I2C_Timout);
	
	get_status->mpu_who_am_i = check;
	
	if(get_status->mpu_who_am_i == MPU9250_fdbckA || get_status->mpu_who_am_i == MPU9250_fdbckB || get_status->mpu_who_am_i == MPU6050_fdbck)
	{
		get_status->gyro_stat = INITIALIZE_GYRO_OK;
	}
	
	else
	{
		get_status->gyro_stat = INITIALIZE_GYRO_FAILED;
	}
	
	return get_status->mpu_who_am_i;
}

//CEK NILAI PADA REGISTER WHO AM I AK8963 (MAGNETOMETER)
uint8_t CHECK_AK8963_WHO_AM_I(I2C_HandleTypeDef *I2Cx, MPU_status_t *get_status)
{
	uint8_t check;
	
	Read_AK8963_Reg(I2Cx, AK8963_WHO_AM_I, 1, buffer);
	
	get_status->ak8963_who_am_i = buffer[0];
	
	if(get_status->ak8963_who_am_i == MPU_MAG_fdbck)
	{
		get_status->mag_stat = INITIALIZE_GYRO_OK;
	}
	
	else
	{
		get_status->mag_stat = INITIALIZE_GYRO_FAILED;
	}
	
	return get_status->ak8963_who_am_i;
}

uint8_t MPU9250_Init(I2C_HandleTypeDef *I2Cx, MPU_status_t *get_status)
{
	uint8_t data;
	
	//SELECT CLOCK GYRO
	data = CLCK_SEL;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, PWR_MGMT_1, 1, &data, 1, I2C_Timout);
	
	//ENABLE MASTER MODE I2C
	data = I2C_MST_EN;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, USER_CTRL, 1, &data, 1, I2C_Timout);
	
	//SET KECEPATAN BUSS KE 400kHz
	data = I2C_MST_CLK;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, I2C_MST_EN, 1, &data, 1, I2C_Timout);
	
	//POWER DOWN MAGNETOMETER
	data = AK8963_PWR_DWN;
	Write_AK8963_Reg(I2Cx, AK8963_CNTL1, data);
	
	//RESET MPU
	data = PWR_RESET;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, PWR_MGMT_1, 1, &data, 1, I2C_Timout);
	
	HAL_Delay(10);
	
	//POWER DOWN MAGNETOMETER
	data = AK8963_RST;
	Write_AK8963_Reg(I2Cx, AK8963_CNTL1, data);
	
	//ENABLE ACCEL & GYRO
	data = ENABLE_ALL;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, PWR_MGMT_2, 1, &data, 1, I2C_Timout);

	//SET GYRO FS SEL KE 2000DPS
	data = GYRO_FS_SEL_2000;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, GYRO_CONFIG_REG, 1,&data, 1, I2C_Timout);
	
	//SET RENTANG PENGUKURAN ACCELEROMETER KE 16g
	data = ACCEL_FS_SEL_16G;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, ACCEL_CONFIG_REG, 1,&data, 1, I2C_Timout);
	
	//SET BANDWITTH 180Hz
	data = DLPS_180HZ;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, ACCEL_CONFIG_REG2, 1,&data, 1, I2C_Timout);
	
	//SAMPLE RATE DIV KE 0
	data = SMPLRT_DIV_0;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, SMPLRT_DIV_REG, 1, &data, 1, I2C_Timout);
	
	//ENABLE MASTER MODE I2C
	data = I2C_MST_EN;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, USER_CTRL, 1, &data, 1, I2C_Timout);
	
	//SET KECEPATAN BUSS KE 400kHz
	data = I2C_MST_CLK;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, I2C_MST_EN, 1, &data, 1, I2C_Timout);
	
	/*MAGNETOMETER KALIBRASI*/
	
	//POWER DOWN MAGNETOMETER
	data = AK8963_PWR_DWN;
	Write_AK8963_Reg(I2Cx, AK8963_CNTL1, data);
	
	HAL_Delay(100);
	
	//SET MAGNETOMETER KE ROM ACCESS
	data = AK8963_FUSE_ROM;
	Write_AK8963_Reg(I2Cx, AK8963_CNTL1, data);
	
	HAL_Delay(100);
	
	//BACA ASA MAGNETOMETER
	Read_AK8963_Reg(I2Cx, AK8963_ASA, 3, Mag_Adjust);
	
	//POWER DOWN MAGNETOMETER
	data = AK8963_PWR_DWN;
	Write_AK8963_Reg(I2Cx, AK8963_CNTL1, data);
	
	HAL_Delay(100);
	
	//PENGUKURAN KONTINU MODE 2
	data = AK8963_CNT_MEAS2;
	Write_AK8963_Reg(I2Cx, AK8963_CNTL1, data);
	
	HAL_Delay(100);
	
	//SELECT CLOCK GYRO
	data = CLCK_SEL;
	HAL_I2C_Mem_Write(I2Cx, MPU_ADDR_LOW, PWR_MGMT_1, 1, &data, 1, I2C_Timout);
	
	//AMBIL 7 BYTE DATA SEBAGAI REFERENCE
	Read_AK8963_Reg(I2Cx, AK8963_XOUT_L, 7, buffer);
	
	get_status->gyro_stat = INITIALIZE_GYRO_OK;
	
	return get_status->gyro_stat;
}

void MPU9250_Read_RawData(I2C_HandleTypeDef *I2Cx, MPU9250_t *DataStruct)
{
	uint32_t micros();
	
	HAL_I2C_Mem_Read(I2Cx, MPU_ADDR_LOW, ACCEL_XOUT_H, 1, buffer, 14, I2C_Timout);
	
	DataStruct->Acc_X_Raw = (int16_t)(buffer[0] << 8  | buffer[1]);
	DataStruct->Acc_Y_Raw = (int16_t)(buffer[2] << 8  | buffer[3]);
	DataStruct->Acc_Z_Raw = (int16_t)(buffer[4] << 8  | buffer[5]);
	
	DataStruct->Temp_Raw = (int16_t)(buffer[6] << 8 | buffer[7]);
	
	DataStruct->Gyro_X_Raw = (int16_t)(buffer[8] << 8  | buffer[9]);
	DataStruct->Gyro_Y_Raw = (int16_t)(buffer[10] << 8  | buffer[11]);
	DataStruct->Gyro_Z_Raw = (int16_t)(buffer[12] << 8 | buffer[13]);
	
	DataStruct->Ax = DataStruct->Acc_X_Raw / aScaleFactor_16G;
	DataStruct->Ay = DataStruct->Acc_Y_Raw / aScaleFactor_16G;
	DataStruct->Az = DataStruct->Acc_Z_Raw / aScaleFactor_16G;
	
	DataStruct->Gx = DataStruct->Gyro_X_Raw - DataStruct->Call_Gx;
	DataStruct->Gy = DataStruct->Gyro_Y_Raw - DataStruct->Call_Gy;
	DataStruct->Gz = DataStruct->Gyro_Z_Raw - DataStruct->Call_Gz;
}

void MPU9250_Read_Data(I2C_HandleTypeDef *I2Cx, MPU9250_t *DataStruct)
{	
	HAL_I2C_Mem_Read(I2Cx, MPU_ADDR_LOW, ACCEL_XOUT_H, 1, buffer, 21, I2C_Timout);
	
	DataStruct->Acc_X_Raw = (int16_t)(buffer[0] << 8  | buffer[1]);
	DataStruct->Acc_Y_Raw = (int16_t)(buffer[2] << 8  | buffer[3]);
	DataStruct->Acc_Z_Raw = (int16_t)(buffer[4] << 8  | buffer[5]);
	
	DataStruct->Gyro_X_Raw = (int16_t)(buffer[8] << 8  | buffer[9]);
	DataStruct->Gyro_Y_Raw = (int16_t)(buffer[10] << 8  | buffer[11]);
	DataStruct->Gyro_Z_Raw = (int16_t)(buffer[12] << 8 | buffer[13]);
	
	DataStruct->MagX = ((int16_t)buffer[15] << 8  | buffer[14]);
	DataStruct->MagY = ((int16_t)buffer[17] << 8  | buffer[16]);
	DataStruct->MagZ = ((int16_t)buffer[19] << 8  | buffer[18]);
	
	DataStruct->Ax = DataStruct->Acc_X_Raw / aScaleFactor_16G;
	DataStruct->Ay = DataStruct->Acc_Y_Raw / aScaleFactor_16G;
	DataStruct->Az = DataStruct->Acc_Z_Raw / aScaleFactor_16G;
	
	DataStruct->Gx = ((DataStruct->Gyro_X_Raw - DataStruct->Call_Gx) / gScaleFactor_2000) * 0.01;
	DataStruct->Gy = ((DataStruct->Gyro_Y_Raw - DataStruct->Call_Gy) / gScaleFactor_2000) * 0.01;
	DataStruct->Gz = ((DataStruct->Gyro_Z_Raw - DataStruct->Call_Gz) / gScaleFactor_2000) * 0.01;
	
	DataStruct->Gx = DataStruct->Gx + (atan2(DataStruct->Ay, DataStruct->Az))*RAD_TO_DEG;
	DataStruct->Gy = DataStruct->Gy + (-(atan2(DataStruct->Ax, sqrt(pow(DataStruct->Ay, 2) + pow(DataStruct->Az, 2)))))*RAD_TO_DEG;
  DataStruct->Gz = DataStruct->Gz + (atan2(DataStruct->Az, sqrt(pow(DataStruct->Ax, 2) + pow(DataStruct->Az, 2))))*RAD_TO_DEG;
	
	DataStruct->Call_MagX = (int16_t)((double)DataStruct->MagX * ((double)(Mag_Adjust[0] - 128) / 256.0f + 1.0f));
	DataStruct->Call_MagY = (int16_t)((double)DataStruct->MagY * ((double)(Mag_Adjust[1] - 128) / 256.0f + 1.0f));
	DataStruct->Call_MagZ = (int16_t)((double)DataStruct->MagZ * ((double)(Mag_Adjust[2] - 128) / 256.0f + 1.0f));
	
	double dt = (double)(HAL_GetTick() - timer) / 1000;
	timer = HAL_GetTick();
	double roll;
	double roll_sqrt = sqrt(DataStruct->Acc_X_Raw * DataStruct->Acc_X_Raw + DataStruct->Acc_Z_Raw * DataStruct->Acc_Z_Raw);
	
	if (roll_sqrt != 0.0)
	{
		roll = atan(DataStruct->Acc_Y_Raw / roll_sqrt) * RAD_TO_DEG;
	}
	else
	{
		roll = 0.0;
	}
	
	double pitch = atan2(-DataStruct->Acc_X_Raw, DataStruct->Acc_Z_Raw) * RAD_TO_DEG;
	
	if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
	{
			KalmanY.angle = pitch;
			DataStruct->KalmanAngleY = pitch;
	}
	else
	{
			DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
	}
	if (fabs(DataStruct->KalmanAngleY) > 90)
			DataStruct->Gx = -DataStruct->Gx;
	DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

void MPU9250_Callibrate_Gyro(I2C_HandleTypeDef *I2Cx, MPU9250_t *DataStruct, uint16_t numCallPoints)
{
	int32_t x = 0;
	int32_t y = 0;
	int32_t z = 0;
	
	uint8_t buffer[14];

	HAL_I2C_Mem_Read(I2Cx, MPU_ADDR_LOW, ACCEL_XOUT_H, 1, buffer, 14, I2C_Timout);
	
	DataStruct->Acc_X_Raw = (int16_t)(buffer[0] << 8  | buffer[1]);
	DataStruct->Acc_Y_Raw = (int16_t)(buffer[2] << 8  | buffer[3]);
	DataStruct->Acc_Z_Raw = (int16_t)(buffer[4] << 8  | buffer[5]);
	
	DataStruct->Gyro_X_Raw = (int16_t)(buffer[8] << 8  | buffer[9]);
	DataStruct->Gyro_Y_Raw = (int16_t)(buffer[10] << 8  | buffer[11]);
	DataStruct->Gyro_Z_Raw = (int16_t)(buffer[12] << 8 | buffer[13]);
	
	if (numCallPoints == 0)
	{
		numCallPoints = 1;
	}
	
	for(uint16_t i=0; i<numCallPoints; i++)
	{
		x += DataStruct->Gyro_X_Raw;
		y += DataStruct->Gyro_Y_Raw;
		z += DataStruct->Gyro_Z_Raw;
		HAL_Delay(1);
	}
	
	DataStruct->Call_Gx = (double)x / (double)numCallPoints;
	DataStruct->Call_Gy = (double)y / (double)numCallPoints;
	DataStruct->Call_Gz = (double)z / (double)numCallPoints;
}
