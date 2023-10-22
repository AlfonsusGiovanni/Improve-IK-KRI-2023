/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "Math.h"
#include "MPU9250_lib.h"
#include "KomunikasiRobot.h"
	
#define nilaiminservo1 	270		/*0 derajat servo 1*/
#define nilaiservo1 		450 	/*90 derajat servo 1*/
#define nilaimaxservo1 	630		/*180 derajat servo 1*/

#define nilaiminservo2  320		/*0 derajat servo 2*/
#define nilaiservo2			500		/*90 derajat servo 2*/
#define nilaimaxservo2	680		/*180 derajat servo 2*/


/*---SETTING ---*/
///////////////////////////////////////////////

//#define BOOTING_ON
//#define ALL_ZERO_POS
//#define CALIBRATION_ON  
//#define TOGGLE_ON

//#define SET_FUNGSI_KOMUNIKASI  
#define SET_FUNGSI_GERAK_TRANSLASI
//#define SET_FUNGSI_GERAK_ROTASI
//#define SET_FUNGSI_GERAK_NANJAK
//#define SET_FUNGSI_GERAKAN
//#define SET_FUNGSI_LEANING
//#define SET_FUNGSI_GERAKCAPIT
//#define SET_FUNGSI_TEST_STABILIZER
//#define SET_FUNGSI_TEST_SEROKAN
//#define SET_FUNGSI_TEST_GYRO
//#define SET_FUNGSI_TEST_PID_GYRO
//#define SET_FUNGSI_TEST_MAG
//#define SET_RANDOM_TESTING

//#define CEK_BODY_NANJAK

//#define CEK_CAPIT_HOME
//#define CEK_CAPIT_STEADY 
//#define CEK_CAPIT_NYAPIT1
//#define CEK_CAPIT_NYAPIT2
//#define CEK_CAPIT_EVAKUASI
//#define CEK_CAPITBAWAH_HOME
//#define CEK_CAPITBAWAH_STEADY 
//#define CEK_CAPITBAWAH_NYAPIT1
//#define CEK_CAPITBAWAH_NYAPIT2
//#define CEK_CAPITBAWAH_EVAKUASI

#ifdef ALL_ZERO_POS
	#define POSISI_NOL
	#define KALIBRASI_CAPIT
#endif


#ifdef CALIBRATION_ON
	#define KALIBRASI_KAKI
	#define KALIBRASI_CAPIT
#endif


#ifdef SET_FUNGSI_KOMUNIKASI
	#define POSISI_STEADY
	#define RX_POS
	#define KOMUNIKASI_ON
	#define GYRO_ON
#endif
 
 
#ifdef SET_FUNGSI_GERAK_TRANSLASI
	#define POSISI_STEADY
	#define NON_INDEPENDENT_POS 
	//#define INDEPENDENT_POS
	#define TEST_TRANSLASI 
#endif


#ifdef SET_FUNGSI_GERAK_ROTASI
	#define POSISI_STEADY
	#define NON_INDEPENDENT_POS 
	#define TEST_ROTASI 
#endif

#ifdef SET_FUNGSI_GERAKAN
	#define POSISI_STEADY
	#define NON_INDEPENDENT_POS 
	#define TEST_GERAKAN 
#endif


#ifdef SET_FUNGSI_GERAK_NANJAK
	#define POSISI_STEADY
	#define TEST_NANJAK 
#endif


#ifdef SET_FUNGSI_LEANING
	#define POSISI_STEADY
	#define NON_INDEPENDENT_POS
	#define TEST_LEANING
#endif


#ifdef SET_FUNGSI_GERAKCAPIT
	#define POSISI_STEADY
	#define NON_INDEPENDENT_POS
	#define TEST_GERAKCAPIT
#endif


#ifdef SET_FUNGSI_TEST_STABILIZER
	#define GYRO_ON
	#define STABILIZER_POS
	#define POSISI_STEADY
	#define TEST_STABILIZER
#endif


#ifdef SET_FUNGSI_TEST_SEROKAN
	#define POSISI_STEADY
	#define NON_INDEPENDENT_POS
	#define TEST_SEROKAN
#endif


#ifdef SET_FUNGSI_TEST_GYRO
	#define GYRO_ON
	#define TEST_GYRO
#endif


#ifdef SET_FUNGSI_TEST_PID_GYRO
	#define GYRO_ON
	#define TEST_PID_GYRO
#endif

#ifdef SET_FUNGSI_TEST_MAG
	#define GYRO_ON
	#define TEST_MAG
#endif

#ifdef SET_RANDOM_TESTING
	#define NON_INDEPENDENT_POS 
	#define POSISI_STEADY
	#define GYRO_ON
#endif

///////////////////////////////////////////////


/*---SETTING PARAMETER---*/
///////////////////////////////////////////////

#define BUKA_CAPIT1 540
#define BUKA_CAPIT2	450

#define TUTUP_CAPIT 290

#define HOME_COXA_CAPIT1 270
#define HOME_COXA_CAPIT2 470
#define HOME_FEMUR_CAPIT1 565
#define HOME_FEMUR_CAPIT2 285

#define STDY_COXA_CAPIT 590
#define STDY_FEMUR_CAPIT 325

#define NYAPIT_COXA_CAPIT 570
#define NYAPIT_FEMUR_CAPIT 305

#define BUKA_CAPIT_BAWAH1 25
#define BUKA_CAPIT_BAWAH2 30
#define TUTUP_CAPIT_BAWAH 65                                                                                                                                             
#define CAPIT_BAWAH_TURUN 25
#define CAPIT_BAWAH_NAIK 75

#define POSISI_STDY_X	90
#define POSISI_STDY_Y	50
#define POSISI_STDY_Z	-90

#define POS_KAL_X	100
#define POS_KAL_Y	50
#define POS_KAL_Z -100

#define KOREKSIZ_SETA 0x01
#define KOREKSIZ_SETB 0x02
#define KOREKSIXY_SETA 0x03
#define KOREKSIXY_SETB 0x04

///////////////////////////////////////////////

/*
COMx = -56,19
COMy = 21,67
COMz = -89,55
*/

/*---LONG---*/
unsigned long 
tick_now = 0, 
prev_tick = 0;


/*---DOUBLE---*/
double 
coxa  	= 53.00,
femur		= 28.00,
tibia		= 90.00,
rad 		= 57.30,
deg 		= 0.0174,
PI 			= 3.14,
offsetsudutfemur1 = 0,
offsetsudutfemur2 = 0,
offsetsuduttibia1 = 0,
offsetsuduttibia2 = 0,
coxa_capit = 107.00,
femur_capit = 113.00,

adjust_posX[6],
adjust_posY[6],
adjust_posZ[6],
posX[6],
posY[6],
posZ[6],
init_posX[6],
init_posY[6],
init_posZ[6],
input_posSTDX,
input_posSTDY,
input_posSTDZ,
posakhirRX[6], move_posRX[6],
posakhirRY[6], move_posRY[6],
posakhirRZ[6], move_posRZ[6],
updateposX[6], updateposY[6], updateposZ[6],
posZ_lean[6],
rx_posSTDX,rx_posSTDY,rx_posSTDZ,
rx_moveposX, rx_moveposY, rx_moveposZ,
rx_setspeed,
rx_leanmode, rx_leanvalXY, rx_leanvalZ,
rx_state_capit_bawah, rx_state_serok,

X0[6], X1[6], Y1[6], Z0[6],
f1[6], f2[6],
A[6],
RX[3][3], RY[3][3], RZ[3][3],
Rxy[3][3], Rxz[3][3], Ryz[3][3], Rxyz[3][3],
bodyX[6], bodyY[6], bodyZ[6],
XP1[6], XP2[6], XP3[6], XP4[6],
YP1[6], YP2[6], YP3[6], YP4[6],
ZP1[6], ZP2[6], ZP3[6], ZP4[6],

theta_C[6], theta_F[6], theta_T[6],
sudutCoxa[6], 
sudutFemur[6],
sudutTibia[6],

dataGX, dataGY, dataGZ,
dataMX, dataMY, dataMZ,
savedataGx, savedataGy, savedataGz,
angleRoll, anglePitch, angleYaw,

Kp = 1.0, 
Ki = 0.0, 
Kd = 0.0,
setRoll_val = 0,
setPitch_val = 0,
newroll_val, newpitch_val,
roll_P, roll_I, roll_D,
pitch_P, pitch_I, pitch_D,
roll_error, last_roll_error,
pitch_error, last_pitch_error,
angleRoll_Corr, anglePitch_Corr,
bodyX_correction[6], 
bodyY_correction[6], 
bodyZ_correction[6],

a, b, c, d,
t,
iterasi;

/*---INTEGER---*/
int                                               
langkah 					= 0,
set_langkah 			= 0,
delay_time 				= 0,
walkpoint 				= 0,
step_sudutRoll 		= 0,
step_sudutPitch 	= 0,
step_sudutYaw 		= 0,
mode_jalan 				= 0,
mode_leaning			= 0,
mode_nanjak				= 0,
run_state 				= 0,
boot_mode 				= 0,
counter						= 0,

deltaCOXA_home_stdy = STDY_COXA_CAPIT - HOME_COXA_CAPIT1,
deltaCOXA_home_start = NYAPIT_COXA_CAPIT - HOME_COXA_CAPIT1,
deltaCOXA_stdy_start = NYAPIT_COXA_CAPIT - STDY_COXA_CAPIT,
deltaCOXA_evakuasihalf_home = HOME_COXA_CAPIT2 - HOME_COXA_CAPIT1,

deltaFEMUR_home_stdy = HOME_FEMUR_CAPIT1 - STDY_FEMUR_CAPIT,
deltaFEMUR_home_start = HOME_FEMUR_CAPIT1 - NYAPIT_FEMUR_CAPIT,
deltaFEMUR_stdy_start = STDY_FEMUR_CAPIT - NYAPIT_FEMUR_CAPIT,
deltaFEMUR_evakuasihalf_home = HOME_FEMUR_CAPIT1 - HOME_FEMUR_CAPIT2,

gyro_status,
mode_rotasi,
capit_pos,
mode_buka,
mode_angkat,
set_speed,
capit_speed,
lean_valXY,
lean_valZ[6],
limit_serokD, limit_serokB,
move_posX, last_move_posX, homing_posX[6], last_posX[6],
move_posY, last_move_posY, homing_posY[6], last_posY[6],
move_posZ, last_move_posZ, homing_posZ[6], last_posZ[6],
move_posY_correction[6],
Y_corection,
i;

/*---UNSIGNED LONG---*/
unsigned long
prev_tick, 
tick_now,
trajectory_delay;

/*---BOOLEAN---*/
bool
home 								= true,
capit_home					= true,
steady 							= true,
boot 								= true,
bawa_korban					= false,
walking 						= false,
walking_nanjak 			= false,
gerak_tripod 				= false,
gerak_rotasi 				= false,
gerak_halftripod		= false,
lean_limit 					= false,
rotasi_limit				= false,
nanjak 							= false,
leaning							= false,
capit_stdy					= false,
capit_nyapit				= false,
capit_evakuasi 			= false,
capit_evakuasihalf	= false,
capit_limit 				= false,
boot_limit					= false,
ping_atas						= false,
walkA								= false, 
walkB								= false,
stop_state					= false;

/*---TYPEDEF STRUCT---*/
MPU9250_t MPU9250;
MPU_status_t get_status;
feedback_t fdbck_atas;
com_get_t data_rx;

/*---BEGIN SET VARIABLE---*/
/*KOREKSI SUDUT ERROR SERVO*/
//int Servoiniterror[18] =
//{
//	/*KANAN DEPAN*/
//	25, -85, -115, 
//	/*KANAN TENGAH*/
//	5, -55, -115,
//	/*KANAN BELAKANG*/
//	-15, -63, -115,
//	
//	/*KIRI BELAKANG*/
//	20, 60, 130,
//	/*KIRI TENGAH*/
//	0, 25, 125,
//	/*KIRI DEPAN*/
//	-15, 95, 135   
//};

int Servoiniterror[18] =
{
	/*KANAN DEPAN*/
	-8, -190, -200,
	/*KANAN TENGAH*/
	3, -150, -170,
	/*KANAN BELAKANG*/
	20, -145, -190,
	
	/*KIRI BELAKANG*/
	-12, 190, 210,
	/*KIRI TENGAH*/
	0, 160, 190,
	/*KIRI DEPAN*/
	15, 205, 200
};

/*OFFSET KAKI->BODY ROBOT SUMBU X*/
double offsetbadanX[6] = 
{
	/*KANAN DEPAN*/
	33.935,
	/*KANAN TENGAH*/
	37.65,
	/*KANAN BELAKANG*/
	33.935,
	
	/*KIRI BELAKANG*/
	-33.935,
	/*KIRI TENGAH*/
  -37.65,
  /*KIRI DEPAN*/
	-33.935
};

/*OFFSET KAKI->BODY ROBOT SUMBU Y*/
double offsetbadanY[6] =
{
	/*KANAN DEPAN*/
	73.94,
	/*KANAN TENGAH*/
	0,
	/*KANAN BELAKANG*/
	-73.94,
	
	/*KIRI BELAKANG*/
	-73.94,
	/*KIRI TENGAH*/
  0,
	/*KIRI DEPAN*/
	73.94
};

/*OFFSET KAKI->BODY ROBOT SUMBU Z*/
double offsetbadanZ[6] =
{
	/*KIRI DEPAN*/
	10,
	/*KIRI TENGAH*/
	10,
	/*KIRI BELAKANG*/
	10,

	/*KANAN DEPAN*/
	10,
	/*KANAN TENGAH*/
	10,
	/*KANAN BELAKANG*/
	10
};

/*---END SET VARIABLE---*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
/*---UART HANDLER---*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//data_rx = (com_get_t){0};
	rx_get(&data_rx);
}

/*---CHECK ERROR SUDUT MPU9250---*/
void check_angle_err()
{
	MPU9250_Read_Data(&hi2c1, &MPU9250);
	
	dataGX = MPU9250.KalmanAngleX;
	dataGY = MPU9250.KalmanAngleY;
	dataGZ = MPU9250.Gx;
	
	pitch_error = setRoll_val - dataGX;
	roll_error = setPitch_val - dataGY;
}

/*---KALKULASI PID STABILIZER---*/
void calculate_pid()
{
	roll_P = roll_error, 											pitch_P = pitch_error;
	roll_I = roll_I + last_roll_error;				pitch_I = pitch_I + last_pitch_error;
	roll_D = roll_error - last_roll_error,		pitch_D = pitch_error - last_pitch_error;
	
	angleRoll_Corr = (Kp*roll_P + Ki*roll_I + Kd*roll_D) * -1;
	anglePitch_Corr = (Kp*pitch_P + Ki*pitch_I + Kd*pitch_D) * -1;
	
	last_roll_error = roll_error;
	last_pitch_error = pitch_error;
	
	newroll_val = setRoll_val + angleRoll_Corr;
	newpitch_val = setPitch_val + anglePitch_Corr;
}

/*---TITIK AWAL TIAP KAKI---*/
void titik_awal(double input_posSTDX, double input_posSTDY, double input_posSTDZ, int mode)
{
	switch(mode)
	{
		/*STEADY NORMAL*/
		case 1:
			adjust_posX[5] = -input_posSTDX,				adjust_posX[0] = input_posSTDX;
			adjust_posY[5] = input_posSTDY,					adjust_posY[0] = input_posSTDY;
			adjust_posZ[5] = input_posSTDZ,					adjust_posZ[0] = input_posSTDZ;

			adjust_posX[4] = -input_posSTDX,				adjust_posX[1] = input_posSTDX;
			adjust_posY[4] = 0,											adjust_posY[1] = 0;
			adjust_posZ[4] = input_posSTDZ,					adjust_posZ[1] = input_posSTDZ;

			adjust_posX[3] = -input_posSTDX,				adjust_posX[2] = input_posSTDX;
			adjust_posY[3] = -input_posSTDY,				adjust_posY[2] = -input_posSTDY;
			adjust_posZ[3] = input_posSTDZ,					adjust_posZ[2] = input_posSTDZ;
			break;
		
		/*STABILIZER ROLL*/
		case 2:
			check_angle_err();
			calculate_pid();
		
			if(newroll_val >= -15 && newroll_val <= 15)
			{		
				for(i=0; i<6; i++)
				{
						if(i<3)
						{
							bodyX_correction[i] = newroll_val/2;
							bodyZ_correction[i] = -newroll_val;
							
							updateposX[i] = bodyX_correction[i];
							updateposZ[i] = bodyZ_correction[i];
						}
						
						else
						{
							bodyX_correction[i] = -newroll_val/2;
							bodyZ_correction[i] = newroll_val;
							
							updateposX[i] = bodyX_correction[i];
							updateposZ[i] = bodyZ_correction[i];
						}
				}
				
				adjust_posX[5] = -input_posSTDX,				adjust_posX[0] = input_posSTDX ;
				adjust_posY[5] = input_posSTDY,					adjust_posY[0] = input_posSTDY;
				adjust_posZ[5] = input_posSTDZ,					adjust_posZ[0] = input_posSTDZ;

				adjust_posX[4] = -input_posSTDX,				adjust_posX[1] = input_posSTDX;
				adjust_posY[4] = 0,											adjust_posY[1] = 0;
				adjust_posZ[4] = input_posSTDZ,					adjust_posZ[1] = input_posSTDZ;

				adjust_posX[3] = -input_posSTDX,				adjust_posX[2] = input_posSTDX;
				adjust_posY[3] = -input_posSTDY,				adjust_posY[2] = -input_posSTDY;
				adjust_posZ[3] = input_posSTDZ,					adjust_posZ[2] = input_posSTDZ;
			}
			break;
	}
}

/*---KONTROL POSISI TIAP KAKI ROBOT---*/
void independent_pos(int no_kaki, double ind_posX, double ind_posY, double ind_posZ)
{
	adjust_posX[no_kaki] = ind_posX;
	adjust_posY[no_kaki] = ind_posY;
	adjust_posZ[no_kaki] = ind_posZ;
}

/*---POSISI NOL DERAJAT SERVO---*/
void pos_nol()
{
	/*KANAN DEPAN*/																								
	htim11.Instance->CCR1 = nilaimaxservo2-10; //180 derjat	
	htim10.Instance->CCR1 = nilaimaxservo1; 	//180 derajat		
	htim3.Instance->CCR2  = nilaimaxservo1; 	//180 derajat
	
	/*KANAN TENGAH*/
	htim3.Instance->CCR1 	= nilaiservo2; 			//90 derajat
	htim2.Instance->CCR2 	= nilaimaxservo1; 	//180 derajat
	htim2.Instance->CCR1 	= nilaimaxservo1; 	//180 derajat
	
	/*KANAN BELAKANG*/	
	htim1.Instance->CCR4 	= nilaiminservo2+5; //0 derajat
	htim1.Instance->CCR2 	= nilaimaxservo1; 	//180 derajat
	htim1.Instance->CCR1 	= nilaimaxservo1; 	//180 derajat
	
	/*KIRI BELAKANG*/																								
	htim3.Instance->CCR3 = nilaimaxservo2-10; //180 derjat			
	htim8.Instance->CCR2 	= nilaiminservo1;		//0 derjat		
	htim1.Instance->CCR3 	= nilaiminservo1;		//0 derjat
	
	/*KIRI TENGAH*/																								
	htim5.Instance->CCR1 	= nilaiservo2;			//90 derajat			
	htim2.Instance->CCR4 	= nilaiminservo1;		//0 derjat		
	htim5.Instance->CCR2 	= nilaiminservo1;		//0 derjat
	
	/*KIRI DEPAN*/
	htim13.Instance->CCR1 = nilaiminservo2+5;  //0 derajat
	htim9.Instance->CCR2 	= nilaiminservo1;		//0 derjat
	htim2.Instance->CCR3 	= nilaiminservo1;		//0 derjat
}

/*---POSISI AWAL TIAP KAKI---*/
void poskaki_awal()
{
	/*KANAN DEPAN*/
	init_posX[0] = adjust_posX[0] + updateposX[0];
	init_posY[0] = adjust_posY[0] + updateposY[0]; 
	init_posZ[0] = adjust_posZ[0] + updateposZ[0];

	/*KANAN TENGAH*/
	init_posX[1] = adjust_posX[1] + updateposX[1];
	init_posY[1] = adjust_posY[1] + updateposY[1];
	init_posZ[1] = adjust_posZ[1] + updateposZ[1];
	
	/*KANAN BELAKANG*/
	init_posX[2] = adjust_posX[2] + updateposX[2];
	init_posY[2] = adjust_posY[2] + updateposY[2];
	init_posZ[2] = adjust_posZ[2] + updateposZ[2];
	
	/*KIRI BELAKANG*/
	init_posX[3] = adjust_posX[3] + updateposX[3];
	init_posY[3] = adjust_posY[3] + updateposY[3];
	init_posZ[3] = adjust_posZ[3] + updateposZ[3];
	
	/*KIRI TENGAH*/
	init_posX[4] = adjust_posX[4] + updateposX[4];
	init_posY[4] = adjust_posY[4] + updateposY[4];
 	init_posZ[4] = adjust_posZ[4] + updateposZ[4];
	
	/*KIRI DEPAN*/
	init_posX[5] = adjust_posX[5] + updateposX[5];
	init_posY[5] = adjust_posY[5] + updateposY[5];
	init_posZ[5] = adjust_posZ[5] + updateposZ[5];
}

/*---BODYKINEMATIC AWAL---*/
void bodykinematic()
{
	/*KANAN DEPAN*/
	Y1[0] = init_posY[0] + posY[0];
	X0[0] = init_posX[0] + posX[0];
	Z0[0] = init_posZ[0] + posZ[0];
	X1[0] = sqrt((pow(X0[0], 2) + pow(Y1[0], 2)));
	
	/*KANAN TENGAH*/
	Y1[1] = init_posY[1] + posY[1];
	X0[1] = init_posX[1] + posX[1];
	Z0[1] = init_posZ[1] + posZ[1];
	X1[1] = sqrt((pow(X0[1], 2) + pow(Y1[1], 2)));
	
	/*KANAN BELAKANG*/
	Y1[2] = init_posY[2] + posY[2];
	X0[2] = init_posX[2] + posX[2];
	Z0[2] = init_posZ[2] + posZ[2];
	X1[2] = sqrt((pow(X0[2], 2) + pow(Y1[2], 2)));
	
	/*KIRI BELAKANG*/
	Y1[3] = init_posY[3] + posY[3];
	X0[3] = init_posX[3] + posX[3];
	Z0[3] = init_posZ[3] + posZ[3];
	X1[3] = sqrt((pow(X0[3], 2) + pow(Y1[3], 2)));
	
	/*KIRI TENGAH*/
	Y1[4] = init_posY[4] + posY[4];
	X0[4] = init_posX[4] + posX[4];
	Z0[4] = init_posZ[4] + posZ[4];
	X1[4] = sqrt((pow(X0[4], 2) + pow(Y1[4], 2)));
	
	/*KIRI DEPAN*/
	Y1[5] = init_posY[5] + posY[5];
	X0[5] = init_posX[5] + posX[5];
	Z0[5] = init_posZ[5] + posZ[5];
	X1[5] = sqrt((pow(X0[5], 2) + pow(Y1[5], 2)));
	
	for(i=0; i<6; i++)
	{
		bodyY[i] = Y1[i] + offsetbadanY[i];
  	bodyX[i] = X0[i] + offsetbadanX[i];
  	bodyZ[i] = Z0[i] + offsetbadanZ[i];
	}
}

/*---INVERSE KINEMATIC AKHIR---*/
void inverse_akhir()
{
	for(i=0; i<6; i++)
	{
		bodyY[i] = bodyY[i] - offsetbadanY[i];
		bodyX[i] = bodyX[i] - offsetbadanX[i];
		bodyZ[i] = bodyZ[i] - offsetbadanZ[i];

		X0[i] = bodyX[i];
		Y1[i] = bodyY[i];
		Z0[i] = bodyZ[i];
	}
}

/*---INVERSE KINEMATIC KAKI---*/
void inverse_kinematic()
{
	for(i=0; i<6; i++)
  {
		A[i]  = sqrt((pow(Z0[i], 2) + pow((X1[i] - coxa), 2)));
		f1[i] = atan((Z0[i]/(X1[i] - coxa)));
		f2[i] = acos(((pow(A[i],2) + pow(femur,2)  - pow(tibia,2))/(2*A[i]*femur))); 
			
		theta_C[i] = atan(Y1[i]/adjust_posX[i])*rad;
		theta_F[i] = (f1[i] + f2[i])*rad;        
		theta_T[i] = acos(((pow(A[i],2) - pow(femur,2) - pow(tibia,2))/(2*femur*tibia)))*rad;		
//		theta_T[i] = acos(((pow(femur,2) + pow(tibia,2) - pow(A[i],2))/(2*femur*tibia)))*rad;
  }
}

/*---SERVO ANGLE WRITE---*/
void servowrite()
{
	/*KANAN DEPAN*/
	sudutCoxa[0] 	= 180 - (theta_C[0]);
	sudutFemur[0]	= 180 + (theta_F[0] - offsetsudutfemur1);
	sudutTibia[0] = 90 + (theta_T[0] - offsetsuduttibia1);
	htim11.Instance->CCR1 = sudutCoxa[0]*2 + nilaiminservo2 + Servoiniterror[0];
	htim10.Instance->CCR1 = sudutFemur[0]*2 + nilaiminservo1 + Servoiniterror[1];
	htim3.Instance->CCR2  = sudutTibia[0]*2 + nilaiminservo1 + Servoiniterror[2];
	
	/*KANAN TENGAH*/
	sudutCoxa[1] 	= 90 - theta_C[1];
	sudutFemur[1]	= 180 + (theta_F[1] - offsetsudutfemur1);
	sudutTibia[1]	= 90 + (theta_T[1] - offsetsuduttibia1);
	htim3.Instance->CCR1 = sudutCoxa[1]*2 + nilaiminservo2 + Servoiniterror[3];
	htim2.Instance->CCR2 = sudutFemur[1]*2 + nilaiminservo1 + Servoiniterror[4];
	htim2.Instance->CCR1 = sudutTibia[1]*2 + nilaiminservo1 + Servoiniterror[5];
	
	/*KANAN BELAKANG*/
	sudutCoxa[2] 	= 180 - (theta_C[2] + 180);
	sudutFemur[2]	= 180 + (theta_F[2] - offsetsudutfemur1);
	sudutTibia[2]	= 90 + (theta_T[2] - offsetsuduttibia1);
	htim1.Instance->CCR4 = sudutCoxa[2]*2 + nilaiminservo2 + Servoiniterror[6];
	htim1.Instance->CCR2 = sudutFemur[2]*2 + nilaiminservo1 + Servoiniterror[7];
	htim1.Instance->CCR1 = sudutTibia[2]*2 + nilaiminservo1 + Servoiniterror[8];
	
	
	/*KIRI BELAKANG*/																																		
	sudutCoxa[3] 	= 180 - (theta_C[3]);
	sudutFemur[3] = -1*(theta_F[3] - offsetsudutfemur2);
	sudutTibia[3] = 90 - theta_T[3] - offsetsuduttibia2;
	htim3.Instance->CCR3 = sudutCoxa[3]*2 + nilaiminservo2 + Servoiniterror[9];					
	htim8.Instance->CCR2  = sudutFemur[3]*2 + nilaiminservo1 + Servoiniterror[10];				
	htim1.Instance->CCR3  = sudutTibia[3]*2 + nilaiminservo1 + Servoiniterror[11];	
	
	/*KIRI TENGAH*/																																			
	sudutCoxa[4] 	= 180 - (theta_C[4] + 90);																								
	sudutFemur[4] = -1*(theta_F[4] - offsetsudutfemur2);																	
	sudutTibia[4] = 90 - theta_T[4] - offsetsuduttibia2;
	htim5.Instance->CCR1 = sudutCoxa[4]*2 + nilaiminservo2 + Servoiniterror[12];
	htim2.Instance->CCR4 = sudutFemur[4]*2 + nilaiminservo1 + Servoiniterror[13];
	htim5.Instance->CCR2 = sudutTibia[4]*2 + nilaiminservo1 + Servoiniterror[14];
	 
	/*KIRI DEPAN*/																																			
	sudutCoxa[5] 	= 180 - (theta_C[5] + 180);																								
	sudutFemur[5] = -1*(theta_F[5] - offsetsudutfemur2);
	sudutTibia[5] = 90 - theta_T[5] - offsetsuduttibia2;										
	htim13.Instance->CCR1 = sudutCoxa[5]*2 + nilaiminservo2 + Servoiniterror[15];
	htim9.Instance->CCR2 = sudutFemur[5]*2 + nilaiminservo1 + Servoiniterror[16];
	htim2.Instance->CCR3 = sudutTibia[5]*2 + nilaiminservo1 + Servoiniterror[17];
}


/*---MATRIKS ROTASI---*/
void matriks(float angleRoll, float angglePitch, float angleYaw)
{		
	RX[0][0] = 1,                       				RX[0][1] = 0,                       			RX[0][2] = 0;
	RX[1][0] = 0,                       				RX[1][1] = cos(angleRoll*deg), 						RX[1][2] = -sin(angleRoll*deg);
	RX[2][0] = 0,                       				RX[2][1] = sin(angleRoll*deg), 						RX[2][2] = cos(angleRoll*deg);

	RY[0][0] = cos(anglePitch*deg),  						RY[0][1] = 0,                       			RY[0][2] = sin(anglePitch*deg);
	RY[1][0] = 0,                       				RY[1][1] = 1,                       			RY[1][2] = 0;
	RY[2][0] = -sin(anglePitch*deg),						RY[2][1] = 0,                       			RY[2][2] = cos(anglePitch*deg);

	RZ[0][0] = cos(angleYaw*deg),  							RZ[0][1] = -sin(angleYaw*deg), 						RZ[0][2] = 0;
	RZ[1][0] = sin(angleYaw*deg),  							RZ[1][1] = cos(angleYaw*deg),  						RZ[1][2] = 0;
	RZ[2][0] = 0,                       				RZ[2][1] = 0,                       			RZ[2][2] = 1;

	
	Rxy[0][0] = (RX[0][0]*RY[0][0] + RX[0][1]*RY[1][0] + RX[0][2]*RY[2][0]);
	Rxy[1][0] = (RX[1][0]*RY[0][0] + RX[1][1]*RY[1][0] + RX[1][2]*RY[2][0]);
	Rxy[2][0] = (RX[2][0]*RY[0][0] + RX[2][1]*RY[1][0] + RX[2][2]*RY[2][0]);
	
	Rxy[0][1] = (RX[0][0]*RY[0][1] + RX[0][1]*RY[1][1] + RX[0][2]*RY[2][1]);
	Rxy[1][1] = (RX[1][0]*RY[0][1] + RX[1][1]*RY[1][1] + RX[1][2]*RY[2][1]);
	Rxy[2][1] = (RX[2][0]*RY[0][1] + RX[2][1]*RY[1][1] + RX[2][2]*RY[2][1]);

	Rxy[0][2] = (RX[0][0]*RY[0][2] + RX[0][1]*RY[1][2] + RX[0][2]*RY[2][2]);
	Rxy[1][2] = (RX[1][0]*RY[0][2] + RX[1][1]*RY[1][2] + RX[1][2]*RY[2][2]);
	Rxy[2][2] = (RX[2][0]*RY[0][2] + RX[2][1]*RY[1][2] + RX[2][2]*RY[2][2]);
		
			
	Rxyz[0][0] = Rxy[0][0]*RZ[0][0] + Rxy[0][1]*RZ[1][0] + Rxy[0][2]*RZ[2][0];
	Rxyz[1][0] = Rxy[1][0]*RZ[0][0] + Rxy[1][1]*RZ[1][0] + Rxy[1][2]*RZ[2][0];
	Rxyz[2][0] = Rxy[2][0]*RZ[0][0] + Rxy[2][1]*RZ[1][0] + Rxy[2][2]*RZ[2][0];
	
	Rxyz[0][1] = Rxy[0][0]*RZ[0][1] + Rxy[0][1]*RZ[1][1] + Rxy[0][2]*RZ[2][1];
	Rxyz[1][1] = Rxy[1][0]*RZ[0][1] + Rxy[1][1]*RZ[1][1] + Rxy[1][2]*RZ[2][1];
	Rxyz[2][1] = Rxy[2][0]*RZ[0][1] + Rxy[2][1]*RZ[1][1] + Rxy[2][2]*RZ[2][1];
	
	Rxyz[0][2] = Rxy[0][0]*RZ[0][2] + Rxy[0][1]*RZ[1][2] + Rxy[0][2]*RZ[2][2];
	Rxyz[1][2] = Rxy[1][0]*RZ[0][2] + Rxy[1][1]*RZ[1][2] + Rxy[1][2]*RZ[2][2];
	Rxyz[2][2] = Rxy[2][0]*RZ[0][2] + Rxy[2][1]*RZ[1][2] + Rxy[2][2]*RZ[2][2];
	
	
	for(i=0; i<6; i++)
	{
		posakhirRX[i] = (Rxyz[0][0]*(adjust_posX[i] + offsetbadanX[i])) + (Rxyz[0][1]*(adjust_posY[i] + offsetbadanY[i])) + (Rxyz[0][2]*(adjust_posZ[i] + offsetbadanZ[i]));
		posakhirRY[i] = (Rxyz[1][0]*(adjust_posX[i] + offsetbadanX[i])) + (Rxyz[1][1]*(adjust_posY[i] + offsetbadanY[i])) + (Rxyz[1][2]*(adjust_posZ[i] + offsetbadanZ[i]));
		posakhirRZ[i] = (Rxyz[2][0]*(adjust_posX[i] + offsetbadanX[i])) + (Rxyz[2][1]*(adjust_posY[i] + offsetbadanY[i])) + (Rxyz[2][2]*(adjust_posZ[i] + offsetbadanZ[i]));
		
		move_posRX[i] = posakhirRX[i] - adjust_posX[i] - offsetbadanX[i];
		move_posRY[i] = posakhirRY[i] - adjust_posY[i] - offsetbadanY[i];
		move_posRZ[i] = posakhirRZ[i] - adjust_posZ[i] - offsetbadanZ[i];		
	}
}

/*---POSISI STEADY ROBOT---*/
void pos_steady()
{
	steady = true;
	stop_state = false;
	
	poskaki_awal();
	bodykinematic();
	inverse_akhir();
	inverse_kinematic();
	servowrite();
	
	for(i=0; i<6; i++) 
	{
		last_posX[i] = init_posX[i] - X0[i];
		last_posY[i] = init_posY[i] - Y1[i];
		last_posZ[i] = init_posZ[i] - Z0[i];
	}
}

/*---POSISI KALIBRASI ROBOT---*/
void pos_kal()
{
	titik_awal(POS_KAL_X, POS_KAL_Y, POS_KAL_Z, 1);
	
	poskaki_awal();
	bodykinematic();
	inverse_akhir();
	inverse_kinematic();
	servowrite();
}

/*---POSISI MENANJAK---*/
void pos_nanjak(int mode_nanjak)
{
	switch(mode_nanjak)
	{
		/*NANJAK KANAN*/
		case 0:
			/*Kanan Depan*/
			independent_pos(0, 90, 70, -90);
			/*Kanan Tengah*/
			independent_pos(1, 90, 0, -90); 
			/*Kanan Belakang*/
			independent_pos(2, 90, -70, -90); 
			/*Kiri Belakang*/
			independent_pos(3, -90, -70, -90); 
			/*Kiri Tengah*/
			independent_pos(4, -90, 0, -90); 
			/*Kiri Depan*/
			independent_pos(5, -90, 70, -90);
			break;
		
		/*NANJAK KIRI*/
		case 1:
			/*Kanan Depan*/
			independent_pos(0, 90, 70, -90);
			/*Kanan Tengah*/
			independent_pos(1, 90, 0, -90); 
			/*Kanan Belakang*/
			independent_pos(2, 90, -70, -90); 
			/*Kiri Belakang*/
			independent_pos(3, -90, -70, -90); 
			/*Kiri Tengah*/
			independent_pos(4, -90, 0, -90); 
			/*Kiri Depan*/
			independent_pos(5, -90, 70, -90);
			break;
		
		/*SI BAGONG MIRING KANAN*/
		case 2:
			/*Kanan Depan*/
			independent_pos(0, 80, 70, -80);
			/*Kanan Tengah*/
			independent_pos(1, 90, 0, -80); 
			/*Kanan Belakang*/
			independent_pos(2, 80, -70, -80); 
			/*Kiri Belakang*/
			independent_pos(3, -80, -70, -100); 
			/*Kiri Tengah*/
			independent_pos(4, -80, 0, -100); 
			/*kiri Depan*/
			independent_pos(5, -80, 70, -100);
			break;
		
		/*SI BAGONG MIRING KIRI*/
		case 3:
			/*Kanan Depan*/
			independent_pos(0, 80, 70, -100);
			/*Kanan Tengah*/
			independent_pos(1, 90, 0, -100); 
			/*Kanan Belakang*/
			independent_pos(2, 80, -70, -100); 
			/*Kiri Belakang*/
			independent_pos(3, -80, -70, -80); 
			/*Kiri Tengah*/
			independent_pos(4, -80, 0, -80); 
			/*kiri Depan*/
			independent_pos(5, -80, 70, -80);
			break;
	}
	
	poskaki_awal();
	bodykinematic();
	inverse_akhir();
	inverse_kinematic();
	servowrite();
}

/*---PENYEDERHANAAN POLINOM---*/
void polinom(double t)
{
	a = pow((1-t),3);
  b = 3*t*pow((1-t),2);
  c = 3*pow(t,2)*(1-t);
  d = pow(t,3);
}

/*---TRAYEKTORI GERAK TRIPOD---*/
void trayektori_tripod(int move_posX, int move_posY, int move_posZ, double t, int walkpoint)
{
	polinom(t);
	
	switch(walkpoint)
	{
		case 1:
			for(i=0; i<6; i++)
			{
				switch(i)
				{
					case 0:
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY,       ZP3[i] = move_posZ;
						XP4[i] = move_posX,       YP4[i] = move_posY,       ZP4[i] = 0;
						break;
					
					case 1:
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY,      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY,      ZP4[i] = 0;
						break;
					
					case 2:
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY,       ZP3[i] = move_posZ;
						XP4[i] = move_posX,       YP4[i] = move_posY,       ZP4[i] = 0;
						break;
					
					case 3:
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY,      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY,      ZP4[i] = 0;
						break;
					
					case 4:
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY,       ZP3[i] = move_posZ;
						XP4[i] = move_posX,       YP4[i] = move_posY,       ZP4[i] = 0;
						break;
					
					case 5:
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY,      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY,      ZP4[i] = 0;
						break;
				}
			}
			break;
		
		case 2:
			for(i=0; i<6; i++)
			{
				switch(i)
				{
					case 0:
						XP1[i] = move_posX,   		YP1[i] = move_posY,   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY,   		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY,      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY,      ZP4[i] = 0;
						break;
					
					case 1:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY,   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY,   		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY,      	ZP3[i] = move_posZ;
						XP4[i] = move_posX,      	YP4[i] = move_posY,      	ZP4[i] = 0;
						break;
					
					case 2:
						XP1[i] = move_posX,   		YP1[i] = move_posY,   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY,   		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY,      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY,      ZP4[i] = 0;
						break;
					
					case 3:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY,   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY,   		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY,      	ZP3[i] = move_posZ;
						XP4[i] = move_posX,      	YP4[i] = move_posY,      	ZP4[i] = 0;
						break;
					
					case 4:
						XP1[i] = move_posX,   		YP1[i] = move_posY,   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY,   		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY,      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY,      ZP4[i] = 0;
						break;
					
					case 5:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY,   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY,   		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY,      	ZP3[i] = move_posZ;
						XP4[i] = move_posX,      	YP4[i] = move_posY,      	ZP4[i] = 0;
						break;
				}
			}
			break;
		
		case 3:
			for(i=0; i<6; i++)
			{
				switch(i)
				{
					case 0:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY,   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY,   		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY,      	ZP3[i] = move_posZ;
						XP4[i] = move_posX,      	YP4[i] = move_posY,      	ZP4[i] = 0;
						break;
					
					case 1:
						XP1[i] = move_posX,   		YP1[i] = move_posY,   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY,   		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY,      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY,      ZP4[i] = 0;
						break;
					
					case 2:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY,   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY,   		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY,      	ZP3[i] = move_posZ;
						XP4[i] = move_posX,      	YP4[i] = move_posY,      	ZP4[i] = 0;
						break;
					
					case 3:
						XP1[i] = move_posX,   		YP1[i] = move_posY,   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY,   		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY,      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY,      ZP4[i] = 0;
						break;
					
					case 4:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY,   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY,   		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY,      	ZP3[i] = move_posZ;
						XP4[i] = move_posX,      	YP4[i] = move_posY,      	ZP4[i] = 0;
						break;
					
					case 5:
						XP1[i] = move_posX,   		YP1[i] = move_posY,   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY,   		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY,      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY,      ZP4[i] = 0;
						break;
				}
			}
			break;
		
		case 4:
			for(i=0; i<6; i++)
			{
				switch(i)
				{
					case 0:
						XP1[i] = move_posX,   		YP1[i] = move_posY,   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY,   		ZP2[i] = 0;
						XP3[i] = 0,       				YP3[i] = 0,       			  ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,				        ZP4[i] = 0;
						break;
					
					case 1:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY,   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY,   		ZP2[i] = move_posZ;
						XP3[i] = 0,       				YP3[i] = 0,       			  ZP3[i] = move_posZ;
						XP4[i] = 0,       				YP4[i] = 0,				        ZP4[i] = 0;
						break;
					
					case 2:
						XP1[i] = move_posX,   		YP1[i] = move_posY,   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY,   		ZP2[i] = 0;
						XP3[i] = 0,       				YP3[i] = 0,       			  ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,				        ZP4[i] = 0;
						break;
					
					case 3:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY,   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY,   		ZP2[i] = move_posZ;
						XP3[i] = 0,       				YP3[i] = 0,       			  ZP3[i] = move_posZ;
						XP4[i] = 0,       				YP4[i] = 0,				        ZP4[i] = 0;
						break;
					
					case 4:
						XP1[i] = move_posX,   		YP1[i] = move_posY,   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY,   		ZP2[i] = 0;
						XP3[i] = 0,       				YP3[i] = 0,       			  ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,				        ZP4[i] = 0;
						break;
					
					case 5:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY,   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY,   		ZP2[i] = move_posZ;
						XP3[i] = 0,       				YP3[i] = 0,       			  ZP3[i] = move_posZ;
						XP4[i] = 0,       				YP4[i] = 0,				        ZP4[i] = 0;
						break;
				}
			}
			break;
	}
	
	posX[5] = (a*XP1[5])+(b*XP2[5])+(c*XP3[5])+(d*XP4[5]),			posX[0] = (a*XP1[0])+(b*XP2[0])+(c*XP3[0])+(d*XP4[0]);
	posY[5] = (a*YP1[5])+(b*YP2[5])+(c*YP3[5])+(d*YP4[5]),			posY[0] = (a*YP1[0])+(b*YP2[0])+(c*YP3[0])+(d*YP4[0]);
	posZ[5] = (a*ZP1[5])+(b*ZP2[5])+(c*ZP3[5])+(d*ZP4[5]),			posZ[0] = (a*ZP1[0])+(b*ZP2[0])+(c*ZP3[0])+(d*ZP4[0]);
	
	posX[4] = (a*XP1[4])+(b*XP2[4])+(c*XP3[4])+(d*XP4[4]),			posX[1] = (a*XP1[1])+(b*XP2[1])+(c*XP3[1])+(d*XP4[1]);
	posY[4] = (a*YP1[4])+(b*YP2[4])+(c*YP3[4])+(d*YP4[4]),			posY[1] = (a*YP1[1])+(b*YP2[1])+(c*YP3[1])+(d*YP4[1]);
	posZ[4] = (a*ZP1[4])+(b*ZP2[4])+(c*ZP3[4])+(d*ZP4[4]),			posZ[1] = (a*ZP1[1])+(b*ZP2[1])+(c*ZP3[1])+(d*ZP4[1]);
	
	posX[3] = (a*XP1[3])+(b*XP2[3])+(c*XP3[3])+(d*XP4[3]),			posX[2] = (a*XP1[2])+(b*XP2[2])+(c*XP3[2])+(d*XP4[2]);
	posY[3] = (a*YP1[3])+(b*YP2[3])+(c*YP3[3])+(d*YP4[3]),			posY[2] = (a*YP1[2])+(b*YP2[2])+(c*YP3[2])+(d*YP4[2]);
	posZ[3] = (a*ZP1[3])+(b*ZP2[3])+(c*ZP3[3])+(d*ZP4[3]),			posZ[2] = (a*ZP1[2])+(b*ZP2[2])+(c*ZP3[2])+(d*ZP4[2]);
	
	poskaki_awal();
	bodykinematic();
	inverse_akhir();
	inverse_kinematic();
	servowrite();
}

/*---TRAYEKTORI STOP GERAK TRIPOD GAIT---*/
void trayektori_tripod_stop(double t, int walkpoint)
{
	polinom(t);
	
	switch(walkpoint)
	{
		case KOREKSIZ_SETA:
			for(i=0; i<6; i++)
			{
				if(i==0 || i==2 || i==4)
				{
					
					XP1[i] = posX[i],			YP1[i] = posY[i],      	ZP1[i] = posZ[i];
					XP2[i] = posX[i],    	YP2[i] = posY[i],				ZP2[i] = posZ[i];
					XP3[i] = 0,						YP3[i] = 0,							ZP3[i] = 0;
					XP4[i] = 0,           YP4[i] = 0,           	ZP4[i] = 0;
					
					posX[i] = (a*XP1[i])+(b*XP2[i])+(c*XP3[i])+(d*XP4[i]);
					posY[i] = (a*YP1[i])+(b*YP2[i])+(c*YP3[i])+(d*YP4[i]);
					posZ[i] = (a*ZP1[i])+(b*ZP2[i])+(c*ZP3[i])+(d*ZP4[i]);
				}
			}
			break;
			
		case KOREKSIZ_SETB:
			for(i=0; i<6; i++)
			{
				if(i==1 || i==3 || i==5)
				{
					XP1[i] = posX[i],     YP1[i] = posY[i],      	ZP1[i] = posZ[i];
					XP2[i] = posX[i],     YP2[i] = posY[i],      	ZP2[i] = posZ[i];
					XP3[i] = 0,						YP3[i] = 0,							ZP3[i] = 0;
					XP4[i] = 0,           YP4[i] = 0,           	ZP4[i] = 0;
					
					posX[i] = (a*XP1[i])+(b*XP2[i])+(c*XP3[i])+(d*XP4[i]);
					posY[i] = (a*YP1[i])+(b*YP2[i])+(c*YP3[i])+(d*YP4[i]);
					posZ[i] = (a*ZP1[i])+(b*ZP2[i])+(c*ZP3[i])+(d*ZP4[i]);
				}
			}
			break;
			
		case KOREKSIXY_SETA:
			for(i=0; i<6; i++)
			{
				if(i==0 || i==2 || i==4)
				{
					XP1[i] = posX[i],     YP1[i] = posY[i],      	ZP1[i] = posZ[i];
					XP2[i] = posX[i],     YP2[i] = posY[i],      	ZP2[i] = last_move_posZ;
					XP3[i] = 0,						YP3[i] = 0,							ZP3[i] = last_move_posZ;
					XP4[i] = 0,           YP4[i] = 0,           	ZP4[i] = 0;
					
					posX[i] = (a*XP1[i])+(b*XP2[i])+(c*XP3[i])+(d*XP4[i]);
					posY[i] = (a*YP1[i])+(b*YP2[i])+(c*YP3[i])+(d*YP4[i]);
					posZ[i] = (a*ZP1[i])+(b*ZP2[i])+(c*ZP3[i])+(d*ZP4[i]);
				}
			}	
			break;
			
		case KOREKSIXY_SETB:
			for(i=0; i<6; i++)
			{
				if(i==1 || i==3 || i==5)
				{
					XP1[i] = posX[i],     YP1[i] = posY[i],      	ZP1[i] = posZ[i];
					XP2[i] = posX[i],     YP2[i] = posY[i],      	ZP2[i] = last_move_posZ;
					XP3[i] = 0,						YP3[i] = 0,							ZP3[i] = last_move_posZ;
					XP4[i] = 0,           YP4[i] = 0,           	ZP4[i] = 0;
					
					posX[i] = (a*XP1[i])+(b*XP2[i])+(c*XP3[i])+(d*XP4[i]);
					posY[i] = (a*YP1[i])+(b*YP2[i])+(c*YP3[i])+(d*YP4[i]);
					posZ[i] = (a*ZP1[i])+(b*ZP2[i])+(c*ZP3[i])+(d*ZP4[i]);
				}
			}
			break;
	}
	
	poskaki_awal();
	bodykinematic();
	inverse_akhir();
	inverse_kinematic();
	servowrite();
}

/*---TRAYEKTORI GERAK HALF TRIPOD GAIT---*/
void treyektori_halftripod(int move_posX, int move_posY, int move_posZ, double t, int walkpoint)
{
	polinom(t);
	
	for(i=0; i<6; i++)
	{
		if(i<3) 
		{
			move_posY_correction[i] = move_posY + Y_corection;
			if(move_posY_correction[i] >= move_posY) move_posY_correction[i] = move_posY;
		}
		
		else 
		{
			move_posY_correction[i] = move_posY - Y_corection;
			if(move_posY_correction[i] >= move_posY) move_posY_correction[i] = move_posY;
		}
	}
	
	switch(walkpoint)
	{
		case 1:
			for(i=0; i<6; i++)
			{
				switch(i)
				{
					case 0:
						XP1[i] = 0,           		YP1[i] = 0,           									ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           									ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY_correction[i],       ZP3[i] = move_posZ;
						XP4[i] = move_posX,       YP4[i] = move_posY_correction[i],       ZP4[i] = 0;
						break;
					
					case 1:
						XP1[i] = 0,           		YP1[i] = 0,           									ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           									ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY_correction[i],      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY_correction[i],      ZP4[i] = 0;
						break;
					
					case 2:
						XP1[i] = 0,           		YP1[i] = 0,           									ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           									ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY_correction[i],       ZP3[i] = move_posZ;
						XP4[i] = move_posX,       YP4[i] = move_posY_correction[i],       ZP4[i] = 0;
						break;
					
					case 3:
						XP1[i] = 0,           		YP1[i] = 0,           									ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           									ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY_correction[i],      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY_correction[i],      ZP4[i] = 0;
						break;
					
					case 4:
						XP1[i] = 0,           		YP1[i] = 0,           									ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           									ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY_correction[i],       ZP3[i] = move_posZ;
						XP4[i] = move_posX,       YP4[i] = move_posY_correction[i],       ZP4[i] = 0;
						break;
					
					case 5:
						XP1[i] = 0,           		YP1[i] = 0,           									ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           									ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY_correction[i],      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY_correction[i],      ZP4[i] = 0;
						break;
				}
			}
			break;
		
		case 2:
			for(i=0; i<6; i++)
			{
				switch(i)
				{
					case 0:
						XP1[i] = move_posX,   		YP1[i] = move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY_correction[i],   		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY_correction[i],      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY_correction[i],      ZP4[i] = 0;
						break;
					
					case 1:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY_correction[i],   		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY_correction[i],      	ZP3[i] = move_posZ;
						XP4[i] = move_posX,      	YP4[i] = move_posY_correction[i],      	ZP4[i] = 0;
						break;
					
					case 2:
						XP1[i] = move_posX,   		YP1[i] = move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY_correction[i],   		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY_correction[i],      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY_correction[i],      ZP4[i] = 0;
						break;
					
					case 3:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY_correction[i],   		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY_correction[i],      	ZP3[i] = move_posZ;
						XP4[i] = move_posX,      	YP4[i] = move_posY_correction[i],      	ZP4[i] = 0;
						break;
					
					case 4:
						XP1[i] = move_posX,   		YP1[i] = move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY_correction[i],   		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY_correction[i],      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY_correction[i],      ZP4[i] = 0;
						break;
					
					case 5:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY_correction[i],   		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY_correction[i],      	ZP3[i] = move_posZ;
						XP4[i] = move_posX,      	YP4[i] = move_posY_correction[i],      	ZP4[i] = 0;
						break;
				}
			}
			break;
		
		case 3:
			for(i=0; i<6; i++)
			{
				switch(i)
				{
					case 0:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY_correction[i],   		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY_correction[i],      	ZP3[i] = move_posZ;
						XP4[i] = move_posX,      	YP4[i] = move_posY_correction[i],      	ZP4[i] = 0;
						break;
					
					case 1:
						XP1[i] = move_posX,   		YP1[i] = move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY_correction[i],   		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY_correction[i],      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY_correction[i],      ZP4[i] = 0;
						break;
					
					case 2:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY_correction[i],   		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY_correction[i],      	ZP3[i] = move_posZ;
						XP4[i] = move_posX,      	YP4[i] = move_posY_correction[i],      	ZP4[i] = 0;
						break;
					
					case 3:
						XP1[i] = move_posX,   		YP1[i] = move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY_correction[i],   		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY_correction[i],      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY_correction[i],      ZP4[i] = 0;
						break;
					
					case 4:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY_correction[i],   		ZP2[i] = move_posZ;
						XP3[i] = move_posX,       YP3[i] = move_posY_correction[i],      	ZP3[i] = move_posZ;
						XP4[i] = move_posX,      	YP4[i] = move_posY_correction[i],      	ZP4[i] = 0;
						break;
					
					case 5:
						XP1[i] = move_posX,   		YP1[i] = move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY_correction[i],   		ZP2[i] = 0;
						XP3[i] = -move_posX,      YP3[i] = -move_posY_correction[i],      ZP3[i] = 0;
						XP4[i] = -move_posX,      YP4[i] = -move_posY_correction[i],      ZP4[i] = 0;
						break;
				}
			}
			break;
		
		case 4:
			for(i=0; i<6; i++)
			{
				switch(i)
				{
					case 0:
						XP1[i] = move_posX,   		YP1[i] = move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY_correction[i],   		ZP2[i] = 0;
						XP3[i] = 0,       				YP3[i] = 0,       			  							ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,				        							ZP4[i] = 0;
						break;
					
					case 1:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY_correction[i],   		ZP2[i] = move_posZ;
						XP3[i] = 0,       				YP3[i] = 0,       			  							ZP3[i] = move_posZ;
						XP4[i] = 0,       				YP4[i] = 0,				        							ZP4[i] = 0;
						break;
					
					case 2:
						XP1[i] = move_posX,   		YP1[i] = move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY_correction[i],   		ZP2[i] = 0;
						XP3[i] = 0,       				YP3[i] = 0,       			  							ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,				        							ZP4[i] = 0;
						break;
					
					case 3:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY_correction[i],   		ZP2[i] = move_posZ;
						XP3[i] = 0,       				YP3[i] = 0,       			  							ZP3[i] = move_posZ;
						XP4[i] = 0,       				YP4[i] = 0,				        							ZP4[i] = 0;
						break;
					
					case 4:
						XP1[i] = move_posX,   		YP1[i] = move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = move_posX,   		YP2[i] = move_posY_correction[i],   		ZP2[i] = 0;
						XP3[i] = 0,       				YP3[i] = 0,       			  							ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,				        							ZP4[i] = 0;
						break;
					
					case 5:
						XP1[i] = -move_posX,   		YP1[i] = -move_posY_correction[i],   		ZP1[i] = 0;
						XP2[i] = -move_posX,   		YP2[i] = -move_posY_correction[i],   		ZP2[i] = move_posZ;
						XP3[i] = 0,       				YP3[i] = 0,       			  							ZP3[i] = move_posZ;
						XP4[i] = 0,       				YP4[i] = 0,				        							ZP4[i] = 0;
						break;
				}
			}
			break;
	} 
	
	posX[5] = (a*XP1[5])+(b*XP2[5])+(c*XP3[5])+(d*XP4[5]),			posX[0] = (a*XP1[0])+(b*XP2[0])+(c*XP3[0])+(d*XP4[0]);
	posY[5] = (a*YP1[5])+(b*YP2[5])+(c*YP3[5])+(d*YP4[5]),			posY[0] = (a*YP1[0])+(b*YP2[0])+(c*YP3[0])+(d*YP4[0]);
	posZ[5] = (a*ZP1[5])+(b*ZP2[5])+(c*ZP3[5])+(d*ZP4[5]),			posZ[0] = (a*ZP1[0])+(b*ZP2[0])+(c*ZP3[0])+(d*ZP4[0]);
	
	posX[4] = (a*XP1[4])+(b*XP2[4])+(c*XP3[4])+(d*XP4[4]),			posX[1] = (a*XP1[1])+(b*XP2[1])+(c*XP3[1])+(d*XP4[1]);
	posY[4] = (a*YP1[4])+(b*YP2[4])+(c*YP3[4])+(d*YP4[4]),			posY[1] = (a*YP1[1])+(b*YP2[1])+(c*YP3[1])+(d*YP4[1]);
	posZ[4] = (a*ZP1[4])+(b*ZP2[4])+(c*ZP3[4])+(d*ZP4[4]),			posZ[1] = (a*ZP1[1])+(b*ZP2[1])+(c*ZP3[1])+(d*ZP4[1]);
	
	posX[3] = (a*XP1[3])+(b*XP2[3])+(c*XP3[3])+(d*XP4[3]),			posX[2] = (a*XP1[2])+(b*XP2[2])+(c*XP3[2])+(d*XP4[2]);
	posY[3] = (a*YP1[3])+(b*YP2[3])+(c*YP3[3])+(d*YP4[3]),			posY[2] = (a*YP1[2])+(b*YP2[2])+(c*YP3[2])+(d*YP4[2]);
	posZ[3] = (a*ZP1[3])+(b*ZP2[3])+(c*ZP3[3])+(d*ZP4[3]),			posZ[2] = (a*ZP1[2])+(b*ZP2[2])+(c*ZP3[2])+(d*ZP4[2]);
	
	poskaki_awal();
	bodykinematic();
	inverse_akhir();
	inverse_kinematic();
	servowrite();
}

/*---TRAYEKTORI GERAK ROTASI---*/
void trayektori_rotasi(double angleRoll, double anglePitch, double angleYaw, double move_posZ, double t, int walkpoint, int mode)
{
	switch(mode)
	{
		/*ROTASI DINAMIS FULL*/
		case 1:
			matriks(angleRoll, anglePitch, angleYaw);
			polinom(t);
			
			switch(walkpoint)
			{
				case 1:
					for(i=0; i<6; i++)
					{
						switch(i)
						{
							case 0:
								XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
								XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = move_posZ;
								XP3[i] = move_posRX[i],       YP3[i] = move_posRY[i],       ZP3[i] = move_posZ;
								XP4[i] = move_posRX[i],       YP4[i] = move_posRY[i],       ZP4[i] = 0;
								break;
							
							case 1:
								XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
								XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = 0;
								XP3[i] = -move_posRX[i],      YP3[i] = -move_posRY[i],      ZP3[i] = 0;
								XP4[i] = -move_posRX[i],      YP4[i] = -move_posRY[i],      ZP4[i] = 0;
								break;
							
							case 2:
								XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
								XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = move_posZ;
								XP3[i] = move_posRX[i],       YP3[i] = move_posRY[i],       ZP3[i] = move_posZ;
								XP4[i] = move_posRX[i],       YP4[i] = move_posRY[i],       ZP4[i] = 0;
								break;
							
							case 3:
								XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
								XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = 0;
								XP3[i] = -move_posRX[i],      YP3[i] = -move_posRY[i],      ZP3[i] = 0;
								XP4[i] = -move_posRX[i],      YP4[i] = -move_posRY[i],      ZP4[i] = 0;
								break;
							
							case 4:
								XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
								XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = move_posZ;
								XP3[i] = move_posRX[i],       YP3[i] = move_posRY[i],       ZP3[i] = move_posZ;
								XP4[i] = move_posRX[i],       YP4[i] = move_posRY[i],       ZP4[i] = 0;
								break;
							
							case 5:
								XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
								XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = 0;
								XP3[i] = -move_posRX[i],      YP3[i] = -move_posRY[i],      ZP3[i] = 0;
								XP4[i] = -move_posRX[i],      YP4[i] = -move_posRY[i],      ZP4[i] = 0;
								break;
						}
					}
					break;
				
				case 2:
					for(i=0; i<6; i++)
					{
						switch(i)
						{
							case 0:
								XP1[i] = move_posRX[i],       YP1[i] = move_posRY[i],       ZP1[i] = 0;
								XP2[i] = move_posRX[i],       YP2[i] = move_posRY[i],		  	ZP2[i] = 0;
								XP3[i] = -move_posRX[i],			YP3[i] = -move_posRY[i],			ZP3[i] = 0;
								XP4[i] = -move_posRX[i],			YP4[i] = -move_posRY[i],			ZP4[i] = 0;
								break;
							
							case 1:
								XP1[i] = -move_posRX[i],      YP1[i] = -move_posRY[i],    	ZP1[i] = 0;
								XP2[i] = -move_posRX[i],      YP2[i] = -move_posRY[i],			ZP2[i] = move_posZ;
								XP3[i] = move_posRX[i],				YP3[i] = move_posRY[i],      	ZP3[i] = move_posZ;
								XP4[i] = move_posRX[i],				YP4[i] = move_posRY[i],       ZP4[i] = 0;
								break;
							
							case 2:
								XP1[i] = move_posRX[i],       YP1[i] = move_posRY[i],       ZP1[i] = 0;
								XP2[i] = move_posRX[i],       YP2[i] = move_posRY[i],		  	ZP2[i] = 0;
								XP3[i] = -move_posRX[i],			YP3[i] = -move_posRY[i],			ZP3[i] = 0;
								XP4[i] = -move_posRX[i],			YP4[i] = -move_posRY[i],			ZP4[i] = 0;
								break;
							
							case 3:
								XP1[i] = -move_posRX[i],      YP1[i] = -move_posRY[i],    	ZP1[i] = 0;
								XP2[i] = -move_posRX[i],      YP2[i] = -move_posRY[i],			ZP2[i] = move_posZ;
								XP3[i] = move_posRX[i],				YP3[i] = move_posRY[i],      	ZP3[i] = move_posZ;
								XP4[i] = move_posRX[i],				YP4[i] = move_posRY[i],       ZP4[i] = 0;
								break;
							
							case 4:
								XP1[i] = move_posRX[i],       YP1[i] = move_posRY[i],       ZP1[i] = 0;
								XP2[i] = move_posRX[i],       YP2[i] = move_posRY[i],		  	ZP2[i] = 0;
								XP3[i] = -move_posRX[i],			YP3[i] = -move_posRY[i],			ZP3[i] = 0;
								XP4[i] = -move_posRX[i],			YP4[i] = -move_posRY[i],			ZP4[i] = 0;
								break;
							
							case 5:
								XP1[i] = -move_posRX[i],      YP1[i] = -move_posRY[i],    	ZP1[i] = 0;
								XP2[i] = -move_posRX[i],      YP2[i] = -move_posRY[i],			ZP2[i] = move_posZ;
								XP3[i] = move_posRX[i],				YP3[i] = move_posRY[i],      	ZP3[i] = move_posZ;
								XP4[i] = move_posRX[i],				YP4[i] = move_posRY[i],       ZP4[i] = 0;
								break;
						}
					}
					break;
					
				case 3:
					for(i=0; i<6; i++)
					{
						switch(i)
						{
							case 0:
								XP1[i] = -move_posRX[i],       YP1[i] = -move_posRY[i],     ZP1[i] = 0;
								XP2[i] = -move_posRX[i],       YP2[i] = -move_posRY[i],		 	ZP2[i] = move_posZ;
								XP3[i] = move_posRX[i],				 YP3[i] = move_posRY[i],			ZP3[i] = move_posZ;
								XP4[i] = move_posRX[i],				 YP4[i] = move_posRY[i],			ZP4[i] = 0;
								break;
							
							case 1:
								XP1[i] = move_posRX[i],      	 YP1[i] = move_posRY[i],    	ZP1[i] = 0;
								XP2[i] = move_posRX[i],      	 YP2[i] = move_posRY[i],			ZP2[i] = 0;
								XP3[i] = -move_posRX[i],			 YP3[i] = -move_posRY[i],     ZP3[i] = 0;
								XP4[i] = -move_posRX[i],			 YP4[i] = -move_posRY[i],     ZP4[i] = 0;
								break;
							
							case 2:
								XP1[i] = -move_posRX[i],       YP1[i] = -move_posRY[i],     ZP1[i] = 0;
								XP2[i] = -move_posRX[i],       YP2[i] = -move_posRY[i],		 	ZP2[i] = move_posZ;
								XP3[i] = move_posRX[i],				 YP3[i] = move_posRY[i],			ZP3[i] = move_posZ;
								XP4[i] = move_posRX[i],				 YP4[i] = move_posRY[i],			ZP4[i] = 0;
								break;
							
							case 3:
								XP1[i] = move_posRX[i],      	 YP1[i] = move_posRY[i],    	ZP1[i] = 0;
								XP2[i] = move_posRX[i],      	 YP2[i] = move_posRY[i],			ZP2[i] = 0;
								XP3[i] = -move_posRX[i],			 YP3[i] = -move_posRY[i],     ZP3[i] = 0;
								XP4[i] = -move_posRX[i],			 YP4[i] = -move_posRY[i],     ZP4[i] = 0;
								break;
							
							case 4:
								XP1[i] = -move_posRX[i],       YP1[i] = -move_posRY[i],     ZP1[i] = 0;
								XP2[i] = -move_posRX[i],       YP2[i] = -move_posRY[i],		 	ZP2[i] = move_posZ;
								XP3[i] = move_posRX[i],				 YP3[i] = move_posRY[i],			ZP3[i] = move_posZ;
								XP4[i] = move_posRX[i],				 YP4[i] = move_posRY[i],			ZP4[i] = 0;
								break;
							
							case 5:
								XP1[i] = move_posRX[i],      	 YP1[i] = move_posRY[i],    	ZP1[i] = 0;
								XP2[i] = move_posRX[i],      	 YP2[i] = move_posRY[i],			ZP2[i] = 0;
								XP3[i] = -move_posRX[i],			 YP3[i] = -move_posRY[i],     ZP3[i] = 0;
								XP4[i] = -move_posRX[i],			 YP4[i] = -move_posRY[i],     ZP4[i] = 0;
								break;
						}
					}
					break;
					
				case 4:
					for(i=0; i<6; i++)
					{
						switch(i)
						{
							case 0:
								XP1[i] = move_posRX[i],       YP1[i] = move_posRY[i],       ZP1[i] = 0;
								XP2[i] = move_posRX[i],       YP2[i] = move_posRY[i],       ZP2[i] = 0;
								XP3[i] = 0,    								YP3[i] = 0,       						ZP3[i] = 0;
								XP4[i] = 0,     							YP4[i] = 0,       						ZP4[i] = 0;
								break;
							
							case 1:
								XP1[i] = -move_posRX[i],      YP1[i] = -move_posRY[i],      ZP1[i] = 0;
								XP2[i] = -move_posRX[i],      YP2[i] = -move_posRY[i],      ZP2[i] = move_posZ;
								XP3[i] = 0,    								YP3[i] = 0,       						ZP3[i] = move_posZ;
								XP4[i] = 0,     							YP4[i] = 0,       						ZP4[i] = 0;
								break;
							
							case 2:
								XP1[i] = move_posRX[i],       YP1[i] = move_posRY[i],       ZP1[i] = 0;
								XP2[i] = move_posRX[i],       YP2[i] = move_posRY[i],       ZP2[i] = 0;
								XP3[i] = 0,    								YP3[i] = 0,       						ZP3[i] = 0;
								XP4[i] = 0,     							YP4[i] = 0,       						ZP4[i] = 0;
								break;
							
							case 3:
								XP1[i] = -move_posRX[i],      YP1[i] = -move_posRY[i],      ZP1[i] = 0;
								XP2[i] = -move_posRX[i],      YP2[i] = -move_posRY[i],      ZP2[i] = move_posZ;
								XP3[i] = 0,    								YP3[i] = 0,       						ZP3[i] = move_posZ;
								XP4[i] = 0,     							YP4[i] = 0,       						ZP4[i] = 0;
								break;
							
							case 4:
								XP1[i] = move_posRX[i],       YP1[i] = move_posRY[i],       ZP1[i] = 0;
								XP2[i] = move_posRX[i],       YP2[i] = move_posRY[i],       ZP2[i] = 0;
								XP3[i] = 0,    								YP3[i] = 0,       						ZP3[i] = 0;
								XP4[i] = 0,     							YP4[i] = 0,       						ZP4[i] = 0;
								break;
							
							case 5:
								XP1[i] = -move_posRX[i],      YP1[i] = -move_posRY[i],      ZP1[i] = 0;
								XP2[i] = -move_posRX[i],      YP2[i] = -move_posRY[i],      ZP2[i] = move_posZ;
								XP3[i] = 0,    								YP3[i] = 0,       						ZP3[i] = move_posZ;
								XP4[i] = 0,     							YP4[i] = 0,       						ZP4[i] = 0;
								break;
						}
					}
					break;
			}
			
			posX[5] = (a*XP1[5])+(b*XP2[5])+(c*XP3[5])+(d*XP4[5]),			posX[0] = (a*XP1[0])+(b*XP2[0])+(c*XP3[0])+(d*XP4[0]);
			posY[5] = (a*YP1[5])+(b*YP2[5])+(c*YP3[5])+(d*YP4[5]),			posY[0] = (a*YP1[0])+(b*YP2[0])+(c*YP3[0])+(d*YP4[0]);
			posZ[5] = (a*ZP1[5])+(b*ZP2[5])+(c*ZP3[5])+(d*ZP4[5]),			posZ[0] = (a*ZP1[0])+(b*ZP2[0])+(c*ZP3[0])+(d*ZP4[0]);
			
			posX[4] = (a*XP1[4])+(b*XP2[4])+(c*XP3[4])+(d*XP4[4]),			posX[1] = (a*XP1[1])+(b*XP2[1])+(c*XP3[1])+(d*XP4[1]);
			posY[4] = (a*YP1[4])+(b*YP2[4])+(c*YP3[4])+(d*YP4[4]),			posY[1] = (a*YP1[1])+(b*YP2[1])+(c*YP3[1])+(d*YP4[1]);
			posZ[4] = (a*ZP1[4])+(b*ZP2[4])+(c*ZP3[4])+(d*ZP4[4]),			posZ[1] = (a*ZP1[1])+(b*ZP2[1])+(c*ZP3[1])+(d*ZP4[1]);
			
			posX[3] = (a*XP1[3])+(b*XP2[3])+(c*XP3[3])+(d*XP4[3]),			posX[2] = (a*XP1[2])+(b*XP2[2])+(c*XP3[2])+(d*XP4[2]);
			posY[3] = (a*YP1[3])+(b*YP2[3])+(c*YP3[3])+(d*YP4[3]),			posY[2] = (a*YP1[2])+(b*YP2[2])+(c*YP3[2])+(d*YP4[2]);
			posZ[3] = (a*ZP1[3])+(b*ZP2[3])+(c*ZP3[3])+(d*ZP4[3]),			posZ[2] = (a*ZP1[2])+(b*ZP2[2])+(c*ZP3[2])+(d*ZP4[2]);
			
			bodykinematic();
			inverse_akhir();
			inverse_kinematic();
			servowrite();
			break;
		
		/*ROTASI DINAMIS HALF*/
		case 2:		
			matriks(angleRoll, anglePitch, angleYaw);
			polinom(t);
			
			switch(walkpoint)
			{
				case 1:
					for(i=0; i<6; i++)
					{
						switch(i)
						{
							case 0:
								XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
								XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = move_posZ;
								XP3[i] = move_posRX[i],       YP3[i] = move_posRY[i],       ZP3[i] = move_posZ;
								XP4[i] = move_posRX[i],       YP4[i] = move_posRY[i],       ZP4[i] = 0;
								break;
							
							case 1:
								
							
								XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
								XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = 0;
								XP3[i] = -move_posRX[i],      YP3[i] = -move_posRY[i],      ZP3[i] = 0;
								XP4[i] = -move_posRX[i],      YP4[i] = -move_posRY[i],      ZP4[i] = 0;
								break;
							
							case 2:
								XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
								XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = move_posZ;
								XP3[i] = move_posRX[i],       YP3[i] = move_posRY[i],       ZP3[i] = move_posZ;
								XP4[i] = move_posRX[i],       YP4[i] = move_posRY[i],       ZP4[i] = 0;
								break;
							
							case 3:
								XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
								XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = 0;
								XP3[i] = -move_posRX[i],      YP3[i] = -move_posRY[i],      ZP3[i] = 0;
								XP4[i] = -move_posRX[i],      YP4[i] = -move_posRY[i],      ZP4[i] = 0;
								break;
							
							case 4:
								XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
								XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = move_posZ;
								XP3[i] = move_posRX[i],       YP3[i] = move_posRY[i],       ZP3[i] = move_posZ;
								XP4[i] = move_posRX[i],       YP4[i] = move_posRY[i],       ZP4[i] = 0;
								break;
							
							case 5:
								XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
								XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = 0;
								XP3[i] = -move_posRX[i],      YP3[i] = -move_posRY[i],      ZP3[i] = 0;
								XP4[i] = -move_posRX[i],      YP4[i] = -move_posRY[i],      ZP4[i] = 0;
								break;
						}
					}
					break;
					
				case 2:
					for(i=0; i<6; i++)
					{
						switch(i)
						{
							case 0:
								XP1[i] = move_posRX[i],       YP1[i] = move_posRY[i],       ZP1[i] = 0;
								XP2[i] = move_posRX[i],       YP2[i] = move_posRY[i],       ZP2[i] = 0;
								XP3[i] = 0,    								YP3[i] = 0,       						ZP3[i] = 0;
								XP4[i] = 0,     							YP4[i] = 0,       						ZP4[i] = 0;
								break;
							
							case 1:
								XP1[i] = -move_posRX[i],      YP1[i] = -move_posRY[i],      ZP1[i] = 0;
								XP2[i] = -move_posRX[i],      YP2[i] = -move_posRY[i],      ZP2[i] = move_posZ;
								XP3[i] = 0,    								YP3[i] = 0,       						ZP3[i] = move_posZ;
								XP4[i] = 0,     							YP4[i] = 0,       						ZP4[i] = 0;
								break;
							
							case 2:
								XP1[i] = move_posRX[i],       YP1[i] = move_posRY[i],       ZP1[i] = 0;
								XP2[i] = move_posRX[i],       YP2[i] = move_posRY[i],       ZP2[i] = 0;
								XP3[i] = 0,    								YP3[i] = 0,       						ZP3[i] = 0;
								XP4[i] = 0,     							YP4[i] = 0,       						ZP4[i] = 0;
								break;
							
							case 3:
								XP1[i] = -move_posRX[i],      YP1[i] = -move_posRY[i],      ZP1[i] = 0;
								XP2[i] = -move_posRX[i],      YP2[i] = -move_posRY[i],      ZP2[i] = move_posZ;
								XP3[i] = 0,    								YP3[i] = 0,       						ZP3[i] = move_posZ;
								XP4[i] = 0,     							YP4[i] = 0,       						ZP4[i] = 0;
								break;
							
							case 4:
								XP1[i] = move_posRX[i],       YP1[i] = move_posRY[i],       ZP1[i] = 0;
								XP2[i] = move_posRX[i],       YP2[i] = move_posRY[i],       ZP2[i] = 0;
								XP3[i] = 0,    								YP3[i] = 0,       						ZP3[i] = 0;
								XP4[i] = 0,     							YP4[i] = 0,       						ZP4[i] = 0;
								break;
							
							case 5:
								XP1[i] = -move_posRX[i],      YP1[i] = -move_posRY[i],      ZP1[i] = 0;
								XP2[i] = -move_posRX[i],      YP2[i] = -move_posRY[i],      ZP2[i] = move_posZ;
								XP3[i] = 0,    								YP3[i] = 0,       						ZP3[i] = move_posZ;
								XP4[i] = 0,     							YP4[i] = 0,       						ZP4[i] = 0;
								break;
						}
					}
					break;
			}
			
			posX[5] = (a*XP1[5])+(b*XP2[5])+(c*XP3[5])+(d*XP4[5]),			posX[0] = (a*XP1[0])+(b*XP2[0])+(c*XP3[0])+(d*XP4[0]);
			posY[5] = (a*YP1[5])+(b*YP2[5])+(c*YP3[5])+(d*YP4[5]),			posY[0] = (a*YP1[0])+(b*YP2[0])+(c*YP3[0])+(d*YP4[0]);
			posZ[5] = (a*ZP1[5])+(b*ZP2[5])+(c*ZP3[5])+(d*ZP4[5]),			posZ[0] = (a*ZP1[0])+(b*ZP2[0])+(c*ZP3[0])+(d*ZP4[0]);
			
			posX[4] = (a*XP1[4])+(b*XP2[4])+(c*XP3[4])+(d*XP4[4]),			posX[1] = (a*XP1[1])+(b*XP2[1])+(c*XP3[1])+(d*XP4[1]);
			posY[4] = (a*YP1[4])+(b*YP2[4])+(c*YP3[4])+(d*YP4[4]),			posY[1] = (a*YP1[1])+(b*YP2[1])+(c*YP3[1])+(d*YP4[1]);
			posZ[4] = (a*ZP1[4])+(b*ZP2[4])+(c*ZP3[4])+(d*ZP4[4]),			posZ[1] = (a*ZP1[1])+(b*ZP2[1])+(c*ZP3[1])+(d*ZP4[1]);
			
			posX[3] = (a*XP1[3])+(b*XP2[3])+(c*XP3[3])+(d*XP4[3]),			posX[2] = (a*XP1[2])+(b*XP2[2])+(c*XP3[2])+(d*XP4[2]);
			posY[3] = (a*YP1[3])+(b*YP2[3])+(c*YP3[3])+(d*YP4[3]),			posY[2] = (a*YP1[2])+(b*YP2[2])+(c*YP3[2])+(d*YP4[2]);
			posZ[3] = (a*ZP1[3])+(b*ZP2[3])+(c*ZP3[3])+(d*ZP4[3]),			posZ[2] = (a*ZP1[2])+(b*ZP2[2])+(c*ZP3[2])+(d*ZP4[2]);
			
			bodykinematic();
			inverse_akhir();
			inverse_kinematic();
			servowrite();
			break;
	}
}

/*---TRAYEKTORI LEANING---*/
void trayektori_lean(int lean_valX, int lean_valY, int walkpoint, int mode, double t) 
{
	polinom(t);
	
	switch(walkpoint)
	{
		/*WALKPOINT 1*/
		case 1:
			/*LEANING DEPAN*/
			if(mode == 1)
			{
				for(i=0; i<6; i++)
				{
					if(i == 0 || i == 5)
					{
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = 0;
						XP3[i] = lean_valX,       YP3[i] = lean_valY,      	ZP3[i] = lean_valZ[i];
						XP4[i] = lean_valX,       YP4[i] = lean_valY,       ZP4[i] = lean_valZ[i];
					}
					
					if(i == 1 || i == 4)
					{
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = 0;
						XP3[i] = lean_valX,       YP3[i] = lean_valY,      	ZP3[i] = lean_valZ[i]*2/3;
						XP4[i] = lean_valX,       YP4[i] = lean_valY,       ZP4[i] = lean_valZ[i]*2/3;
					}
					
					if(i == 2 || i == 3)
					{
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = 0;
						XP3[i] = lean_valX,       YP3[i] = lean_valY,      	ZP3[i] = 0;
						XP4[i] = lean_valX,       YP4[i] = lean_valY,       ZP4[i] = 0;
					}
				}
			}
			
			/*LEANING BELAKANG*/
			else if(mode == 2)
			{
				for(i=0; i<6; i++)
				{
					if(i == 0 || i == 5)
					{
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = 0;
						XP3[i] = lean_valX,       YP3[i] = lean_valY,      	ZP3[i] = 0;
						XP4[i] = lean_valX,       YP4[i] = lean_valY,       ZP4[i] = 0;
					}
					
					if(i == 1 || i == 4)
					{
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = 0;
						XP3[i] = lean_valX,       YP3[i] = lean_valY,      	ZP3[i] = lean_valZ[i]*2/3;
						XP4[i] = lean_valX,       YP4[i] = lean_valY,       ZP4[i] = lean_valZ[i]*2/3;
					}
					
					if(i == 2 || i == 3)
					{
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = 0;
						XP3[i] = lean_valX,       YP3[i] = lean_valY,      	ZP3[i] = lean_valZ[i];
						XP4[i] = lean_valX,       YP4[i] = lean_valY,       ZP4[i] = lean_valZ[i];
					}
				}
			}
			
			/*LEANING KANAN*/
			else if(mode == 3)
			{
				for(i=0; i<6; i++)
				{
					if(i<3)
					{
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = 0;
						XP3[i] = lean_valX,       YP3[i] = lean_valY,      	ZP3[i] = lean_valZ[i];
						XP4[i] = lean_valX,       YP4[i] = lean_valY,       ZP4[i] = lean_valZ[i];
					}
					
					else
					{
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = 0;
						XP3[i] = lean_valX,       YP3[i] = lean_valY,      	ZP3[i] = -lean_valZ[i];
						XP4[i] = lean_valX,       YP4[i] = lean_valY,       ZP4[i] = -lean_valZ[i];
					}
				}
			}
			
			/*LEANING KIRI*/
			else if(mode == 4)
			{
				for(i=0; i<6; i++)
				{
					if(i<3)
					{
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = 0;
						XP3[i] = lean_valX,       YP3[i] = lean_valY,      	ZP3[i] = -lean_valZ[i];
						XP4[i] = lean_valX,       YP4[i] = lean_valY,       ZP4[i] = -lean_valZ[i];
					}
					
					else
					{
						XP1[i] = 0,           		YP1[i] = 0,           		ZP1[i] = 0;
						XP2[i] = 0,           		YP2[i] = 0,           		ZP2[i] = 0;
						XP3[i] = lean_valX,       YP3[i] = lean_valY,      	ZP3[i] = lean_valZ[i];
						XP4[i] = lean_valX,       YP4[i] = lean_valY,       ZP4[i] = lean_valZ[i];
					}
				}
			}
			break;
		
		/*WALKPOINT 2*/
		case 2:
			/*LEANING DEPAN*/
			if(mode == 1)
			{
				for(i=0; i<6; i++)
				{
					if(i == 0 || i == 5)
					{
						XP1[i] = lean_valX,   		YP1[i] = lean_valY,       ZP1[i] = lean_valZ[i];
						XP2[i] = lean_valX,       YP2[i] = lean_valY,       ZP2[i] = lean_valZ[i];
						XP3[i] = 0,       				YP3[i] = 0,      					ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,       				ZP4[i] = 0;
					}
					
					if(i == 1 || i == 4)
					{
						XP1[i] = lean_valX,   		YP1[i] = lean_valY,       ZP1[i] = lean_valZ[i]*2/3;
						XP2[i] = lean_valX,       YP2[i] = lean_valY,       ZP2[i] = lean_valZ[i]*2/3;
						XP3[i] = 0,       				YP3[i] = 0,      					ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,       				ZP4[i] = 0;
					}
					
					if(i == 2 || i == 3)
					{
						XP1[i] = lean_valX,   		YP1[i] = lean_valY,       ZP1[i] = 0;
						XP2[i] = lean_valX,       YP2[i] = lean_valY,       ZP2[i] = 0;
						XP3[i] = 0,       				YP3[i] = 0,      					ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,       				ZP4[i] = 0;
					}
				}
			}
			
			else if(mode == 2)
			{
				for(i=0; i<6; i++)
				{
					if(i == 0 || i == 5)
					{
						XP1[i] = lean_valX,   		YP1[i] = lean_valY,       ZP1[i] = 0;
						XP2[i] = lean_valX,       YP2[i] = lean_valY,       ZP2[i] = 0;
						XP3[i] = 0,       				YP3[i] = 0,      					ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,       				ZP4[i] = 0;
					}
					
					if(i == 1 || i == 4)
					{
						XP1[i] = lean_valX,   		YP1[i] = lean_valY,       ZP1[i] = lean_valZ[i]*2/3;
						XP2[i] = lean_valX,       YP2[i] = lean_valY,       ZP2[i] = lean_valZ[i]*2/3;
						XP3[i] = 0,       				YP3[i] = 0,      					ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,       				ZP4[i] = 0;
					}
					
					if(i == 2 || i == 3)
					{
						XP1[i] = lean_valX,   		YP1[i] = lean_valY,       ZP1[i] = lean_valZ[i];
						XP2[i] = lean_valX,       YP2[i] = lean_valY,       ZP2[i] = lean_valZ[i];
						XP3[i] = 0,       				YP3[i] = 0,      					ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,       				ZP4[i] = 0;
					}
				}
			}
			
			/*LEANING KANAN*/
			else if(mode == 3)
			{
				for(i=0; i<6; i++)
				{
					if(i<3)
					{
						XP1[i] = lean_valX,   		YP1[i] = lean_valY,       ZP1[i] = lean_valZ[i];
						XP2[i] = lean_valX,       YP2[i] = lean_valY,       ZP2[i] = lean_valZ[i];
						XP3[i] = 0,       				YP3[i] = 0,      					ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,       				ZP4[i] = 0;
					}
					
					else
					{
						XP1[i] = lean_valX,   		YP1[i] = lean_valY,       ZP1[i] = -lean_valZ[i];
						XP2[i] = lean_valX,       YP2[i] = lean_valY,       ZP2[i] = -lean_valZ[i];
						XP3[i] = 0,       				YP3[i] = 0,      					ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,       				ZP4[i] = 0;
					}
				}
			}
			
			/*LEANING KIRI*/
			else if(mode == 4)
			{
				for(i=0; i<6; i++)
				{
					if(i<3)
					{
						XP1[i] = lean_valX,   		YP1[i] = lean_valY,       ZP1[i] = -lean_valZ[i];
						XP2[i] = lean_valX,       YP2[i] = lean_valY,       ZP2[i] = -lean_valZ[i];
						XP3[i] = 0,       				YP3[i] = 0,      					ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,       				ZP4[i] = 0;
					}
					
					else
					{
						XP1[i] = lean_valX,   		YP1[i] = lean_valY,       ZP1[i] = lean_valZ[i];
						XP2[i] = lean_valX,       YP2[i] = lean_valY,       ZP2[i] = lean_valZ[i];
						XP3[i] = 0,       				YP3[i] = 0,      					ZP3[i] = 0;
						XP4[i] = 0,       				YP4[i] = 0,       				ZP4[i] = 0;
					}
				}
			}
			break;
	}
	
	posX[5] = (a*XP1[5])+(b*XP2[5])+(c*XP3[5])+(d*XP4[5]),			posX[0] = (a*XP1[0])+(b*XP2[0])+(c*XP3[0])+(d*XP4[0]);
	posY[5] = (a*YP1[5])+(b*YP2[5])+(c*YP3[5])+(d*YP4[5]),			posY[0] = (a*YP1[0])+(b*YP2[0])+(c*YP3[0])+(d*YP4[0]);
	posZ[5] = (a*ZP1[5])+(b*ZP2[5])+(c*ZP3[5])+(d*ZP4[5]),			posZ[0] = (a*ZP1[0])+(b*ZP2[0])+(c*ZP3[0])+(d*ZP4[0]);
	
	posX[4] = (a*XP1[4])+(b*XP2[4])+(c*XP3[4])+(d*XP4[4]),			posX[1] = (a*XP1[1])+(b*XP2[1])+(c*XP3[1])+(d*XP4[1]);
	posY[4] = (a*YP1[4])+(b*YP2[4])+(c*YP3[4])+(d*YP4[4]),			posY[1] = (a*YP1[1])+(b*YP2[1])+(c*YP3[1])+(d*YP4[1]);
	posZ[4] = (a*ZP1[4])+(b*ZP2[4])+(c*ZP3[4])+(d*ZP4[4]),			posZ[1] = (a*ZP1[1])+(b*ZP2[1])+(c*ZP3[1])+(d*ZP4[1]);
	
	posX[3] = (a*XP1[3])+(b*XP2[3])+(c*XP3[3])+(d*XP4[3]),			posX[2] = (a*XP1[2])+(b*XP2[2])+(c*XP3[2])+(d*XP4[2]);
	posY[3] = (a*YP1[3])+(b*YP2[3])+(c*YP3[3])+(d*YP4[3]),			posY[2] = (a*YP1[2])+(b*YP2[2])+(c*YP3[2])+(d*YP4[2]);
	posZ[3] = (a*ZP1[3])+(b*ZP2[3])+(c*ZP3[3])+(d*ZP4[3]),			posZ[2] = (a*ZP1[2])+(b*ZP2[2])+(c*ZP3[2])+(d*ZP4[2]);
	
	bodykinematic();
	inverse_akhir();
	inverse_kinematic();
	servowrite();
}

/*---TRAYEKTORI BOOTING---*/
void trayektori_booting(double t, int walkpoint)
{
	polinom(t);
	
	switch(walkpoint)
	{
		case 1:	
			for(i=0; i<6; i++)
			{
				XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
				XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = 0;
				XP3[i] = adjust_posX[i],      YP3[i] = adjust_posY[i],      ZP3[i] = adjust_posZ[i] + updateposZ[i];
				XP4[i] = adjust_posX[i],      YP4[i] = adjust_posY[i],      ZP4[i] = adjust_posZ[i] + updateposZ[i];
			}
			break;
		
		case 2:
			for(i=0; i<6; i++)
			{
				XP1[i] = 0,           				YP1[i] = 0,           				ZP1[i] = 0;
				XP2[i] = 0,           				YP2[i] = 0,           				ZP2[i] = 0;
				XP3[i] = -adjust_posX[i],     YP3[i] = -adjust_posY[i],     ZP3[i] = -(adjust_posZ[i] + updateposZ[i]);
				XP4[i] = -adjust_posX[i],     YP4[i] = -adjust_posY[i],     ZP4[i] = -(adjust_posZ[i] + updateposZ[i]);
			}
			break;
	}
	
	posX[5] = (a*XP1[5])+(b*XP2[5])+(c*XP3[5])+(d*XP4[5]),			posX[0] = (a*XP1[0])+(b*XP2[0])+(c*XP3[0])+(d*XP4[0]);
	posY[5] = (a*YP1[5])+(b*YP2[5])+(c*YP3[5])+(d*YP4[5]),			posY[0] = (a*YP1[0])+(b*YP2[0])+(c*YP3[0])+(d*YP4[0]);
	posZ[5] = (a*ZP1[5])+(b*ZP2[5])+(c*ZP3[5])+(d*ZP4[5]),			posZ[0] = (a*ZP1[0])+(b*ZP2[0])+(c*ZP3[0])+(d*ZP4[0]);
	
	posX[4] = (a*XP1[4])+(b*XP2[4])+(c*XP3[4])+(d*XP4[4]),			posX[1] = (a*XP1[1])+(b*XP2[1])+(c*XP3[1])+(d*XP4[1]);
	posY[4] = (a*YP1[4])+(b*YP2[4])+(c*YP3[4])+(d*YP4[4]),			posY[1] = (a*YP1[1])+(b*YP2[1])+(c*YP3[1])+(d*YP4[1]);
	posZ[4] = (a*ZP1[4])+(b*ZP2[4])+(c*ZP3[4])+(d*ZP4[4]),			posZ[1] = (a*ZP1[1])+(b*ZP2[1])+(c*ZP3[1])+(d*ZP4[1]);
	
	posX[3] = (a*XP1[3])+(b*XP2[3])+(c*XP3[3])+(d*XP4[3]),			posX[2] = (a*XP1[2])+(b*XP2[2])+(c*XP3[2])+(d*XP4[2]);
	posY[3] = (a*YP1[3])+(b*YP2[3])+(c*YP3[3])+(d*YP4[3]),			posY[2] = (a*YP1[2])+(b*YP2[2])+(c*YP3[2])+(d*YP4[2]);
	posZ[3] = (a*ZP1[3])+(b*ZP2[3])+(c*ZP3[3])+(d*ZP4[3]),			posZ[2] = (a*ZP1[2])+(b*ZP2[2])+(c*ZP3[2])+(d*ZP4[2]);
	
	bodykinematic();
	inverse_akhir();
	inverse_kinematic();
	servowrite();
}

/*---GERAK TRANSLASI ROBOT---*/
void jalan(int move_posX, int move_posY, int move_posZ, double set_speed, int mode)
{
	steady = false;
	tick_now = HAL_GetTick();
	
	iterasi 					= 0.5/20;
	delay_time 				= set_speed;
	
	switch(mode)
	{
		case 1:
			last_move_posX = move_posX, last_move_posY = move_posY, last_move_posZ = move_posZ;
			gerak_tripod 	= true;
			gerak_halftripod = false;
		
			if(langkah == 0 && stop_state == false)
			{		
				if(tick_now - prev_tick > delay_time && t<=1){
					trayektori_tripod(move_posX, move_posY, move_posZ, t, 1);
					
					homing_posX[5] = posX[5],		homing_posX[0] = posX[0];
					homing_posY[5] = posY[5],		homing_posY[0] = posY[0];
					homing_posZ[5] = posZ[5],		homing_posZ[0] = posZ[0];
					
					homing_posX[4] = posX[4],		homing_posX[1] = posX[1];
					homing_posY[4] = posY[4],		homing_posY[1] = posY[1];
					homing_posZ[4] = posZ[4],		homing_posZ[1] = posZ[1];
					
					homing_posX[3] = posX[3],		homing_posX[2] = posX[2];
					homing_posY[3] = posY[3],		homing_posY[2] = posY[2];
					homing_posZ[3] = posZ[3],		homing_posZ[2] = posZ[2];
					
					t+=iterasi;
					prev_tick = tick_now;
				}
				
				if(t>=1 && walkA == false && walkB == false){
					t=0;
					langkah += 1;
					walkA = true, walkB = false;
				}
			}
			
			else if(langkah > 0 && stop_state == false){
				if(tick_now - prev_tick > delay_time && t<=1 && walkA == true){
					trayektori_tripod(move_posX, move_posY, move_posZ, t, 2);
					
					homing_posX[5] = posX[5],		homing_posX[0] = posX[0];
					homing_posY[5] = posY[5],		homing_posY[0] = posY[0];
					homing_posZ[5] = posZ[5],		homing_posZ[0] = posZ[0];
					
					homing_posX[4] = posX[4],		homing_posX[1] = posX[1];
					homing_posY[4] = posY[4],		homing_posY[1] = posY[1];
					homing_posZ[4] = posZ[4],		homing_posZ[1] = posZ[1];
					
					homing_posX[3] = posX[3],		homing_posX[2] = posX[2];
					homing_posY[3] = posY[3],		homing_posY[2] = posY[2];
					homing_posZ[3] = posZ[3],		homing_posZ[2] = posZ[2];
					
					t+=iterasi;
					prev_tick = tick_now;
				}
				
				else if(t>=1 && walkA == true){
					t = 0;
					langkah += 1;
					walkA = false, walkB = true;
				}
				
				if(tick_now - prev_tick > delay_time && t<=1 && walkB == true){
					trayektori_tripod(move_posX, move_posY, move_posZ, t, 3);
					
					homing_posX[5] = posX[5],		homing_posX[0] = posX[0];
					homing_posY[5] = posY[5],		homing_posY[0] = posY[0];
					homing_posZ[5] = posZ[5],		homing_posZ[0] = posZ[0];
					
					homing_posX[4] = posX[4],		homing_posX[1] = posX[1];
					homing_posY[4] = posY[4],		homing_posY[1] = posY[1];
					homing_posZ[4] = posZ[4],		homing_posZ[1] = posZ[1];
					
					homing_posX[3] = posX[3],		homing_posX[2] = posX[2];
					homing_posY[3] = posY[3],		homing_posY[2] = posY[2];
					homing_posZ[3] = posZ[3],		homing_posZ[2] = posZ[2];
					
					t+=iterasi;
					prev_tick = tick_now;
				}
				
				else if(t>=1 && walkB == true){
					t = 0;
					langkah += 1;
					walkA = true, walkB = false;
				}
			}	
			break;
		
		case 2:
			last_move_posX = move_posX, last_move_posY = move_posY, last_move_posZ = move_posZ;
			gerak_halftripod 	= true;
			gerak_tripod = false;
		
			if(langkah == 0)
			{
				for(t=0.00; t<=1; t=t+iterasi)
				{
					treyektori_halftripod(move_posX, move_posY, move_posZ, t, 1);
					HAL_Delay(delay_time);
				}
				
				langkah += 1;
			}
			
			else if(langkah > 0)
			{
				for(t=0.00; t<=1; t=t+iterasi)
				{
					treyektori_halftripod(move_posX, move_posY, move_posZ, t, 2);
					HAL_Delay(delay_time);
				}
				
				for(t=0.00; t<=1; t=t+iterasi)
				{
					treyektori_halftripod(move_posX, move_posY, move_posZ, t, 3);
					HAL_Delay(delay_time);
				}
				
				langkah += 1;
			}	
			break;
	}
}

/*---GERAK ROTASI ROBOT---*/
void rotasi(double angleRoll, double anglePitch, double angleYaw, double move_posZ, int mode, int set_speed)
{
	gerak_rotasi 	= true;
	steady = false;
	
	iterasi 			= 0.5/set_speed;
	delay_time 		= set_speed;
	
	mode_rotasi = mode;
	
	switch(mode)
	{
		case 1:
			if(langkah == 0)
			{
				for(t=0.00; t<=1; t=t+iterasi)
				{
					trayektori_rotasi(angleRoll, anglePitch, angleYaw, move_posZ, t, 1, 1);
					HAL_Delay(delay_time);
				}
				
				langkah += 1;
			}
			
			else if(langkah > 0)
			{
				for(t=0.00; t<=1; t=t+iterasi)
				{
					trayektori_rotasi(angleRoll, anglePitch, angleYaw, move_posZ, t, 2, 1);
					HAL_Delay(delay_time);
				}
				
				for(t=0.00; t<=1; t=t+iterasi)
				{
					trayektori_rotasi(angleRoll, anglePitch, angleYaw, move_posZ, t, 3, 1);
					HAL_Delay(delay_time);
				}
				
				langkah += 1;
			}
			break;
		
		case 2:
			if(langkah == 0)
			{
				for(t=0.00; t<=1; t=t+iterasi)
				{
					trayektori_rotasi(angleRoll, anglePitch, angleYaw, move_posZ, t, 1, 2);
					HAL_Delay(delay_time);
				}
				
				langkah += 1;
			}
			
			else if(langkah > 0)
			{
				for(t=0.00; t<=1; t=t+iterasi)
				{
					trayektori_rotasi(angleRoll, anglePitch, angleYaw, move_posZ, t, 2, 2);
					HAL_Delay(delay_time);
				}
				
				for(t=0.00; t<=1; t=t+iterasi)
				{
					trayektori_rotasi(angleRoll, anglePitch, angleYaw, move_posZ, t, 1, 2);
					HAL_Delay(delay_time);
				}
				
				langkah += 1;
			}
			break;
			
		case 3:
			if(rotasi_limit != true)
			{
				for(t=0.00; t<=1; t=t+iterasi)
				{
					trayektori_rotasi(angleRoll, anglePitch, angleYaw, move_posZ, t, 1, 3);
					HAL_Delay(delay_time);
					
					if(t==1)
					{
						for(i=0; i<6; i++)
						{
							updateposX[i] = posX[i];
							updateposY[i] = posY[i];
							updateposZ[i] = posZ[i];
						}
					}
				}
				
				rotasi_limit = true;
			}
			
			else if(rotasi_limit == true)
			{
				pos_steady();
			}
			
			else if(rotasi_limit == true)
			{
				for(i=0; i<6; i++)
				{
					updateposX[i] = 0;
					updateposY[i] = 0;
					updateposZ[i] = 0;
				}
						
				for(t=0.00; t<=1; t=t+iterasi)
				{
					trayektori_rotasi(angleRoll, anglePitch, angleYaw, move_posZ, t, 2, 3);
					HAL_Delay(delay_time);
				}
			}
			break;
	}
}

/*---GERAK LEANING ROBOT---*/
void lean(int input_valXY, int input_valZ, int mode, int set_speed)
{
	iterasi 			= 0.5/set_speed;
	delay_time 		= set_speed;
	
	lean_valXY = input_valXY;
	
	mode_leaning = mode;
	
	leaning = true;
	steady = false;
	
	if(lean_limit == false)
	{
		switch(mode)
		{
			/*DEPAN*/
			case 1:
				if(input_valZ > 0 && input_valZ <= 30) 
				{
					lean_valZ[5] = input_valZ,		lean_valZ[0] = input_valZ;
					lean_valZ[4] = input_valZ,		lean_valZ[1] = input_valZ;
					lean_valZ[3] = input_valZ,		lean_valZ[2] = input_valZ;
				}
				
				else
				{
					lean_valZ[5] = 0,		lean_valZ[0] = 0;
					lean_valZ[4] = 0,		lean_valZ[1] = 0;
					lean_valZ[3] = 0,		lean_valZ[2] = 0;
				}
				
				for(t=0; t<=1; t=t+iterasi)
				{
					trayektori_lean(0, -lean_valXY, 1, mode, t);
					HAL_Delay(delay_time);
				}
				
				if(t==1)
				{
					for(i=0; i<6; i++)
					{
						updateposX[i] = posX[i];
						updateposY[i] = posY[i];
						updateposZ[i] = posZ[i];
					}
				}
				break;
			
			/*BELAKANG*/
			case 2:
				if(input_valZ > 0 && input_valZ <= 30) 
				{
					lean_valZ[5] = input_valZ,		lean_valZ[0] = input_valZ;
					lean_valZ[4] = input_valZ,		lean_valZ[1] = input_valZ;
					lean_valZ[3] = input_valZ,		lean_valZ[2] = input_valZ;
				}
				
				else
				{
					lean_valZ[5] = 0,		lean_valZ[0] = 0;
					lean_valZ[4] = 0,		lean_valZ[1] = 0;
					lean_valZ[3] = 0,		lean_valZ[2] = 0;
				}
				
				for(t=0; t<=1; t=t+iterasi)
				{
					trayektori_lean(0, lean_valXY, 1, mode, t);
					HAL_Delay(delay_time);
				}
				
				if(t==1)
				{
					for(i=0; i<6; i++)
					{
						updateposX[i] = posX[i];
						updateposY[i] = posY[i];
						updateposZ[i] = posZ[i];
					}
				}
				break;
			
			/*KANAN*/
			case 3:
				if(posZ_lean[0] == -80 && input_valZ > 0) lean_valZ[0] = 0;
				else lean_valZ[0] = input_valZ;
				if(posZ_lean[1] == -80 && input_valZ > 0) lean_valZ[1] = 0;
				else lean_valZ[1] = input_valZ;
				if(posZ_lean[2] == -80 && input_valZ > 0) lean_valZ[2] = 0;
				else lean_valZ[2] = input_valZ;
			
				lean_valZ[3] = 0;
				lean_valZ[4] = 0;
				lean_valZ[5] = 0;
				
				for(t=0; t<=1; t=t+iterasi)
				{
					trayektori_lean(-lean_valXY, 0, 1, mode, t);
					HAL_Delay(delay_time);
				}
				
				if(t==1)
				{
					for(i=0; i<6; i++)
					{
						updateposX[i] = posX[i];
						updateposY[i] = posY[i];
						updateposZ[i] = posZ[i];
					}
				}
				break;
			
			/*KIRI*/
			case 4:
				lean_valZ[0] = 0;
				lean_valZ[1] = 0;
				lean_valZ[2] = 0;
			
				if(posZ_lean[3] == -80 && input_valZ > 0) lean_valZ[3] = 0;
				else lean_valZ[3] = input_valZ;
				if(posZ_lean[4] == -80 && input_valZ > 0) lean_valZ[4] = 0;
				else lean_valZ[4] = input_valZ;
				if(posZ_lean[5] == -80 && input_valZ > 0) lean_valZ[5] = 0;
				else lean_valZ[5] = input_valZ;
			
				for(t=0; t<=1; t=t+iterasi)
				{
					trayektori_lean(lean_valXY, 0, 1, mode, t);
					HAL_Delay(delay_time);
				}
				
				if(t==1)
				{
					for(i=0; i<6; i++)
					{
						updateposX[i] = posX[i];
						updateposY[i] = posY[i];
						updateposZ[i] = posZ[i];
					}
				}
				break;
		}
		
		lean_limit = true;
	}
	
	else if(lean_limit == true && leaning == true)
	{
		titik_awal(rx_posSTDX, rx_posSTDY, rx_posSTDZ, 1);
		pos_steady();
	}
}

/*---KONTROL STOP GERAK KOREKSI---*/
void pos_correction(int set_speed)
{	
	iterasi 					= 0.5/set_speed;
	delay_time 				= set_speed;
	trajectory_delay = delay_time;
	
	if(gerak_tripod == true){
		stop_state = true;
		
		while(1){
			if(Z0[0] > adjust_posZ[0] && Z0[2] > adjust_posZ[2] && Z0[4] > adjust_posZ[4]){
				while(1){	
					for(t=0; t<=1; t+=iterasi){
						trayektori_tripod_stop(t, KOREKSIZ_SETA);
						HAL_Delay(delay_time);
					}
					
					for(t=0; t<=1; t+=iterasi){
						trayektori_tripod_stop(t, KOREKSIXY_SETB);
						HAL_Delay(delay_time);
					}
					break;
				}
				gerak_tripod = false;
				stop_state = false;
				break;
			}
			
			else if(Z0[1] > adjust_posZ[1] && Z0[3] > adjust_posZ[3] && Z0[5] > adjust_posZ[5]){
				while(1){
					for(t=0; t<=1; t+=iterasi)
					{
						trayektori_tripod_stop(t, KOREKSIZ_SETB);
						HAL_Delay(delay_time);
					}
					
					for(t=0; t<=1; t+=iterasi)
					{
						trayektori_tripod_stop(t, KOREKSIXY_SETA);
						HAL_Delay(delay_time);
					}
					break;
				}
				gerak_tripod = false;
				stop_state = false;
				break;
			}
		}
	}
	
	else pos_steady();
}

/*---KONTROL STOP GERAK---*/
void stop_gerak()
{
	steady = true;
	leaning = false;
	
	if(steady == true && gerak_rotasi == false && gerak_tripod == false && gerak_halftripod == false && lean_limit == false)
	{
		pos_steady();
	}
	
	if(gerak_tripod == true && steady == true)
	{
		for(t=0.00; t<=1; t=t+iterasi)
		{  
			trayektori_tripod(last_move_posX, last_move_posY, last_move_posZ, t, 4);
			HAL_Delay(delay_time);
		}

		gerak_tripod = false;
		
// 		stop_gerak_cor();
	}
	
	else if(gerak_halftripod == true && steady == true)
	{
		for(t=0.00; t<=1; t=t+iterasi)
		{
			treyektori_halftripod(last_move_posX, last_move_posY, last_move_posZ, t, 4);
			HAL_Delay(delay_time);
		}
		
		gerak_halftripod = false;
	}
		
	else if(gerak_rotasi == true && steady == true)
	{
		switch(mode_rotasi)
		{			
			case 1:
				for(t=0.00; t<=1; t=t+iterasi)
				{
					trayektori_rotasi(angleRoll, anglePitch, angleYaw, move_posZ, t, 4, 1);
					HAL_Delay(delay_time);
				}
				break;
				
			case 2:
				break;
		}
			
		gerak_rotasi = false;
	}
	
	else if(leaning == false && lean_limit == true && steady == true)
	{
		for(i=0; i<6; i++)
		{
			updateposX[i] = 0;
			updateposY[i] = 0;
			updateposZ[i] = 0;
		}
		
		switch(mode_leaning)
		{
			/*LEAN DEPAN*/
			case 1:
				for(t=0; t<=1; t=t+iterasi)
				{
					trayektori_lean(0, -lean_valXY, 2, mode_leaning, t);
					HAL_Delay(delay_time);
				}
				
				lean_limit = false;
				break;
			
			/*LEAN BELAKANG*/
			case 2:
				for(t=0; t<=1; t=t+iterasi)
				{
					trayektori_lean(0, lean_valXY, 2, mode_leaning, t);
					HAL_Delay(delay_time);
				}
				
				lean_limit = false;
				break;
			
			/*LEAN KANAN*/
			case 3:
				for(t=0; t<=1; t=t+iterasi)
				{
					trayektori_lean(-lean_valXY, 0, 2, mode_leaning, t);
					HAL_Delay(delay_time);
				}
				
				lean_limit = false;
				break;
			
			/*LEAN KIRI*/
			case 4:
				for(t=0; t<=1; t=t+iterasi)
				{
					trayektori_lean(lean_valXY, 0, 2, mode_leaning, t);
					HAL_Delay(delay_time);
				}
				
				lean_limit = false;
				break;
		}
	}
}

/*---BOOTING ON---*/
void booting_on(int set_speed)
{ 
//	init_posX[0] = 0, init_posY[0] = 0, init_posZ[0] = 0;
//	init_posX[1] = 0, init_posY[1] = 0, init_posZ[1] = 0;
//	init_posX[2] = 0, init_posY[2] = 0, init_posZ[2] = 0;
//	
//	init_posX[3] = 0, init_posY[3] = 0, init_posZ[3] = 0;
//	init_posX[4] = 0, init_posY[4] = 0, init_posZ[4] = 0;
//	init_posX[5] = 0, init_posY[5] = 0, init_posZ[5] = 0;
	
	iterasi 		= 0.5/set_speed;
	delay_time 	= set_speed;
	
	pos_nol();
	
	HAL_Delay(1000);
	
	for(t=0; t<=1; t=t+iterasi)
	{
		trayektori_booting(t, 1);
		HAL_Delay(delay_time);
	}
}

/*---BOOTING OFF---*/
void booting_off(int set_speed)
{ 
	iterasi 		= 0.5/set_speed;
	delay_time 	= set_speed;
	
	for(t=0; t<=1; t=t+iterasi)
	{
		trayektori_booting(t, 2);
		HAL_Delay(delay_time);
	}
}

/*---CEK KONDISI TERRAIN---*/
void cek_nanjak()
{
	MPU9250_Read_Data(&hi2c1, &MPU9250);

	dataGX = MPU9250.KalmanAngleX;
	dataGY = MPU9250.KalmanAngleY;
	dataGZ = MPU9250.Gz;
	
	if(dataGX > 20 && dataGY > 200)
	{
		savedataGx = dataGX;
		savedataGy = dataGX;
	}
}

/*---POSISI CAPIT KALIBRASI---*/
void poscapit_kal()
{
	htim4.Instance->CCR1		= HOME_COXA_CAPIT1;
	htim4.Instance->CCR2		= HOME_FEMUR_CAPIT1;
	htim4.Instance->CCR3		= TUTUP_CAPIT;
	
	htim12.Instance->CCR1		= CAPIT_BAWAH_NAIK;
	htim12.Instance->CCR2		= TUTUP_CAPIT_BAWAH;
}

/*---POSISI CAPIT HOME---*/
void poscapit_home()
{
	htim4.Instance->CCR1		= HOME_COXA_CAPIT1;
	htim4.Instance->CCR2		= HOME_FEMUR_CAPIT1;
	htim4.Instance->CCR3		= BUKA_CAPIT1;
	
	bawa_korban = false;
}

/*---POSISI CAPIT STEADY---*/
void poscapit_stdy()
{
	switch(mode_buka)
	{
		case 0:
			htim4.Instance->CCR1		= STDY_COXA_CAPIT;
			htim4.Instance->CCR2		= STDY_FEMUR_CAPIT;
			htim4.Instance->CCR3		= BUKA_CAPIT1;
			break;
		
		case 1:
			htim4.Instance->CCR1		= STDY_COXA_CAPIT;
			htim4.Instance->CCR2		= STDY_FEMUR_CAPIT;
			htim4.Instance->CCR3		= BUKA_CAPIT2;
			break;
	}
	
	bawa_korban = false;
}
  
/*---POSISI CAPIT MENYAPIT---*/
void poscapit_nyapit(int mode_nyapit)
{
	switch(mode_nyapit)
	{
		case 1:
			htim4.Instance->CCR1		= NYAPIT_COXA_CAPIT;
			htim4.Instance->CCR2		= NYAPIT_FEMUR_CAPIT;
			htim4.Instance->CCR3		= TUTUP_CAPIT;
		
			bawa_korban = true;
			break;
		
		case 2:
			htim4.Instance->CCR1		= NYAPIT_COXA_CAPIT;
			htim4.Instance->CCR2		= NYAPIT_FEMUR_CAPIT;
			htim4.Instance->CCR3		= BUKA_CAPIT1;
		
			bawa_korban = false;
			break;
	}
}

/*---POSISI CAPIT EVAKUASI---*/
void poscapit_evakuasi(int mode_evakuasi)
{
	bawa_korban = true;
	
	switch(mode_evakuasi)
	{
		case 1:
			htim4.Instance->CCR1	= HOME_COXA_CAPIT1;
			htim4.Instance->CCR2	= HOME_FEMUR_CAPIT1;
			htim4.Instance->CCR3	= TUTUP_CAPIT;
			break;
			
		case 2:
			htim4.Instance->CCR1	= HOME_COXA_CAPIT2;
			htim4.Instance->CCR2	= HOME_FEMUR_CAPIT2;
			htim4.Instance->CCR3	= TUTUP_CAPIT;
			break;
	}
}

/*KONTROL CAPIT*/
void nyapit(int mode, double set_speed)
{
	switch(mode)
	{
		case 1:
			/*STEADY -> HOME*/
			if(capit_pos == 1 && capit_home == false && capit_evakuasihalf == false)
			{
				if(mode_buka == 0) htim4.Instance->CCR3 = BUKA_CAPIT1;
				else if(mode_buka == 1) htim4.Instance->CCR3 = BUKA_CAPIT2;
				
				for(i=0; i<=deltaFEMUR_home_stdy; i+=5)
				{
					htim4.Instance->CCR2 = STDY_FEMUR_CAPIT + i;
					
					HAL_Delay(set_speed/2);
				}
				
				for(i=0; i<deltaCOXA_home_stdy; i+=10)
				{
					htim4.Instance->CCR1 = STDY_COXA_CAPIT - i;
					
					HAL_Delay(set_speed/2);
				}
				
				capit_home = true;
			}
			
			/*EVAKUASI HALF -> HOME*/
			else if(capit_pos == 0 && capit_home == false && capit_evakuasihalf == true)
			{				
				for(i=0; i<=deltaFEMUR_evakuasihalf_home; i+=5)
				{
					htim4.Instance->CCR2 = HOME_FEMUR_CAPIT2 + i;
					
					HAL_Delay(set_speed/2);
				}	
				
				for(i=0; i<=deltaCOXA_evakuasihalf_home; i+=5)
				{
					htim4.Instance->CCR1 = HOME_COXA_CAPIT2 - i;
					
					HAL_Delay(set_speed/2);
				}	
				
				capit_home = true;
			}
			
			if(capit_pos == 0)
			{
				capit_home = true;
			}
			
			if(capit_home == true)
			{
				poscapit_home();
				capit_stdy = false;
				capit_nyapit = false;
				capit_evakuasi = false;
				capit_evakuasihalf = false;
				bawa_korban = false;
			}
			break;
		
		case 2:
			/*HOME -> STEADY*/
			if(capit_pos == 0 && capit_stdy == false)
			{
				if(mode_buka == 0) htim4.Instance->CCR3 = BUKA_CAPIT1;
				else if(mode_buka == 1) htim4.Instance->CCR3 = BUKA_CAPIT2;
				
				for(i=0; i<=deltaCOXA_home_stdy; i+=10)
				{
					htim4.Instance->CCR1 = HOME_COXA_CAPIT1 + i;
					
					HAL_Delay(set_speed/2);
				}
				
				for(i=0; i<=deltaFEMUR_home_stdy; i+=5)
				{
					htim4.Instance->CCR2 = HOME_FEMUR_CAPIT1 - i;
					
					HAL_Delay(set_speed/2);
				}
				
				capit_stdy = true;
			}
			
			/*NYAPIT -> STEADY*/
			else if(capit_pos == 1 && capit_stdy == false)
			{
				if(mode_buka == 0) htim4.Instance->CCR3 = BUKA_CAPIT1;
				else if(mode_buka == 1) htim4.Instance->CCR3 = BUKA_CAPIT2;
				
				for(i=0; i<=deltaCOXA_stdy_start; i++)
				{
					htim4.Instance->CCR1 = NYAPIT_COXA_CAPIT + i;
					
					HAL_Delay(set_speed * 0.3);
				}
				
				for(i=0; i<=deltaFEMUR_stdy_start; i++)
				{
					htim4.Instance->CCR2 = NYAPIT_FEMUR_CAPIT + i;
					
					HAL_Delay(set_speed * 0.3);
				}
				
				capit_stdy = true;
			}
			
			if(capit_stdy == true)
			{
				poscapit_stdy();
				capit_home = false;
				capit_nyapit = false;
				capit_evakuasi = false;
				capit_evakuasihalf = false;
			}
			break;
			
		case 3:
			/*STEADY -> NYAPIT*/
			if(capit_pos == 0 && capit_nyapit == false)
			{
				if(mode_buka == 0) htim4.Instance->CCR3 = BUKA_CAPIT1;
				else if(mode_buka == 1) htim4.Instance->CCR3 = BUKA_CAPIT2;
				
				for(i=0; i<=deltaCOXA_stdy_start; i++)
				{
					htim4.Instance->CCR1 = STDY_COXA_CAPIT - i;
					
					HAL_Delay(set_speed * 0.3);
				}
				
				for(i=0; i<=deltaFEMUR_stdy_start; i++)
				{
					htim4.Instance->CCR2 = STDY_FEMUR_CAPIT - i;
					
					HAL_Delay(set_speed * 0.3);
				}
				
				capit_nyapit = true;
			}
			
			/*EVAKUASI -> NYAPIT*/
			else if(capit_pos == 1 && capit_nyapit == false)
			{
				htim4.Instance->CCR3 = TUTUP_CAPIT;
				
				for(i=0; i<=deltaCOXA_home_start; i+=5)
				{
					htim4.Instance->CCR1 = HOME_COXA_CAPIT1 + i;

					HAL_Delay(set_speed/5);
				}
				
				for(i=0; i<=deltaFEMUR_home_start; i+=5)
				{
					htim4.Instance->CCR2 = HOME_FEMUR_CAPIT1 - i;
					
					HAL_Delay(set_speed/5);
				}
				
				for(i=0; i<=20; i++)
				{
					htim4.Instance->CCR1 = STDY_COXA_CAPIT - i;
					htim4.Instance->CCR2 = STDY_FEMUR_CAPIT - i;
					
					HAL_Delay(set_speed * 0.6);
				}
				
				capit_nyapit = true;
			}
			
			if(capit_pos == 0 && capit_nyapit == true)
			{
				poscapit_nyapit(1);
				capit_home = false;
				capit_stdy = false;
				capit_evakuasi = false;
				capit_evakuasihalf = false;
			}
			
			else if(capit_pos == 1 && capit_nyapit == true)
			{
				HAL_Delay(1000);
				poscapit_nyapit(2);
				capit_home = false;
				capit_stdy = false;
				capit_evakuasi = false;
				capit_evakuasihalf = false;
			}
			break;
		
		case 4:
			/*NYAPIT -> EVAKUASI_HALF*/
			if(capit_pos == 0 && capit_evakuasihalf == false && capit_home == false)
			{
				for(i=0; i<=deltaCOXA_stdy_start; i++)
				{
					htim4.Instance->CCR1 = NYAPIT_COXA_CAPIT + i;
					
					HAL_Delay(set_speed * 0.3);
				}
				
				for(i=0; i<=deltaFEMUR_stdy_start; i++)
				{
					htim4.Instance->CCR2 = NYAPIT_FEMUR_CAPIT + i;
					
					HAL_Delay(set_speed * 0.3);
				}
				
				for(i=0; i<=160; i+=5)
				{ 
					if(i <= 115) 
					{
						htim4.Instance->CCR2 = STDY_FEMUR_CAPIT - i;
						htim4.Instance->CCR1 = STDY_COXA_CAPIT - i;
					}
					
					else{
						htim4.Instance->CCR1 = STDY_COXA_CAPIT - i;
					}
					
					HAL_Delay(set_speed);
				}
				 
				capit_evakuasihalf = true;
			}
			
			/*HOME -> EVAKUASI HALF*/
			else if(capit_pos == 0 && capit_evakuasihalf == false && capit_home == true)
			{
				htim4.Instance->CCR3 = TUTUP_CAPIT;
				
				for(i=0; i<=deltaCOXA_evakuasihalf_home; i+=5)
				{
					htim4.Instance->CCR1 = HOME_COXA_CAPIT1 + i;
					
					HAL_Delay(set_speed/2);
				}	
				
				for(i=0; i<=deltaFEMUR_evakuasihalf_home; i+=5)
				{
					htim4.Instance->CCR2 = HOME_FEMUR_CAPIT1 - i;
					
					HAL_Delay(set_speed/2);
				}	
				
				capit_evakuasihalf = true;
			}
			
			if(capit_pos == 1)
			{
				capit_evakuasihalf = true;
			}
			
			if(capit_evakuasihalf == true)
			{
				poscapit_evakuasi(2);
				capit_home = false;
				capit_stdy = false;
				capit_nyapit = false;
				capit_evakuasi = false;
			}
			break;
			
		case 5:
			/*NYAPIT -> EVAKUASI*/
			if(capit_evakuasihalf == false)
			{
				if(capit_pos == 0 && capit_evakuasi == false)
				{
					for(i=0; i<=20; i++)
					{
						htim4.Instance->CCR2 = NYAPIT_FEMUR_CAPIT + i;
						htim4.Instance->CCR1 = NYAPIT_COXA_CAPIT + i;
						
						HAL_Delay(set_speed * 0.6); 
					}
					
					for(i=0; i<=deltaFEMUR_home_stdy; i+=5)
					{ 
						htim4.Instance->CCR2 = STDY_FEMUR_CAPIT + i;
						
						HAL_Delay(set_speed/2);
					}
					
					for(i=0; i<=deltaCOXA_home_stdy; i+=10)
					{ 
						htim4.Instance->CCR1 = STDY_COXA_CAPIT - i;
						
						HAL_Delay(set_speed/2);
					}
					
					capit_evakuasi = true;
				}
				
				else if(capit_pos == 1)
				{
					capit_evakuasi = true;
				}
				
				if(capit_evakuasi == true)
				{
					poscapit_evakuasi(1);
					capit_home = false;
					capit_stdy = false;
					capit_nyapit = false;
					capit_evakuasihalf = false;
				}
			}
			
			else if(capit_evakuasihalf == true)
			{
				if(capit_pos == 0 && capit_evakuasi == false)
				{
					for(i=0; i<=295; i+=5)
					{ 
						htim4.Instance->CCR2 = HOME_FEMUR_CAPIT2 + i;
						
						HAL_Delay(set_speed/2);
					}
					
					capit_evakuasi = true;
				}
				
				if(capit_evakuasi == true)
				{
					poscapit_evakuasi(1);
					capit_home = false;
					capit_stdy = false;
					capit_nyapit = false;
					capit_evakuasihalf = false;
				}
			}
			break;
	}
}

/*KONTROL CAPIT & SEROK*/
void capit_serok(int state_capit)
{
	/*CAPIT SEROK HOME*/
	if(state_capit == 0)
	{
		htim12.Instance->CCR1 = CAPIT_BAWAH_NAIK;
		htim12.Instance->CCR2 = BUKA_CAPIT_BAWAH2;
	}
	
	/*CAPIT SEROK STEADY*/
	if(state_capit == 1)
	{
		htim12.Instance->CCR1 = CAPIT_BAWAH_TURUN;
		htim12.Instance->CCR2 = BUKA_CAPIT_BAWAH2;
	} 
	
	/*CAPIT SEROK START*/
	if(state_capit == 2)
	{
		htim12.Instance->CCR1 = CAPIT_BAWAH_TURUN;
		htim12.Instance->CCR2 = TUTUP_CAPIT_BAWAH;
	}
	
	/*CAPIT SEROK EVAKUASI*/
	if(state_capit == 3)
	{
		htim12.Instance->CCR2 = TUTUP_CAPIT_BAWAH;
		htim12.Instance->CCR1 = CAPIT_BAWAH_NAIK;
	}
}

void buzzer_seq()
{
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	htim14.Instance->CCR1 = 5;
	HAL_Delay(100);
	
	HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
	HAL_Delay(100);
	
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	htim14.Instance->CCR1 = 5;
	HAL_Delay(100);

	HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	/*---START SETUP MPU---*/
	#ifdef GYRO_ON
	while(get_status.gyro_stat != INITIALIZE_GYRO_OK && get_status.mag_stat != INITIALIZE_GYRO_OK)
	{
		CHECK_MPU_WHO_AM_I(&hi2c1, &get_status);
		CHECK_AK8963_WHO_AM_I(&hi2c1, &get_status); 
	}
	MPU9250_Init(&hi2c1, &get_status);
	MPU9250_Callibrate_Gyro(&hi2c1, &MPU9250, 50);
	#endif
	/*---END SETUP MPU---*/
	
	
	/*---START SETUP UART---*/
	#ifdef KOMUNIKASI_ON
	komunikasi_init(&huart6);
	rx_start_get();
	#endif
	/*---END SETUP UART---*/

	
	/*---START SETUP BUZZER---*/
	#ifdef BUZZER_ON
	buzzer_seq();
	#endif
	/*---END SETUP BUZZER---*/
	
	/*---START SETUP PWM---*/
	/*KANAN DEPAN*/
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	
	/*KANAN TENGAH*/
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	
	/*KANAN BELAKANG*/
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	
	/*KIRI BELAKANG*/
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	
	/*KIRI TENGAH*/
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	
	/*KIRI DEPAN*/
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	
	/*CAPIT*/
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	
	/*CAPIT SEROK*/
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	/*---END SETUP PWM---*/
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	{
		run_state = HAL_GPIO_ReadPin(GPIOE, TOGGLE_Pin);
		
		/*SET POSISI KAKI INDEPENDENT*/
		#ifdef INDEPENDENT_POS
		pos_nanjak(1);
//		/*Kanan Depan*/
//		independent_pos(0, 80, 70, -100);
//		/*Kanan Tengah*/
//		independent_pos(1, 90, 0, -100); 
//		/*Kanan Belakang*/
//		independent_pos(2, 80, -70, -100); 
//		/*Kiri Belakang*/
//		independent_pos(3, -80, -70, -80); 
//		/*Kiri Tengah*/
//		independent_pos(4, -80, 0, -80); 
//		/*kiri Depan*/
//		independent_pos(5, -80, 70, -80); 
		#endif
		
		/*SET POSISI KAKI NON INDEPENDENT*/
		#ifdef NON_INDEPENDENT_POS
		titik_awal(POSISI_STDY_X, POSISI_STDY_Y, POSISI_STDY_Z, 1);
		#endif
		
		if(run_state == GPIO_PIN_RESET)
		{
			boot_mode = 1;
			boot = true;
			boot_limit = false;

/*----TOGGLE PING ON----*/
			#ifdef TOGGLE_ON
			if(fdbck_atas.statis != 0x01 && ping_atas == false)
			{
				tx_ping();
			}
			
			else if(fdbck_atas.statis == 0x01)
			{
				ping_atas = true;
			}
			#endif
			
/*----TESTING KOMUNIKASI----*/
			#ifdef KOMUNIKASI_ON
			/*---MODE STANDBY---*/
			if(data_rx.type == MOVE_STEADY)
			{
				pos_correction(10);
			}
			
			/*---MODE JALAN BIASA---*/
			else if((data_rx.type == MOVE_JALAN) && (data_rx.mode_jalan == JALAN_NOT_FOUND || data_rx.mode_jalan == JALAN_NORMAL))
			{
				set_langkah = data_rx.walkpoint;
				titik_awal(rx_posSTDX, rx_posSTDY, rx_posSTDZ, 1);
				
				jalan(data_rx.pos_x, data_rx.pos_y, data_rx.pos_z, data_rx.speed, 1);
			}
			
			/*---MODE JALAN BIASA DENGAN STABILIZER---*/
			else if((data_rx.type == MOVE_JALAN_STABIL) && (data_rx.mode_jalan == JALAN_NOT_FOUND || data_rx.mode_jalan == JALAN_NORMAL))
			{
				set_langkah = data_rx.walkpoint;
				titik_awal(rx_posSTDX, rx_posSTDY, rx_posSTDZ, 1);
				Y_corection = data_rx.y_correction;
				
				jalan(data_rx.pos_x, data_rx.pos_y, data_rx.pos_z, data_rx.speed, 2);
			}
			
			/*---MODE JALAN MENAIKI TANGGA---*/
			else if(data_rx.type == MOVE_JALAN && data_rx.mode_jalan == JALAN_TANGGA)
			{
				set_langkah = data_rx.walkpoint;
				
				if(data_rx.nanjak_mode == NANJAK_KANAN) mode_nanjak = 0;
				else if(data_rx.nanjak_mode == NANJAK_KIRI) mode_nanjak = 1;
				
				pos_nanjak(mode_nanjak);
				jalan(data_rx.pos_x, data_rx.pos_y, data_rx.pos_z, data_rx.speed, 1);
			}
			
			/*---MODE JALAN JEMBATAN SI BAGONG---*/
			else if(data_rx.type == MOVE_JALAN && data_rx.mode_jalan == JALAN_MIRING)
			{
				set_langkah = data_rx.walkpoint;
				
				if(data_rx.nanjak_mode == NANJAK_KANAN) mode_nanjak = 2;
				else if(data_rx.nanjak_mode == NANJAK_KIRI) mode_nanjak = 3;
				
				pos_nanjak(mode_nanjak);
				
				jalan(data_rx.pos_x, data_rx.pos_y, data_rx.pos_z, data_rx.speed, 1);
			}
			
			/*---MODE GERAK TRANSLASI---*/
			else if(data_rx.type == MOVE_TRANSLASI)
			{
				posZ_lean[0] = rx_posSTDZ, posZ_lean[1] = rx_posSTDZ, posZ_lean[2] = rx_posSTDZ;
				posZ_lean[3] = rx_posSTDZ, posZ_lean[4] = rx_posSTDZ, posZ_lean[4] = rx_posSTDZ;
				
				/*LEAN KE DEPAN*/
				if(data_rx.skew_mode == MIRING_DEPAN)
				{
					rx_leanvalXY 	= data_rx.skew_value_xy;
					rx_leanvalZ = data_rx.skew_value_z;
					rx_setspeed	= data_rx.time;
					
					lean(rx_leanvalXY, rx_leanvalZ, 1, rx_setspeed);
				}
				
				/*LEAN KE BELAKANG*/
				else if(data_rx.skew_mode == MIRING_BELAKANG)
				{
					rx_leanvalXY 	= data_rx.skew_value_xy;
					rx_leanvalZ = data_rx.skew_value_z;
					rx_setspeed	= data_rx.time;
					
					lean(rx_leanvalXY, rx_leanvalZ, 2, rx_setspeed);
				}
				
				/*LEAN KE KANAN*/
				else if(data_rx.skew_mode == MIRING_KANAN)
				{
					rx_leanvalXY 	= data_rx.skew_value_xy;
					rx_leanvalZ = data_rx.skew_value_z;
					rx_setspeed	= data_rx.time;
					
					lean(rx_leanvalXY, rx_leanvalZ, 3, rx_setspeed);
				}
				
				/*LEAN KE KIRI*/
				else if(data_rx.skew_mode == MIRING_KIRI)
				{
					rx_leanvalXY 	= data_rx.skew_value_xy;
					rx_leanvalZ = data_rx.skew_value_z;
					rx_setspeed	= data_rx.time;
					
					lean(rx_leanvalXY, rx_leanvalZ, 4, rx_setspeed);
				}
			}
			
			/*---MODE GERAK ROTASI---*/
			else if(data_rx.type == MOVE_ROTASI)
			{
				set_langkah = data_rx.walkpoint;
				titik_awal(rx_posSTDX, rx_posSTDY, rx_posSTDZ, 1);
				
				if(data_rx.mode_rotasi == ROTASI_FULL)
				{
					rotasi(data_rx.roll, data_rx.pitch, data_rx.yaw, data_rx.pos_z, 1, data_rx.speed);		
				}
				
				else if(data_rx.mode_rotasi == ROTASI_HALF)
				{
					rotasi(data_rx.roll, data_rx.pitch, data_rx.yaw, data_rx.pos_z, 2, data_rx.speed);
				}
			}				
			
			/*---SEND REQ MPU DATA---*/
			else if(data_rx.type == SEND_REQ)
			{
				MPU9250_Read_Data(&hi2c1, &MPU9250);
			
				dataGX = MPU9250.KalmanAngleX;
				dataGY = MPU9250.KalmanAngleY;
				dataGZ = MPU9250.Gz;

				dataMX = MPU9250.Call_MagX;
				dataMY = MPU9250.Call_MagY;
				dataMZ = MPU9250.Call_MagZ;
				
				tx_send_mpu(dataGX, dataGY, dataGZ, dataMX, dataMY, dataMZ);
			}
			
			/*---REQUEST GYRO DATA---*/
			else if(data_rx.type == SEND_REQ)
			{
				MPU9250_Read_Data(&hi2c1, &MPU9250);
				
				dataGX = MPU9250.KalmanAngleX;
				dataGY = MPU9250.KalmanAngleY;
				dataGZ = MPU9250.Gz;
			}
			
			/*---POSISI STEADY---*/
			else if(data_rx.type == GET_STATIS)
			{
				rx_posSTDX = data_rx.pos_x;
				rx_posSTDY = data_rx.pos_y;
				rx_posSTDZ = data_rx.pos_z;
				
				titik_awal(rx_posSTDX, rx_posSTDY, rx_posSTDZ, 1);
			}
			
			/*---KONTROL CAPIT---*/
			else if(data_rx.type == PLAY_CAPIT)
			{				
				capit_speed = data_rx.speed_capit;
				
				if(data_rx.status == CAPIT_KOSONG) capit_pos = 0;
				else if(data_rx.status == CAPIT_KORBAN) capit_pos = 1;
				
				if(data_rx.mode_capitstdy == CAPIT_FULL || data_rx.mode_capitstdy == CAPIT_NONE) mode_buka = 0;
				if(data_rx.mode_capitstdy == CAPIT_NDAKFULL) mode_buka = 1;
				
				/*SEQUENCE PENUH AMBIL KORBAN*/
				if(data_rx.cmd == AMBIL_KORBAN)
				{
					capit_pos = 0;
					capit_speed = data_rx.speed_capit;
					
					if(bawa_korban == false)
					{
						nyapit(1, capit_speed);
						nyapit(2, capit_speed);
						nyapit(3, capit_speed);
						HAL_Delay(500);
						nyapit(5, capit_speed);
						
						bawa_korban = true;
					}
					
					poscapit_evakuasi(1);
				}
				
				/*SEQUENCE PENUH MELETAKAN KORBAN*/
				else if(data_rx.cmd == PENYELAMATAN_KORBAN)
				{
					capit_pos = 1;
					capit_speed = data_rx.speed_capit;
					
					if(bawa_korban == true)
					{
						nyapit(5, capit_speed);
						nyapit(3, capit_speed);
						HAL_Delay(500);
						nyapit(2, capit_speed);
						nyapit(1, capit_speed);
						
						bawa_korban = false;
					}
					
					poscapit_home();
				}
				
				/*CAPIT HOME*/
				else if(data_rx.cmd == HOME_CAPIT) nyapit(1, capit_speed);
				
				/*CAPIT STEADY*/
				else if(data_rx.cmd == STEADY_CAPIT) nyapit(2, capit_speed);
				
				/*CAPIT START*/
				else if(data_rx.cmd == START_CAPIT) nyapit(3, capit_speed);
				
				/*CAPIT EVAKUASI HALF*/
				else if(data_rx.cmd == EVAKUASI_CAPIT_HALF) nyapit(4, capit_speed);
				
				/*CAPIT EVAKUASI*/
				else if(data_rx.cmd == EVAKUASI_CAPIT) nyapit(5, capit_speed);
			}
			
			/*---KONTROL CAPIT SEROK---*/
			else if(data_rx.type == PLAY_SEROK)
			{
				if(data_rx.pos_capit == CAPIT_SEROK_HOME)
				{
					capit_serok(0);
				}
				
				else if(data_rx.pos_capit == CAPIT_SEROK_STEADY)
				{
					capit_serok(1);
				}
				
				else if(data_rx.pos_capit == CAPIT_SEROK_START)
				{
					capit_serok(2);
				}
				
				else if(data_rx.pos_capit == CAPIT_SEROK_EVAKUASI)
				{
					capit_serok(3);
				}
			}
			
			/*---SINKRONISASI LANGKAH TERPENUHI---*/
			if(langkah > 0 && langkah == set_langkah)
			{
				stop_gerak();
				
				uint8_t info_langkah[3] = {0xA5, 0x5A, 0x01};
				HAL_UART_Transmit(&huart6, info_langkah, 3, 10);
				
				langkah = 0;
			}
			#endif


/*----TESTING GERAK TRANSLASI----*/
			#ifdef TEST_TRANSLASI
			
//			if(mode_jalan == 0)
//			{
//				
//				set_langkah = 5;
//				Y_corection = -20;
//				jalan(0, 40, 50, 15, 2);
//				
//				if(langkah == set_langkah) mode_jalan = 1, langkah = 0;
//			}
//			
//			if(mode_jalan == 1)
//			{
//				set_langkah = 5;	
//				Y_corection = -20;
//				jalan(0, 40, 50, 15, 2);
//				
//				if(langkah == set_langkah) mode_jalan = 2, langkah = 0;
//			}
//			
//			if(mode_jalan == 2)
//			{
//				stop_gerak();
//				HAL_Delay(2500);
//				mode_jalan = 0;
//			}

//			if(langkah == set_langkah)
//			{ 
//				stop_gerak();
//				HAL_Delay(2500);
//				langkah = 0;
//			}
			jalan(0, 0, 65, 10, 1);
			#endif
				
/*----TESTING GERAK ROTASI----*/
			#ifdef TEST_ROTASI
			set_langkah = 4;
			
			rotasi(0, 0, 35, 50, 1, 7);

			if(langkah == set_langkah)
			{ 
				stop_gerak();
				HAL_Delay(2500);
				langkah = 0;
			}
			#endif
				
/*----TESTING NANJAK----*/
			#ifdef TEST_NANJAK
			set_langkah = 20;

			pos_nanjak(1);
//			jalan(-35, 0, 65, 15, 1);
//		
//			if(langkah == set_langkah)
//			{ 
//				stop_gerak();
//				HAL_Delay(2000);
//				langkah = 0;
//			}

//			if(mode_jalan == 0)
//			{
//				set_langkah = 10;
//				pos_nanjak(1);
//				jalan(35, 0, 60, 20, 1);
//					
//				if(langkah == set_langkah) mode_jalan = 1, langkah = 0;
//			}			

//			else if(mode_jalan == 1)
//			{
//				set_langkah = 30;
//				pos_nanjak(1);
//				jalan(35, 0, 60, 15, 1);
//					
//				if(langkah == set_langkah) mode_jalan = 2, langkah = 0;
//			}
//			
//			if(mode_jalan == 2)
//			{ 
//				stop_gerak();
//				HAL_Delay(2000);
//				mode_jalan = 0;
//				langkah = 0;
//			}
			#endif

/*----TESTING JALAN-JALAN----*/
			#ifdef TEST_GERAKAN
			if(mode_jalan == 0)
			{
				set_langkah = 5;
				jalan(0, 10, 60, 10, 1);
				
				if(langkah == set_langkah) mode_jalan = 1, langkah = 0;;
			}
			
			else if(mode_jalan == 1)
			{
				set_langkah = 5;
				jalan(0, -10, 60, 10, 1);
				
				if(langkah == set_langkah) mode_jalan = 2, langkah = 0;;
			}
			
			if(mode_jalan == 2)
			{
				stop_gerak();
				HAL_Delay(2000);
				mode_jalan = 0;
			}
			#endif

/*----TEST CAPIT 1----*/
			#ifdef TEST_GERAKCAPIT
//			capit_pos = 0;
//			nyapit(1, 10); 		//tx_capit(HOME_CAPIT, CAPIT_KOSONG, 15, STEADY_NON);
//			HAL_Delay(1000);
//			nyapit(2, 10);		//tx_capit(STEADY_CAPIT, CAPIT_KOSONG, 15, STEADY_FULL);
//			HAL_Delay(1000);
//			nyapit(3, 10);		//tx_capit(START_CAPIT, CAPIT_KOSONG, 15, STEADY_NON);
//			HAL_Delay(1000);
//			nyapit(4, 10);
//			HAL_Delay(3000);
//			nyapit(5, 10);		//tx_capit(EVAKUASI_CAPIT, CAPIT_KOSONG, 15, STEADY_NON);
//			
//			HAL_Delay(1000);
//			
//			capit_pos = 1;
//			nyapit(5, 10);		//tx_capit(EVAKUASI_CAPIT, CAPIT_KORBAN, 15, STEADY_NON);
//			HAL_Delay(1000);
//			nyapit(3, 10);		//tx_capit(START_CAPIT, CAPIT_KORBAN, 15, STEADY_NON);
//			HAL_Delay(1000);
//			nyapit(2, 10);		//tx_capit(STEADY_CAPIT, CAPIT_KORBAN, 15, STEADY_FULL);
//			HAL_Delay(1000);
//			nyapit(1, 10);		//tx_capit(HOME_CAPIT, CAPIT_KORBAN, 15, STEADY_NON);
//		
//			HAL_Delay(1000);
			
			capit_serok(0);
			HAL_Delay(500);
			capit_serok(1);
			HAL_Delay(500);
			capit_serok(2);
			HAL_Delay(500);
			capit_serok(3);
			
			HAL_Delay(1000);
			
			capit_serok(3);
			HAL_Delay(500);
			capit_serok(2);
			HAL_Delay(500);
			capit_serok(1);
			HAL_Delay(500);
			capit_serok(0);
			
			HAL_Delay(1000);
			#endif

/*----TEST LEANING----*/
			#ifdef TEST_LEANING
			lean(0, 20, 4, 10);
			#endif
			
/*----RANDOM TESTING----*/
			#ifdef SET_RANDOM_TESTING
			jalan(0, 0, 60, 20, 1);
			#endif
		}
				
		else if(run_state == GPIO_PIN_SET)
		{
	
/*----POSISI AWAL DARI STM ATAS----*/
			#ifdef RX_POS
			if(data_rx.type == GET_STATIS)
			{
				rx_posSTDX = data_rx.pos_x;
				rx_posSTDY = data_rx.pos_y;
				rx_posSTDZ = data_rx.pos_z;
				
				titik_awal(rx_posSTDX, rx_posSTDY, rx_posSTDZ, 1); 
			}
			#endif

/*----POSISI NOL KAKI----*/
			#ifdef POSISI_NOL
			pos_nol();
			#endif

/*----POSISI ROBOT STEADY----*/			
			#ifdef POSISI_STEADY
			pos_steady();
//			poscapit_home();
//			capit_serok(0);
			#endif

/*----POSISI KALIBRASI KAKI----*/
			#ifdef KALIBRASI_KAKI
			pos_kal();
			#endif
			
/*----POSISI KALIBRASI CAPIT----*/
			#ifdef KALIBRASI_CAPIT
			poscapit_kal();
			#endif
		
/*----CEK POSISI NANJAK----*/
			#ifdef CEK_BODY_NANJAK
			//pos_nanjak(1);
			/*Kanan Depan*/
			independent_pos(0, 80, 70, -40);
			/*Kanan Tengah*/
			independent_pos(1, 80, 0, -100); 
			/*Kanan Belakang*/
			independent_pos(2, 80, -70, -100); 
			/*Kiri Belakang*/
			independent_pos(3, -88, -70, -100); 
			/*Kiri Tengah*/
			independent_pos(4, -88, 0, -100); 
			/*Kiri Depan*/
			independent_pos(5, -88, 70, -100);
			
			pos_steady();
			#endif
			
/*----CEK POSISI HOME CAPIT----*/
			#ifdef CEK_CAPIT_HOME
			poscapit_home();
			#endif
			
/*----CEK POSISI STEADY CAPIT----*/
			#ifdef CEK_CAPIT_STEADY
			poscapit_steady();
			#endif
			
/*----CEK POSISI NYAPIT 1 CAPIT----*/
			#ifdef CEK_CAPIT_NYAPIT1
			poscapit_nyapit(1);
			#endif
			
/*----CEK POSISI NYAPIT 1 CAPIT----*/
			#ifdef CEK_CAPIT_NYAPIT2
			poscapit_nyapit(2);
			#endif
			
/*----CEK POSISI EVAKUASI----*/
			#ifdef CEK_CAPIT_EVAKUASI
			poscapit_evakuasi();
			#endif
		
/*----CEK POSISI CAPIT BAWAH HOME*/
			#ifdef CEK_CAPITBAWAH_HOME
			capit_serok(0);
			#endif

/*----CEK POSISI CAPIT BAWAH STEADY*/
			#ifdef CEK_CAPITBAWAH_STEADY
			capit_serok(1);
			#endif
			
/*----CEK POSISI CAPIT BAWAH START NYAPIT*/
			#ifdef CEK_CAPITBAWAH_NYAPIT
			capit_serok(2);
			#endif
			
/*----CEK POSISI CAPIT BAWAH EVAKUASI*/
			#ifdef CEK_CAPITBAWAH_EVAKUASI
			capit_serok(3);
			#endif
			
/*----TESTING GYRO----*/
			#ifdef TEST_GYRO 
			MPU9250_Read_Data(&hi2c1, &MPU9250); 
			
			dataGX = MPU9250.KalmanAngleX;
			dataGY = MPU9250.KalmanAngleY;
			dataGZ = MPU9250.Gz;
			#endif
			
/*----TESTING MAGNETOMETER----*/
			#ifdef TEST_MAG 
			//MPU9250_Read_MagData(&hi2c1, &MPU9250); 
			
			dataMX = MPU9250.Call_MagX;
			dataMY = MPU9250.Call_MagY;
			dataMZ = MPU9250.Call_MagZ;
			#endif
			
/*----TEST PID----*/
			#ifdef TEST_PID_GYRO
			check_angle_err();
			calculate_pid();
			#endif
			
/*----TEST STABILIZER----*/
			#ifdef TEST_STABILIZER
			MPU9250_Read_Data(&hi2c1, &MPU9250);
			
			dataGX = MPU9250.KalmanAngleX;
			dataGY = MPU9250.KalmanAngleY;
			dataGZ = MPU9250.Gz;
			
			check_angle_err();
			calculate_pid();

			titik_awal(80, 70, -100, 2);
			#endif
		
/*----GERAKAN BOOTING----*/
			#ifdef BOOTING_ON
			if(boot == true && boot_mode == 0)
			{
				boot_limit = false;
				do
				{
					booting_on(20);
					boot_limit = true;
				}while(boot_limit != true && boot_mode == 0);
				boot = false;
			}
			
			else if(boot == false && boot_limit == true && boot_mode == 0)
			{
				pos_steady();
			}
			
			if(boot == true && boot_mode == 1)
			{
				boot_limit = false;
//				do
//				{
//					booting_off(20);
//					boot_limit = true;
//				}while(boot_limit != true && boot_mode == 1);
				boot = false;
			}
			
			else if(boot == false && boot_limit == true && boot_mode == 1)
			{
				pos_nol();
			}
			#endif
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 240;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 240;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 240;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 262;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 240;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 240;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 240;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 240;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 240;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 1600;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 240;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 1230;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 255;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 576000;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, M2_Pin|M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LIMIT_F_Pin TOGGLE_Pin LIMIT_B_Pin */
  GPIO_InitStruct.Pin = LIMIT_F_Pin|TOGGLE_Pin|LIMIT_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : M2_Pin M1_Pin */
  GPIO_InitStruct.Pin = M2_Pin|M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
