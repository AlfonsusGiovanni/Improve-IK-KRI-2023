/*
 * Komunikasi.h
 *
 *  Created on: Mar 2, 2022
 *      Author: Maulana Reyhan Savero
 */
#ifndef KomunikasiRobot_H_
#define KomunikasiRobot_H_

#include "main.h"
#include <stdbool.h>

typedef enum{
	JALAN_NOT_FOUND = 0x00U,
	JALAN_PECAH = 0x01U,
	JALAN_KELERENG = 0x02U,
	JALAN_BATU = 0x03U,
	JALAN_NORMAL = 0x04U,
	JALAN_TANGGA = 0x05U,
	JALAN_MIRING = 0x06U
}mode_jalan_t;

typedef struct{
	bool ping;
	bool standby;
	bool jalan;
	bool translasi;
	bool rotasi;
	bool req;
	bool statis;
	bool capit;
	bool serok;
	bool jalan_stabil;
	bool send_req;
}feedback_t;

typedef enum{
	NO_SKEW = 0x00U,
	MIRING_DEPAN = 0x01U,
	MIRING_BELAKANG = 0x02U,
	MIRING_KANAN = 0x03U,
	MIRING_KIRI = 0x04U
}type_skew_t;

typedef enum{
	PING = 0x01U,
	MOVE_STEADY = 0x02U,
	MOVE_JALAN = 0x03U,
	MOVE_TRANSLASI = 0x04U,
	MOVE_ROTASI = 0x05U,
	SEND_REQ = 0x06U,
	GET_STATIS = 0x07U,
	PLAY_CAPIT = 0x08U,
	PLAY_SEROK = 0x09U,
	MOVE_JALAN_STABIL = 0x0A,
	GET_REQ_DATA = 0x0B
}type_jalan_t;

typedef enum{
	NANJAK_NONE = 0x00U,
	NANJAK_KIRI = 0x01U,
	NANJAK_KANAN = 0x02U
}type_mode_nanjak_t;

typedef enum{
	ROTASI_FULL = 0X01U,
	ROTASI_HALF = 0x02U
}type_mode_rotasi_t;

typedef enum{
	AMBIL_KORBAN = 0x01U,
	PENYELAMATAN_KORBAN = 0x02U,
	HOME_CAPIT = 0x03U,
	STEADY_CAPIT = 0x04U,
	START_CAPIT = 0x05U,
	EVAKUASI_CAPIT = 0x06U,
	EVAKUASI_CAPIT_HALF = 0x07U
}type_capit_t;

typedef enum{
	CAPIT_KORBAN = 0x01U,
	CAPIT_KOSONG = 0x02U,
}type_capit_status_t;

typedef enum{
	CAPIT_NONE = 0x00U,
	CAPIT_FULL = 0x01U,
	CAPIT_NDAKFULL = 0x02U,
}type_stdy_capit_t;

typedef enum{
	MOVE_DEPAN = 0x01U,
	MOVE_BELAKANG = 0x02U
}type_serok_t;

typedef enum{
	CAPIT_SEROK_HOME = 0x00U,
	CAPIT_SEROK_STEADY = 0x01U,
	CAPIT_SEROK_START = 0x02U,
	CAPIT_SEROK_EVAKUASI = 0x03U,
}type_capit_serok_t;

typedef struct{
	int16_t pos_x;
	int16_t pos_y;
	int16_t pos_z;
	int16_t y_correction;
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	int16_t roll_body;
	int16_t pitch_body;
	int16_t yaw_body;
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
	int8_t time;
	int8_t walkpoint;
	int8_t mode;
	type_serok_t move;
	type_jalan_t type;
	type_mode_nanjak_t nanjak_mode;
	type_mode_rotasi_t mode_rotasi;
	int8_t speed;
	mode_jalan_t mode_jalan;
	type_capit_t cmd;
	type_capit_serok_t pos_capit;
	type_capit_status_t status;
	type_stdy_capit_t mode_capitstdy;
	int8_t speed_capit;
	type_skew_t skew_mode;
	uint8_t skew_value_xy;
	uint8_t skew_value_z;
}com_get_t;

void komunikasi_init(UART_HandleTypeDef* uart_handler);
bool tx_ping(void);
static uint8_t checksum_generator(uint8_t* arr, uint8_t size);
bool tx_move_steady(void);
bool tx_move_jalan(int16_t pos_x, int16_t pos_y, int16_t pos_z, int8_t speed, mode_jalan_t mode, uint8_t walkpoint, type_mode_nanjak_t nanjak);
bool tx_move_jalan_stabil(int16_t pos_x, int16_t pos_y, int16_t pos_z, int16_t y_correction, int8_t speed, mode_jalan_t mode, uint8_t walkpoint);
bool tx_move_translasi(int8_t value_xy, int8_t value_z, int8_t skew_mode, uint8_t time);
bool tx_move_rotasi(int16_t roll, int16_t pitch, int16_t yaw, int16_t pos_z, type_mode_rotasi_t mode_rotasi, int8_t speed, uint8_t walkpoint);
void rx_start(void);
void rx_feedback(feedback_t* fed);
void rx_start_get(void);
void rx_get(com_get_t* get);
bool tx_req_mpu(void);
bool tx_send_mpu(int16_t roll_body, int16_t pitch_body, int16_t yaw_body, int16_t mag_x, int16_t mag_y, int16_t mag_z);
bool tx_statis(int16_t pos_x, int16_t pos_y, int16_t pos_z);
bool tx_capit(type_capit_t cmd, type_capit_status_t status, int8_t speed_capit, type_stdy_capit_t stdy_mode);
bool tx_capit_serok(type_capit_serok_t cmd_capit);
#endif
