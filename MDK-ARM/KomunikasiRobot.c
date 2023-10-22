/*
 * Komunikasi.c
 *
 *  Created on: Mar 2, 2022
 *      Author: Maulana Reyhan Savero
 */

#include "KomunikasiRobot.h"

#define TIMEOUT 10

static UART_HandleTypeDef* huart;
static uint8_t rxbuf[3];
static uint8_t rxbuf_get[16];

void komunikasi_init(UART_HandleTypeDef* uart_handler){
	huart = uart_handler;
}

static uint8_t checksum_generator(uint8_t* arr, uint8_t size){
	uint8_t chksm = 0;
	for(uint8_t i = 0; i < size; i++) chksm += arr[i];
	return (chksm & 0xFF);
}

bool tx_ping(void){
	uint8_t ping[16] = {0xA5, 0x5A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	ping[15] = checksum_generator(ping, 16);
	
	if(HAL_UART_Transmit(huart, ping, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_move_steady(void){
	uint8_t steady[16] = {0xA5, 0x5A, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	steady[15] = checksum_generator(steady, 16);
	
	if(HAL_UART_Transmit(huart, steady, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_move_jalan(int16_t pos_x, int16_t pos_y, int16_t pos_z, int8_t speed, mode_jalan_t mode, uint8_t walkpoint, type_mode_nanjak_t nanjak){
	uint8_t jalan[16] = {0xA5, 0x5A, 0x03, ((pos_x >> 8) & 0xFF),(pos_x & 0xFF),((pos_y >> 8) & 0xFF),(pos_y & 0xFF), ((pos_z >> 8) & 0xFF),(pos_z & 0xFF), speed, mode, walkpoint, nanjak, 0x00, 0x00, 0x00};
	jalan[15] = checksum_generator(jalan, 16);
		
	if(HAL_UART_Transmit(huart, jalan, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_move_translasi(int8_t value_xy, int8_t value_z, int8_t skew_mode, uint8_t time){
	uint8_t translasi[16] = {0xA5, 0x5A, 0x04, value_xy, value_z, skew_mode, time, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	translasi[15] = checksum_generator(translasi, 16);
		
	if(HAL_UART_Transmit(huart, translasi, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}
bool tx_move_rotasi(int16_t roll, int16_t pitch, int16_t yaw, int16_t pos_z, type_mode_rotasi_t mode_rotasi, int8_t speed, uint8_t walkpoint){
	uint8_t rotasi[16] = {0xA5, 0x5A, 0x05, ((roll >> 8) & 0xFF),(roll & 0xFF),((pitch >> 8) & 0xFF),(pitch & 0xFF),((yaw >> 8) & 0xFF),(yaw & 0xFF), ((pos_z >> 8) & 0xFF),(pos_z & 0xFF), mode_rotasi, speed, walkpoint, 0x00, 0x00};
	rotasi[15] = checksum_generator(rotasi, 16);
		
	if(HAL_UART_Transmit(huart, rotasi, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_statis(int16_t pos_x, int16_t pos_y, int16_t pos_z){
	uint8_t statis[16] = {0xA5, 0x5A, 0x07, ((pos_x >> 8) & 0xFF),(pos_x & 0xFF),((pos_y >> 8) & 0xFF),(pos_y & 0xFF), ((pos_z >> 8) & 0xFF),(pos_z & 0xFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	statis[15] = checksum_generator(statis, 16);
		
	if(HAL_UART_Transmit(huart, statis, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_capit(type_capit_t cmd, type_capit_status_t status, int8_t speed_capit, type_stdy_capit_t stdy_mode){
	uint8_t capit[16] = {0xA5, 0x5A, 0x08, cmd, status, speed_capit, stdy_mode, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	capit[15] = checksum_generator(capit, 16);
		
	if(HAL_UART_Transmit(huart, capit, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_capit_serok(type_capit_serok_t cmd_capit){
	uint8_t serok[16] = {0xA5, 0x5A, 0x09, cmd_capit, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	serok[15] = checksum_generator(serok, 16);
		
	if(HAL_UART_Transmit(huart, serok, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_move_jalan_stabil(int16_t pos_x, int16_t pos_y, int16_t pos_z, int16_t y_correction, int8_t speed, mode_jalan_t mode, uint8_t walkpoint){
	uint8_t jalan[16] = {0xA5, 0x5A, 0x0A, ((pos_x >> 8) & 0xFF),(pos_x & 0xFF),((pos_y >> 8) & 0xFF),(pos_y & 0xFF), ((pos_z >> 8) & 0xFF),(pos_z & 0xFF),((y_correction >> 8) & 0xFF),(y_correction & 0xFF), speed, mode, walkpoint, 0x00, 0x00};
	jalan[15] = checksum_generator(jalan, 16);
		
	if(HAL_UART_Transmit(huart, jalan, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_req_mpu(void)
{
	uint8_t req_data[16] = {0xA5, 0x5A, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	req_data[15] = checksum_generator(req_data, 16);
	
	if(HAL_UART_Transmit(huart, req_data, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

bool tx_send_mpu(int16_t roll_body, int16_t pitch_body, int16_t yaw_body, int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
	uint8_t send_data[16] = {0xA5, 0x5A, 0x0B, roll_body, pitch_body, yaw_body, mag_x, mag_y, mag_z, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	send_data[15] = checksum_generator(send_data, 16);
	
	if(HAL_UART_Transmit(huart, send_data, 16, TIMEOUT) == HAL_OK) return true;
	else return false;
}

void rx_start(void){
	HAL_UART_Receive_DMA(huart,rxbuf, 3);
}

void rx_start_get(void){
	HAL_UART_Receive_DMA(huart,rxbuf_get, 16);
}

void rx_feedback(feedback_t* fed){
	if(rxbuf[0] == 0xA5 && rxbuf[1]  == 0x5A){
		if(rxbuf[2] == 0x01) fed->ping = true;
		else if(rxbuf[2] == 0x02) fed->standby = true;
		else if(rxbuf[2] == 0x03) fed->jalan = true;
		else if(rxbuf[2] == 0x04) fed->translasi = true;
		else if(rxbuf[2] == 0x05) fed->rotasi = true;
		else if(rxbuf[2] == 0x06) fed->req = true;
		else if(rxbuf[2] == 0x07) fed->statis = true;
		else if(rxbuf[2] == 0x08) fed->capit = true;
		else if(rxbuf[2] == 0x09) fed->serok = true;
		else if(rxbuf[2] == 0x0A) fed->jalan_stabil = true;
		else if(rxbuf[2] == 0x0B) fed->send_req = true;
	}
	HAL_UART_Receive_DMA(huart,rxbuf, 3);
}

void rx_get(com_get_t* get){
	for(int i = 0; i < 16; i++){
		if((rxbuf_get[i] == 0xA5) && (rxbuf_get[i+1] == 0x5A)){
			
			// Check for ping
			if(rxbuf_get[i+2] == 0x01){
				get->type = PING;
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x01};
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
			}
			
			// Check for Move Steady
			else if(rxbuf_get[i+2] == 0x02){
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x02};
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->type = MOVE_STEADY;
			}
			
			// Check for Move Jalan
			else if(rxbuf_get[i+2] == 0x03){
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x03};
				
				// Check negative value
				if((rxbuf_get[i+3] & 0x80)) get->pos_x = ((rxbuf_get[i+3] << 8) | rxbuf_get[i+4])-(65536);
				else get->pos_x = (rxbuf_get[i+3] << 8) | rxbuf_get[i+4];
				
				// Check negative value
				if(rxbuf_get[i+5] & 0x80) get->pos_y = ((rxbuf_get[i+5] << 8) | rxbuf_get[i+6])-(65536);
				else get->pos_y = ((rxbuf_get[i+5] << 8) | rxbuf_get[i+6]);
				
				// Check negative value
				if(rxbuf_get[i+7] & 0x80) get->pos_z = ((rxbuf_get[i+7] << 8) | rxbuf_get[i+8])-(65536);
				else get->pos_z = (rxbuf_get[i+7] << 8) | rxbuf_get[i+8];
				
				// Check negative value
				if(rxbuf_get[i+9] & 0x80) get->speed =  (rxbuf_get[i+9])-(256);
				else get->speed =  rxbuf_get[i+9];
				
				// Get Mode
				get->mode_jalan = rxbuf_get[i+10];
				
				// Get Langkah Kaki
				get->walkpoint = rxbuf_get[i+11];
				
				// Get Mode Nanjak
				get->nanjak_mode = rxbuf_get[i+12];
				
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->type = MOVE_JALAN;
			}
			
			// Check for Translasi
			else if(rxbuf_get[i+2] == 0x04){
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x04};
				
				// Check negative value
				if(rxbuf_get[i+3] & 0x80) get->skew_value_xy =  (rxbuf_get[i+3]) - (256);
				else get->skew_value_xy =  (rxbuf_get[i+3]);
				
				// Check negative value
				if(rxbuf_get[i+4] & 0x80) get->skew_value_z =  (rxbuf_get[i+4]) - (256);
				else get->skew_value_z =  (rxbuf_get[i+4]);
				
				// Get Mode Skew
				get->skew_mode = rxbuf_get[i+5];
				
				// Check negative value
				get->time =  (rxbuf_get[i+6]);
				
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->type = MOVE_TRANSLASI;
			}
			
			// Check for Rotasi
			else if(rxbuf_get[i+2] == 0x05){
				
				// Check negative value
				if(rxbuf_get[i+3] & 0x80) get->roll = ((rxbuf_get[i+3] << 8) | rxbuf_get[i+4])-(65536);
				else get->roll = ((rxbuf_get[i+3] << 8) | rxbuf_get[i+4]);
				
				// Check negative value
				if(rxbuf_get[i+5] & 0x80) get->pitch = ((rxbuf_get[i+5] << 8) | rxbuf_get[i+6])-(65536);
				else get->pitch = ((rxbuf_get[i+5] << 8) | rxbuf_get[i+6]);
				
				// Check negative value
				if(rxbuf_get[i+7] & 0x80) get->yaw = ((rxbuf_get[i+7] << 8) | rxbuf_get[i+8])-(65536);
				else get->yaw = ((rxbuf_get[i+7] << 8) | rxbuf_get[i+8]);
				
				// Check negative value
				if(rxbuf_get[i+9] & 0x80) get->pos_z = ((rxbuf_get[i+9] << 8) | rxbuf_get[i+10])-(65536);
				else get->pos_z = ((rxbuf_get[i+9] << 8) | rxbuf_get[i+10]);
				
				// Check negative value
				if(rxbuf_get[i+11] & 0x80) get->mode_rotasi =  (rxbuf_get[i+11])-(256);
				else get->mode_rotasi =  rxbuf_get[i+11];
				
				// Check negative value
				if(rxbuf_get[i+12] & 0x80) get->speed =  (rxbuf_get[i+12])-(256);
				else get->speed =  (rxbuf_get[i+12]);
				
				// Get Langkah Kaki
				get->walkpoint = rxbuf_get[i+13];
				
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x05};
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->type = MOVE_ROTASI;
			}
			
			// Check for Req
			else if(rxbuf_get[i+2] == 0x06){
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x06};
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->type = SEND_REQ;
			}
			
			// Check for Statis
			else if(rxbuf_get[i+2] == 0x07){
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x07};
				
				// Check negative value
				if((rxbuf_get[i+3] & 0x80)) get->pos_x = ((rxbuf_get[i+3] << 8) | rxbuf_get[i+4])-(65536);
				else get->pos_x = (rxbuf_get[i+3] << 8) | rxbuf_get[i+4];
				
				// Check negative value
				if(rxbuf_get[i+5] & 0x80) get->pos_y = ((rxbuf_get[i+5] << 8) | rxbuf_get[i+6])-(65536);
				else get->pos_y = ((rxbuf_get[i+5] << 8) | rxbuf_get[i+6]);
				
				// Check negative value
				if(rxbuf_get[i+7] & 0x80) get->pos_z = ((rxbuf_get[i+7] << 8) | rxbuf_get[i+8])-(65536);
				else get->pos_z = (rxbuf_get[i+7] << 8) | rxbuf_get[i+8];
				
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->type = GET_STATIS;
			}
			
			// Check for Capit
			else if(rxbuf_get[i+2] == 0x08){
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x08};
				
				// Check command
				if(rxbuf_get[i+3]) get->cmd = rxbuf_get[i+3];

				// Check command
				if(rxbuf_get[i+4]) get->status = rxbuf_get[i+4];
				
				// Check command
				if(rxbuf_get[i+5]) get->speed_capit = rxbuf_get[i+5];
				
				// Check command
				if(rxbuf_get[i+6]) get->mode_capitstdy = rxbuf_get[i+6];
				
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->type = PLAY_CAPIT;
			}
			
			// Check for Capit Serok
			else if(rxbuf_get[i+2] == 0x09){
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x09};
				
				// Check command
				if(rxbuf_get[i+3]) get->pos_capit = rxbuf_get[i+3];
				
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->type = PLAY_SEROK;
			}
			
			// Check for Move Jalan Stabil
			else if(rxbuf_get[i+2] == 0x0A){
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x0A};
				
				// Check negative value
				if((rxbuf_get[i+3] & 0x80)) get->pos_x = ((rxbuf_get[i+3] << 8) | rxbuf_get[i+4])-(65536);
				else get->pos_x = (rxbuf_get[i+3] << 8) | rxbuf_get[i+4];
				
				// Check negative value
				if(rxbuf_get[i+5] & 0x80) get->pos_y = ((rxbuf_get[i+5] << 8) | rxbuf_get[i+6])-(65536);
				else get->pos_y = ((rxbuf_get[i+5] << 8) | rxbuf_get[i+6]);
				
				// Check negative value
				if(rxbuf_get[i+7] & 0x80) get->pos_z = ((rxbuf_get[i+7] << 8) | rxbuf_get[i+8])-(65536);
				else get->pos_z = (rxbuf_get[i+7] << 8) | rxbuf_get[i+8];
				
				// Check negative value
				if(rxbuf_get[i+9] & 0x80) get->y_correction = ((rxbuf_get[i+9] << 8) | rxbuf_get[i+10])-(65536);
				else get->y_correction = (rxbuf_get[i+9] << 8) | rxbuf_get[i+10];
				
				// Check negative value
				if(rxbuf_get[i+11] & 0x80) get->speed = (rxbuf_get[i+11])-(256);
				else get->speed =  rxbuf_get[i+11];
				
				// Get Mode
				get->mode_jalan = rxbuf_get[i+12];
				
				// Get Langkah Kaki
				get->walkpoint = rxbuf_get[i+13];
				
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->type = MOVE_JALAN_STABIL;
			}
			
			// Check for Send Req MPU Data
			else if(rxbuf_get[i+2] == 0x0B){
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x0B};
				
				// Check negative value
				if((rxbuf_get[i+3] & 0x80)) get->roll_body = ((rxbuf_get[i+3] << 8) | rxbuf_get[i+4])-(65536);
				else get->roll_body = (rxbuf_get[i+3] << 8) | rxbuf_get[i+4];
				
				// Check negative value
				if(rxbuf_get[i+5] & 0x80) get->pitch_body = ((rxbuf_get[i+5] << 8) | rxbuf_get[i+6])-(65536);
				else get->pitch_body = ((rxbuf_get[i+5] << 8) | rxbuf_get[i+6]);
				
				// Check negative value
				if(rxbuf_get[i+7] & 0x80) get->yaw_body = ((rxbuf_get[i+7] << 8) | rxbuf_get[i+8])-(65536);
				else get->yaw_body = (rxbuf_get[i+7] << 8) | rxbuf_get[i+8];
				
				// Check negative value
				if((rxbuf_get[i+9] & 0x80)) get->roll_body = ((rxbuf_get[i+9] << 8) | rxbuf_get[i+10])-(65536);
				else get->roll_body = (rxbuf_get[i+9] << 8) | rxbuf_get[i+10];
				
				// Check negative value
				if(rxbuf_get[i+11] & 0x80) get->pitch_body = ((rxbuf_get[i+11] << 8) | rxbuf_get[i+12])-(65536);
				else get->pitch_body = ((rxbuf_get[i+11] << 8) | rxbuf_get[i+12]);
				
				// Check negative value
				if(rxbuf_get[i+13] & 0x80) get->yaw_body = ((rxbuf_get[i+13] << 8) | rxbuf_get[i+14])-(65536);
				else get->yaw_body = (rxbuf_get[i+13] << 8) | rxbuf_get[i+14];
				
				
				HAL_UART_Transmit(huart, txbuf, 3, TIMEOUT);
				get->type = GET_REQ_DATA;
			}
		}
	}
	HAL_UART_Receive_DMA(huart, rxbuf_get, 16);
}
