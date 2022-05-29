/**
  ******************************************************************************
  * @file    mobirec_rplidar.c
  * @brief   rplidar minimum driver.
  ******************************************************************************
  */

#include "mobirec_rplidar.h"


mobirec_lidar_StatusTypeDef lidar_check_scan_answer_received(mobirec_lidar_HandleTypeDef* lidar);

mobirec_lidar_StatusTypeDef lidar_check_node(mobirec_lidar_HandleTypeDef* lidar);

mobirec_lidar_StatusTypeDef lidar_read_node(mobirec_lidar_HandleTypeDef* lidar);

mobirec_lidar_StatusTypeDef lidar_start_scan(mobirec_lidar_HandleTypeDef* lidar){
	uint8_t rplidar_stop_msg[] = { SYNC_BYTE, CMD_STOP};
	uint8_t rplidar_scan_msg[] = { SYNC_BYTE, CMD_SCAN};

	HAL_UART_Transmit(lidar->huart, rplidar_stop_msg, 2, 1000);
	HAL_UART_Transmit(lidar->huart, rplidar_scan_msg, 2, 1000);
	uint8_t s= HAL_UART_Receive(lidar->huart, lidar->buff, 7,1000 );
	if(s==HAL_OK){
		return lidar_check_scan_answer_received(lidar);
	}
	return MOBI_TIMEOUT;

}

mobirec_lidar_StatusTypeDef lidar_check_scan_answer_received(mobirec_lidar_HandleTypeDef* lidar){

	if(lidar->buff[0]==0xa5 && lidar->buff[1] == 0x5a &&
	  			  lidar->buff[2]==0x5 && lidar->buff[3] == 0 && lidar->buff[4] == 0 &&
				  lidar->buff[5] ==0x40 && lidar->buff[6] == 0x81){
		return MOBI_OK;
	}
	return MOBI_NO_GOOD_ANS;
}

mobirec_lidar_StatusTypeDef lidar_read_node(mobirec_lidar_HandleTypeDef* lidar){

	uint8_t s = HAL_UART_Receive(lidar->huart,lidar->buff, 5,1000);
	if(s==HAL_OK){
		return lidar_check_node(lidar);

	}
	return MOBI_TIMEOUT;

}

mobirec_lidar_StatusTypeDef lidar_check_node(mobirec_lidar_HandleTypeDef* lidar){
	if((lidar->buff[1]&0x01)&((lidar->buff[0]^(lidar->buff[0]>>1))&0x01)){
		return MOBI_OK;
	}
	return MOBI_NO_GOOD_NODE;

}


mobirec_lidar_StatusTypeDef lidar_get_point(mobirec_lidar_HandleTypeDef* lidar){
	mobirec_lidar_StatusTypeDef s;
	uint8_t wrong_nodes = 0;
	do{
		s=lidar_read_node(lidar);
		wrong_nodes++;
	}while((s==MOBI_NO_GOOD_NODE) && (wrong_nodes<MAX_WRONG_NODES));

	if(s == MOBI_OK){
		lidar->angle = ((lidar->buff[1] >> 1) | (lidar->buff[2] << 7))/64.0; //deg
		lidar->distance = ((lidar->buff[3] | (lidar->buff[4]<<8))/4.0)/1000; // m
		lidar->start_scan_flag = lidar->buff[0] & 0x1;
		lidar->quality = lidar->buff[0]>>2;
	}
	return s;

}
