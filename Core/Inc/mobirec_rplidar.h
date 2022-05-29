
#include "stm32f4xx_hal.h"

#define MAX_WRONG_NODES 20

#define CMD_SCAN 0x20
#define CMD_STOP 0x25

#define SYNC_BYTE 0xa5

/*defining lidar statuses*/
typedef enum
{
  MOBI_OK       = 0x00U,
  MOBI_TIMEOUT    = 0x01U,
  MOBI_NO_GOOD_ANS     = 0x02U,
  MOBI_NO_GOOD_NODE  = 0x03U
} mobirec_lidar_StatusTypeDef;

typedef struct __mobirec_lidar_Handle
{	/*utilities */
	UART_HandleTypeDef* huart;
	uint8_t buff[20]; //temp buffer used to read data
	/* data  */
	uint8_t start_scan_flag; /* 1 if new 360° scan started*/
	uint8_t quality; /* related to the strength of the received laser pulse */
	float angle; /* angular displacement of the point */
	float distance; /* distance from the lidar to the point */
} mobirec_lidar_HandleTypeDef;



mobirec_lidar_StatusTypeDef lidar_start_scan(mobirec_lidar_HandleTypeDef* lidar);

mobirec_lidar_StatusTypeDef lidar_get_point(mobirec_lidar_HandleTypeDef* lidar);
