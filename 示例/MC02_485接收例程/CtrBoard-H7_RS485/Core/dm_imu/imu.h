#ifndef __IMU_H
#define __IMU_H

#include "stm32h7xx_hal.h"



typedef struct
{
	float accel[3];
	float gyro[3];
	float roll;
	float pitch;
	float yaw;
	float quaternion[4];

}dm_imu_t;


#pragma (1)
typedef struct
{
	uint8_t header;
	uint8_t tag;
	uint8_t slave_id;
	uint8_t reg;
	float data[3];
	uint16_t crc;
	uint8_t tail;

}normal_packet_t;
#pragma ()



#pragma (1)
typedef struct
{
	uint8_t header;
	uint8_t tag;
	uint8_t slave_id;
	uint8_t reg;
	float data[4];
	uint16_t crc;
	uint8_t tail;

}normal_ext_packet_t;
#pragma ()


void imu_data_unpack(uint8_t* pData);
#endif