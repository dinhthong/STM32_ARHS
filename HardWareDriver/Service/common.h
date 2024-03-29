/******************** (C) COPYRIGHT 2015 DUT ********************************
 * 作者    ：胡文博
 * 文件名  ：common.h
 * 描述    ：包含所有驱动程序头文件
 * 日期    ：2015/11/30 12:43:38
 * 联系方式：1461318172（qq）
**********************************************************************************/


#ifndef __common_H
#define __common_H
//#define IMU_lib "IMU_AHRS.h"
// define sensor
//#define HMC5883

#include "IOI2C.h"
#include "delay.h"
#include "MPU6050.h"
//#ifndef HMC5883
#include "HMC5883L.h"
//#endif
#include "ms5611.h"
#include "report.h"
#include "IMU_AHRS.h"
#include "Kalman.h"
//#include "AT45DB.h"


//浮点 联合体
typedef union
{
	float  value;
	unsigned char byte[4];
} f_bytes;

//整数 联合体
typedef union
{
	int16_t  value;
	unsigned char byte[2];
} i_bytes;


#endif

//------------------End of File----------------------------



