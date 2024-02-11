#ifndef __IMU_H
#define __IMU_H


#include <math.h>
#include "common.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/*
imu_quaternion
imu_brokking
*/
#define imu_quaternion

#define M_PI  (float)3.1415926535
#define micros() TIM5->CNT
extern float IMU_values[9];
extern int16_t IMU_unscaled[6];
extern volatile float IMU_Pitch, IMU_Roll, IMU_Yaw, ACC_Pitch, ACC_Roll;	 //µ¥Î» ¶È

extern volatile float IMU_GYROx, IMU_GYROy, IMU_GYROz;	
extern volatile float acc_vector;  
//Mini IMU AHRS 
void IMU_getValues(float *values);
void IMU_init(void);
//void Initialize_Q(void);
//void IMU_getRollPitchYaw(float *angles);
float LPF(float x,float pre_value, float CUTOFF,float dt);
void IMU_getAttitude(float *RPY,float *rate_RPY);
void Initialize_Q(void);
//void Initialize_Q(void);
void IMU_getQ(float *q,volatile float IMU_values[9]);
void IMU_getRollPitchYaw(float *angles, float *qa);
void IMU_AHRSupdate(float* q_update, volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az, volatile float mx, volatile float my, volatile float mz);
void FreeIMU_AHRSupdate(float* q_update, volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az);
#endif

