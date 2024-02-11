#include "IMU_AHRS.h"

#define DATA_SIZE 200
float safe_asin(float v);

volatile float exInt, eyInt, ezInt;  
volatile float integralFBx, integralFBy, integralFBz;
volatile float qa0, qa1, qa2, qa3;
float q_ahrs[4];
volatile float integralFBhand, handdiff;
volatile double halfT ,elapsedT;
volatile uint32_t lastUpdate, now;
volatile float acc_vector = 0;  
volatile float IMU_Pitch, IMU_Roll, IMU_Yaw;


/**************************??????********************************************

*******************************************************************************/
float invSqrt(float x)
{

	volatile float halfx = 0.5f * x;
	volatile float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/*********************************************************************

*******************************************************************************/
float offset_gx,offset_gy,offset_gz;
#include "dmpu6050.h"
void IMU_init(void)
{
		#if 0
		/*
			IO I2C init??
		*/
    GPIO_InitTypeDef GPIO_InitStructure;

	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
		#ifndef HMC5883
		HMC5883L_SetUp();
		#endif
		#endif
		MPU_I2C_Configuration();  
		MPU6050_Initialize();
		delay_ms(20);
		Initialize_Q();
}

/**************************??????********************************************

*******************************************************************************/
#define new_weight 0.4f
#define old_weight 0.6f

static int16_t IMU_unscaled[6];
void IMU_getValues(float *values)
{
	int16_t accgyroval[6];
	static  volatile float lastacc[3] = {0, 0, 0};
	uint8_t i;
	MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
	// added to watch the 'raw' values
	for (i=0;i<6;i++)
	{
	IMU_unscaled[i]=accgyroval[i];
	}
	
	for(i = 0; i < 6; i++)
	{
		if(i < 3)
		{
			values[i] = (float) accgyroval[i] * new_weight + lastacc[i] * old_weight ;
			lastacc[i] = values[i];
		}
		else
		{
			values[i] = ((float) accgyroval[i]) / 16.4f; 
		}
	}

}

/***************************************************************************

*******************************************************************************/
float acc_x,acc_y,acc_z;
float IMU_data[9];
float alpha_rate_rpy = 0.63f;
extern float roll_offset, pitch_offset;
void IMU_getAttitude(float *RPY,float *rate_RPY)
{
	float temp_rpy[3];
		float gx,gy,gz;
	IMU_getValues(IMU_data);
	now = micros(); 
	if(now < lastUpdate) 
	{
		elapsedT =  (float)(now + (0xffffffff - lastUpdate));
	}
	else
	{
		elapsedT =  (now - lastUpdate);
	}
		elapsedT = elapsedT * 0.000001f;
		halfT = elapsedT / 2.0f;
		lastUpdate = now;
    gx=IMU_unscaled[3] - offset_gx;
    gy=IMU_unscaled[4] - offset_gy;
    gz=IMU_unscaled[5] - offset_gz;
		rate_RPY[0] = rate_RPY[0] * alpha_rate_rpy + gx * (1.0f - alpha_rate_rpy)/16.4f;
	  rate_RPY[1] = rate_RPY[1] * alpha_rate_rpy + gy * (1.0f - alpha_rate_rpy)/16.4f;
	  rate_RPY[2] = rate_RPY[2] * alpha_rate_rpy + gz * (1.0f - alpha_rate_rpy)/16.4f;
    IMU_getQ(q_ahrs, IMU_data); 
	  
	  IMU_getRollPitchYaw(temp_rpy, q_ahrs);
	  RPY[0] = temp_rpy[0] - roll_offset;
	  RPY[1] = (-temp_rpy[1]) - pitch_offset;
	  RPY[2] = -temp_rpy[2];
}

float LPF(float x, float pre_value, float CUTOFF,float dt)
{
    float RC, alpha, y;
    RC = 1.0f/(CUTOFF*2*3.1416f);
    alpha = dt/(RC+dt);
    y = pre_value + alpha * ( x - pre_value );
    return y;
}

float safe_asin(float v)
{
	if (isnan(v))
	{
		return 0.0f;
	}
	if (v >= 1.0f)
	{
		return M_PI / 2;
	}
	if (v <= -1.0f)
	{
		return -M_PI / 2;
	}
	return asin(v);
}

void Initialize_Q(void)
{
float acc[9];
	int i;
	volatile float temp, roll, pitch, yaw, yh, xh;
	volatile float  acc_X, acc_Y, acc_Z, acc_MX, acc_MY, acc_MZ; 
	q_ahrs[0] = 1.0f; 
	q_ahrs[1] = 0.0f;
	q_ahrs[2] = 0.0f;
	q_ahrs[3] = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
	integralFBx = 0.0;
	integralFBy = 0.0;
	integralFBz	= 0.0;
	
	for(i = 0; i < DATA_SIZE; i++) {

		IMU_getValues(acc);
		acc_X += acc[0];
		acc_Y += acc[1];
		acc_Z += acc[2];
		acc_MX += acc[6];
		acc_MY += acc[7];
		acc_MZ += acc[8];
	}
	acc_X /= DATA_SIZE;
	acc_Y /= DATA_SIZE;
	acc_Z /= DATA_SIZE;
	acc_MX /= DATA_SIZE;
	acc_MY /= DATA_SIZE;
	acc_MZ /= DATA_SIZE;

	temp = acc_X * invSqrt((acc_Y * acc_Y + acc_Z * acc_Z));
	pitch = atan(temp) * 57.3;

	temp = acc_Y * invSqrt((acc_X * acc_X + acc_Z * acc_Z));
	roll = atan(temp) * 57.3;

	yh = acc_MY * cos(roll) + acc_MZ * sin(roll);
	xh = acc_MX * cos(pitch) + acc_MY * sin(roll) * sin(pitch) - acc_MZ * cos(roll) * sin(pitch);
	yaw = atan2(yh, xh);
	q_ahrs[0] = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
	q_ahrs[1] = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
	q_ahrs[2] = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
  q_ahrs[3] = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
}

/**
 * @brief FreeAHRS sensor fusion algorithm. which only requires only data from MPU6050 and HMC5883 - 9 DOF
 * @param
	 angular velocity
	 accelerometer
	 magnetometer
 * @return
	 fused quaternion values
 * @author thongnd 
 * @update_date: 2024.02.11
 */

#define Kp 3.0f    
#define Ki 0.03f  
void IMU_AHRSupdate(float* q_update, volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az, volatile float mx, volatile float my, volatile float mz)
{
	volatile  float norm;
	volatile float vx, vy, vz, wx, wy, wz;
	volatile float ex, ey, ez;
	volatile float temp0, temp1, temp2, temp3;
	volatile float temp;

	float q0q0 = q_update[0] * q_update[0];
	float q0q1 = q_update[0] * q_update[1];
	float q0q2 = q_update[0] * q_update[2];
	float q0q3 = q_update[0] * q_update[3];
	float q1q1 = q_update[1] * q_update[1];
	float q1q2 = q_update[1] * q_update[2];
	float q1q3 = q_update[1] * q_update[3];
	float q2q2 = q_update[2] * q_update[2];
	float q2q3 = q_update[2] * q_update[3];
	float q3q3 = q_update[3] * q_update[3];

	temp = sqrt(ax * ax + ay * ay + az * az);
	temp = (temp / 16384.0f) * 9.8f;  
	acc_vector = acc_vector +   
		     (halfT * 2.0f / (7.9577e-3f + halfT * 2.0f)) * (temp - acc_vector);

	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;
		ezInt = ezInt + ez * Ki * halfT;

		// 
		gx = gx + (Kp * ex + exInt);
		gy = gy + (Kp * ey + eyInt);
		gz = gz + (Kp * ez + ezInt);

	}

	temp0 = q_update[0] + (-q_update[1] * gx - q_update[2] * gy - q_update[3] * gz) * halfT;
	temp1 = q_update[1] + (q_update[0] * gx + q_update[2] * gz - q_update[3] * gy) * halfT;
	temp2 = q_update[2] + (q_update[0] * gy - q_update[1] * gz + q_update[3] * gx) * halfT;
	temp3 = q_update[3] + (q_update[0] * gz + q_update[1] * gy - q_update[2] * gx) * halfT;


	norm = invSqrt(temp0 * temp0 + temp1 * temp1 + temp2 * temp2 + temp3 * temp3);
	q_update[0] = temp0 * norm;
	q_update[1] = temp1 * norm;
	q_update[2] = temp2 * norm;
	q_update[3] = temp3 * norm;
}

/**
 * @brief FreeAHRS sensor fusion algorithm. which only requires only data from MPU6050 - 6 DOF
 * @param

 * @return

 * @author thongnd 
 * @update_date: 2024.02.11
 */
#define twoKpDef  (1.0f ) 
#define twoKiDef  (0.2f) 

void FreeIMU_AHRSupdate(float* q_update, volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az)
{
	volatile float norm;
	
	volatile float vx, vy, vz;
	volatile float ex, ey, ez;
	volatile float temp0, temp1, temp2, temp3;
	
	volatile float q0q0 = q_update[0] * q_update[0];
	volatile float q0q1 = q_update[0] * q_update[1];
	volatile float q0q2 = q_update[0] * q_update[2];
	volatile float q0q3 = q_update[0] * q_update[3];
	volatile float q1q1 = q_update[1] * q_update[1];
	volatile float q1q2 = q_update[1] * q_update[2];
	volatile float q1q3 = q_update[1] * q_update[3];
	volatile float q2q2 = q_update[2] * q_update[2];
	volatile float q2q3 = q_update[2] * q_update[3];
	volatile float q3q3 = q_update[3] * q_update[3];

	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	ex = (ay * vz - az * vy) ;
	ey = (az * vx - ax * vz) ;
	ez = (ax * vy - ay * vx) ;

	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{

		integralFBx +=  ex * twoKiDef * halfT;
		integralFBy +=  ey * twoKiDef * halfT;
		integralFBz +=  ez * twoKiDef * halfT;

		gx = gx + twoKpDef * ex + integralFBx;
		gy = gy + twoKpDef * ey + integralFBy;
		gz = gz + twoKpDef * ez + integralFBz;

	}
	temp0 = q_update[0] + (double)(-q_update[1] * gx - q_update[2] * gy - q_update[3] * gz) * halfT;
	temp1 = q_update[1] + (double)(q_update[0] * gx + q_update[2] * gz - q_update[3] * gy) * halfT;
	temp2 = q_update[2] + (double)(q_update[0] * gy - q_update[1] * gz + q_update[3] * gx) * halfT;
	temp3 = q_update[3] + (double)(q_update[0] * gz + q_update[1] * gy - q_update[2] * gx) * halfT;

	norm = invSqrt(temp0 * temp0 + temp1 * temp1 + temp2 * temp2 + temp3 * temp3);
	q_update[0] = temp0 * norm;
	q_update[1] = temp1 * norm;
	q_update[2] = temp2 * norm;
	q_update[3] = temp3 * norm;
}

/****************************************************************************

*******************************************************************************/
void IMU_getQ(float *q,volatile float IMU_values[9])
{

#ifndef HMC5883
	IMU_AHRSupdate(q, IMU_values[3] * M_PI / 180, IMU_values[4] * M_PI / 180, IMU_values[5] * M_PI / 180,
		       IMU_values[0], IMU_values[1], IMU_values[2], IMU_values[6], IMU_values[7], IMU_values[8]);
#else
	FreeIMU_AHRSupdate(q, IMU_values[3] * M_PI / 180, IMU_values[4] * M_PI / 180, IMU_values[5] * M_PI / 180,
			   IMU_values[0], IMU_values[1], IMU_values[2]);
#endif

}
void IMU_getRollPitchYaw(float *angles, float *qa)
{

	IMU_Roll = angles[0] = (atan2(2.0f * (qa[0] * qa[1] + qa[2] * qa[3]),
				      1 - 2.0f * (qa[1] * qa[1] + qa[2] * qa[2]))) * 180 / M_PI;
	
	IMU_Pitch = angles[1] = -safe_asin(2.0f * (qa[0] * qa[2] - qa[3] * qa[1])) * 180 / M_PI;
	IMU_Yaw = angles[2] = -atan2(2 * qa[1] * qa[2] + 2 * qa[0] * qa[3], -2 * qa[2] * qa[2] - 2 * qa[3] * qa[3] + 1) * 180 / M_PI; // yaw
}
