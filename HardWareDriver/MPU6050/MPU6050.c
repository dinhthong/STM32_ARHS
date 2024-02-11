#include "MPU6050.h"
#include "common.h"
#include "Fmath.h"

#define  Buf_Size 10

uint8_t buffer[14], Buf_index = 0;	 //数据读取缓冲区

int16_t  MPU6050_FIFO[6][11];
// automatically embeds into the calculation
int16_t Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;
//int16_t Gyro_ADC[3], ACC_ADC[3];

//Calculate average of 10 consecutive raw IMU values.
int16_t MPU6050_getAvg(int16_t *buff, int size)
{
	int32_t sum = 0;
	int i;
	for(i = 0; i < size; i++)
	{
		sum += buff[i];
	}
	return sum / size;
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功　　能:	    设置  MPU6050 的时钟源
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source)
{
	IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range)
{
	IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}


void MPU6050_setFullScaleAccelRange(uint8_t range)
{
	IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
				enabled =1   
			    enabled =0   
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled)
{
	IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		uint8_t MPU6050_getDeviceID(void)
*功　　能:	    读取  MPU6050 WHO_AM_I 标识	 将返回 0x68
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void)
{
	IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
	return buffer[0];
}

uint8_t MPU6050_testConnection(void)
{
	if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
		return 1;
	else return 0;
}

void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
{
	IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void MPU6050_setI2CBypassEnabled(uint8_t enabled)
{
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
void  MPU6050_newValues(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
//	int16_t new;

//	//Gyro_ADC[0] 数组将会参与PID的运算
//	new = (Gyro_ADC[0] * 3 + (gx - Gx_offset)) / 4; // range: +/- 8192; +/- 2000 deg/sec
//	Gyro_ADC[0] = Math_Constrain(new, Gyro_ADC[0] - 800, Gyro_ADC[0] + 800); //范围限定
//	
//	new = (Gyro_ADC[1] * 3 + (gy - Gy_offset)) / 4;
//	Gyro_ADC[1] = Math_Constrain(new, Gyro_ADC[1] - 800, Gyro_ADC[1] + 800);
//	
//	new = (Gyro_ADC[2] * 3 + (gz - Gz_offset)) / 4;
//	Gyro_ADC[2] = Math_Constrain(new, Gyro_ADC[2] - 800, Gyro_ADC[2] + 800);

//	ACC_ADC[0]  = ax;
//	ACC_ADC[1]  = ay;
//	ACC_ADC[2]  = az;

	MPU6050_FIFO[0][Buf_index] = ax;
	MPU6050_FIFO[1][Buf_index] = ay;
	MPU6050_FIFO[2][Buf_index] = az;
	MPU6050_FIFO[3][Buf_index] = gx;
	MPU6050_FIFO[4][Buf_index] = gy;
	MPU6050_FIFO[5][Buf_index] = gz;
	// Don't need to reset Buff_index
	Buf_index = (Buf_index + 1) % Buf_Size;//循环按列更新
	// ten values with index: 0->9. the 10th index is the Average.
	MPU6050_FIFO[0][10] = MPU6050_getAvg(MPU6050_FIFO[0], Buf_Size);
	MPU6050_FIFO[1][10] = MPU6050_getAvg(MPU6050_FIFO[1], Buf_Size);
	MPU6050_FIFO[2][10] = MPU6050_getAvg(MPU6050_FIFO[2], Buf_Size);
	MPU6050_FIFO[3][10] = MPU6050_getAvg(MPU6050_FIFO[3], Buf_Size);
	MPU6050_FIFO[4][10] = MPU6050_getAvg(MPU6050_FIFO[4], Buf_Size);
	MPU6050_FIFO[5][10] = MPU6050_getAvg(MPU6050_FIFO[5], Buf_Size);
}

void MPU6050_initialize(void)
{
	int16_t temp[6];
	unsigned char i;
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); 
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	// +-2G
	MPU6050_setSleepEnabled(0); //进入工作状态
	MPU6050_setI2CMasterModeEnabled(0);	 
	MPU6050_setI2CBypassEnabled(1);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L

	//配置MPU6050 的中断模式 和中断电平模式
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 0);
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, 0);
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, 1);
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 1);
	// When DATA is READY -> DRY pin <-> PC9 is set
	IICwriteBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1);

	IICwriteBits(devAddr, MPU6050_RA_CONFIG, 7, 8, MPU6050_DLPF); //设置MPU6050低通滤波
	
	// get inital some values into the buffer
	for(i = 0; i < 20; i++) //更新FIFO数组
	{
		delay_us(50);
		MPU6050_getMotion6(&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5]);
	}
}

unsigned char MPU6050_is_DRY(void)
{
	if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_SET)
	{
		return 1;
	}
	else return 0;
}

int16_t MPU6050_Lastax, MPU6050_Lastay, MPU6050_Lastaz  //上次值
, MPU6050_Lastgx, MPU6050_Lastgy, MPU6050_Lastgz;
/*
Return MPU values.

*/
void MPU6050_getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
// If DATA READY, then get
	if(MPU6050_is_DRY())
	{
		// Read MPU
		IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
		MPU6050_Lastax = (((int16_t)buffer[0]) << 8) | buffer[1];
		MPU6050_Lastay = (((int16_t)buffer[2]) << 8) | buffer[3];
		MPU6050_Lastaz = (((int16_t)buffer[4]) << 8) | buffer[5];
		//跳过温度ADC
		MPU6050_Lastgx = (((int16_t)buffer[8]) << 8) | buffer[9];
		MPU6050_Lastgy = (((int16_t)buffer[10]) << 8) | buffer[11];
		MPU6050_Lastgz = (((int16_t)buffer[12]) << 8) | buffer[13];
		MPU6050_newValues(MPU6050_Lastax, MPU6050_Lastay, MPU6050_Lastaz
				  , MPU6050_Lastgx, MPU6050_Lastgy, MPU6050_Lastgz);
		// Average calculated raw values
		*ax  = MPU6050_FIFO[0][10];
		*ay  = MPU6050_FIFO[1][10];
		*az = MPU6050_FIFO[2][10];
		*gx  = MPU6050_FIFO[3][10] - Gx_offset;
		*gy = MPU6050_FIFO[4][10] - Gy_offset;
		*gz = MPU6050_FIFO[5][10] - Gz_offset;
	//else get the old values
	}
	else
	{
		*ax = MPU6050_FIFO[0][10];
		*ay = MPU6050_FIFO[1][10];
		*az = MPU6050_FIFO[2][10];
		*gx = MPU6050_FIFO[3][10] - Gx_offset; 
		*gy = MPU6050_FIFO[4][10] - Gy_offset; 
		*gz = MPU6050_FIFO[5][10] - Gz_offset; 
	}
}


void MPU6050_InitGyro_Offset(void)
{
	unsigned char i;
	int16_t temp[6];
	int32_t	tempgx = 0, tempgy = 0, tempgz = 0;
	int32_t	tempax = 0, tempay = 0, tempaz = 0;
	Gx_offset = 0;
	Gy_offset = 0;
	Gz_offset = 0;
	for(i = 0; i < 50; i++)
	{
		delay_us(100);
		MPU6050_getMotion6(&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5]);
	}
	for(i = 0; i < 100; i++)
	{
		delay_us(200);
		MPU6050_getMotion6(&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5]);
		tempax += temp[0];
		tempay += temp[1];
		tempaz += temp[2];
		tempgx += temp[3];
		tempgy += temp[4];
		tempgz += temp[5];
		//LED_Change();
	}
	Write_Gyro_Offset(tempgx / 100,tempgy / 100,tempgz / 100);
//	Gx_offset = tempgx / 100; //MPU6050_FIFO[3][10];
//	Gy_offset = tempgy / 100; //MPU6050_FIFO[4][10];
//	Gz_offset = tempgz / 100; //MPU6050_FIFO[5][10];
}
/*
return data with unscaled gyro offset.
*/
void MPU6050_Calculate_Gyro_Offset(float *gx_offset,float *gy_offset,float *gz_offset,uint16_t steps)
{
	uint16_t i;
	int16_t temp[6];
	int32_t	tempgx = 0, tempgy = 0, tempgz = 0;
//	int32_t	tempax = 0, tempay = 0, tempaz = 0;
//	float Gx_offset = 0;
//	float Gy_offset = 0;
//	float Gz_offset = 0;
	
//	for(i = 0; i < 50; i++)
//	{
//		delay_us(100);
//		MPU6050_getMotion6(&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5]);
//		//LED_Change();
//	}
	for(i = 0; i < steps; i++)
	{
		
		MPU6050_getMotion6(&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5]);
//		tempax += temp[0];
//		tempay += temp[1];
//		tempaz += temp[2];
		tempgx += temp[3];
		tempgy += temp[4];
		tempgz += temp[5];
		//LED_Change();
		delay_us(500);
	}
				// raw to real deg/sec -> *2000/32767 = 1/16.4

	*gx_offset = tempgx /steps; //MPU6050_FIFO[3][10];
	*gy_offset = tempgy / steps; //MPU6050_FIFO[4][10];
	*gz_offset = tempgz / steps; //MPU6050_FIFO[5][10];
}
// I added this function to my program but it fail to calculate correct angular velocity
void Write_Gyro_Offset(int16_t GX_offset,int16_t GY_offset,int16_t GZ_offset)
{
	Gx_offset=GX_offset;
	Gy_offset=GY_offset;
	Gz_offset=GZ_offset;
}
