/* Includes */
#include "dmpu6050.h"
#include "delay.h"
#define  Buf_Size 10
//#include "stm32f10x_i2c.h"
void  MPU6050_newValues(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
int16_t MPU6050_getAvg(int16_t *buff, int size);
/** @defgroup MPU6050_Library
* @{
*/

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050_Initialize(void) 
{
    MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    MPU6050_SetSleepModeStatus(DISABLE); 
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, FALSE otherwise
 */
bool MPU6050_TestConnection(void) 
{
    if(MPU6050_GetDeviceID() == 0x34) //0b110100; 8-bit representation in hex = 0x34
      return TRUE;
    else
      return FALSE;
}
// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x68, 104 dec, 150 oct)
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
uint8_t MPU6050_GetDeviceID(void)
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
    return tmp; 
}
/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
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
 * </pre>
 *
 * @param source New clock source setting
 * @see MPU6050_GetClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void MPU6050_SetClockSource(uint8_t source) 
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see MPU6050_GetFullScaleGyroRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_SetFullScaleGyroRange(uint8_t range) 
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleGyroRange(void) 
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &tmp);
    return tmp;
}
/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleAccelRange(void) 
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &tmp);
    return tmp;
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see MPU6050_GetFullScaleAccelRange()
 */
void MPU6050_SetFullScaleAccelRange(uint8_t range) 
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool MPU6050_GetSleepModeStatus(void) 
{
    uint8_t tmp;
    MPU6050_ReadBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &tmp);
    if(tmp == 0x00)
      return FALSE;
    else
      return TRUE;    
}
/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see MPU6050_GetSleepModeStatus()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_SetSleepModeStatus(FunctionalState NewState) 
{
    MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
}

/** Get raw 6-axis motion sensor readings (accel/temp/gyro).
 * Retrieves all currently available motion sensor values.
 * @param AccelGyro 16-bit signed integer array of length 7
 * @see MPU6050_RA_ACCEL_XOUT_H
 * data 0 -> 2 : Accel
 * data 3      : Temp
 * data 4 -> 6 : Gyro
 */
void MPU6050_GetRawAccelTempGyro(s16* AccelGyro) 
{
    u8 dataR[14],i; 
    MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, dataR, MPU6050_RA_ACCEL_XOUT_H, 14);

    for(i=0;i<7;i++) 
			AccelGyro[i]=(dataR[i*2]<<8)|dataR[i*2+1];   

}
int16_t  MPU6050_FIFO[6][11];
int16_t MPU6050_Lastax, MPU6050_Lastay, MPU6050_Lastaz  //????
, MPU6050_Lastgx, MPU6050_Lastgy, MPU6050_Lastgz;
uint8_t buffer[14], Buf_index = 0;	 
int16_t Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;
void MPU6050_getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
// If DATA READY, then get
//	if(MPU6050_is_DRY())
//	{
		// Read MPU
	//int16_t AccelGyro[7];
	   //u8 dataR[14],i; 
    MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, buffer, MPU6050_RA_ACCEL_XOUT_H, 14);

		MPU6050_Lastax = (((int16_t)buffer[0]) << 8) | buffer[1];
		MPU6050_Lastay = (((int16_t)buffer[2]) << 8) | buffer[3];
		MPU6050_Lastaz = (((int16_t)buffer[4]) << 8) | buffer[5];
		//?????¶?ADC
		MPU6050_Lastgx = (((int16_t)buffer[8]) << 8) | buffer[9];
		MPU6050_Lastgy = (((int16_t)buffer[10]) << 8) | buffer[11];
		MPU6050_Lastgz = (((int16_t)buffer[12]) << 8) | buffer[13];
	
		//IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
		MPU6050_Lastax = (((int16_t)buffer[0]) << 8) | buffer[1];
		MPU6050_Lastay = (((int16_t)buffer[2]) << 8) | buffer[3];
		MPU6050_Lastaz = (((int16_t)buffer[4]) << 8) | buffer[5];
		//?????¶?ADC
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
//	}
//	else
//	{
//		*ax = MPU6050_FIFO[0][10];
//		*ay = MPU6050_FIFO[1][10];
//		*az = MPU6050_FIFO[2][10];
//		*gx = MPU6050_FIFO[3][10] - Gx_offset; 
//		*gy = MPU6050_FIFO[4][10] - Gy_offset; 
//		*gz = MPU6050_FIFO[5][10] - Gz_offset; 
//	}
}


void  MPU6050_newValues(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
//	int16_t new;

//	//Gyro_ADC[0] ?????????PID??????
//	new = (Gyro_ADC[0] * 3 + (gx - Gx_offset)) / 4; // range: +/- 8192; +/- 2000 deg/sec
//	Gyro_ADC[0] = Math_Constrain(new, Gyro_ADC[0] - 800, Gyro_ADC[0] + 800); //??????
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
	Buf_index = (Buf_index + 1) % Buf_Size;//??????????
	// ten values with index: 0->9. the 10th index is the Average.
	MPU6050_FIFO[0][10] = MPU6050_getAvg(MPU6050_FIFO[0], Buf_Size);
	MPU6050_FIFO[1][10] = MPU6050_getAvg(MPU6050_FIFO[1], Buf_Size);
	MPU6050_FIFO[2][10] = MPU6050_getAvg(MPU6050_FIFO[2], Buf_Size);
	MPU6050_FIFO[3][10] = MPU6050_getAvg(MPU6050_FIFO[3], Buf_Size);
	MPU6050_FIFO[4][10] = MPU6050_getAvg(MPU6050_FIFO[4], Buf_Size);
	MPU6050_FIFO[5][10] = MPU6050_getAvg(MPU6050_FIFO[5], Buf_Size);
}

/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) 
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp, mask;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr);   
}
/** write a single bit in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) 
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr); 
}
/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) 
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp, mask;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1); 
    mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) 
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    *data = tmp & (1 << bitNum);
}

/**
* @brief  Initializes the I2C peripheral used to drive the MPU6050
* @param  None
* @return None
*/
void MPU6050_I2C_Init()
{

}

/**
* @brief  Writes one byte to the  MPU6050.
* @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
* @param  writeAddr : address of the register in which the data will be written
* @return None
*/
void MPU6050_I2C_ByteWrite(u8 slaveAddr, u8* pBuffer, u8 writeAddr)
{
//  ENTR_CRT_SECTION();

  /* Send START condition */
  I2C_GenerateSTART(MPU6050_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for write */
  I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send the MPU6050's internal address to write to */
  I2C_SendData(MPU6050_I2C, writeAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(MPU6050_I2C, *pBuffer);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(MPU6050_I2C, ENABLE);

 // EXT_CRT_SECTION();

}

/**
* @brief  Reads a block of data from the MPU6050.
* @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
* @param  readAddr : MPU6050's internal address to read from.
* @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
* @return None
*/

void MPU6050_I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
 // ENTR_CRT_SECTION();

  /* While the bus is busy */
  while(I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(MPU6050_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for write */
  I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter); 

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(MPU6050_I2C, ENABLE);

  /* Send the MPU6050's internal address to write to */
  I2C_SendData(MPU6050_I2C, readAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STRAT condition a second time */
  I2C_GenerateSTART(MPU6050_I2C, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for read */
  I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the MPU6050 */
      *pBuffer = I2C_ReceiveData(MPU6050_I2C);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
//  EXT_CRT_SECTION();

}


void MPU_I2C_Configuration(void)
{
#ifdef FAST_I2C_MODE
 #define I2C_SPEED 400000
 #define I2C_DUTYCYCLE I2C_DutyCycle_16_9  
#else /* STANDARD_I2C_MODE*/
 #define I2C_SPEED 100000
 #define I2C_DUTYCYCLE I2C_DutyCycle_2
#endif /* FAST_I2C_MODE*/
	
  GPIO_InitTypeDef  GPIO_InitStructure;
  I2C_InitTypeDef   I2C_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);	

  /* I2C De-initialize */
  I2C_DeInit(I2C1);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DUTYCYCLE;
  I2C_InitStructure.I2C_OwnAddress1 = 0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1, &I2C_InitStructure);
 /* I2C ENABLE */
  I2C_Cmd(I2C1, ENABLE); 
  /* Enable Interrupt */
}

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

/*
return data with unscaled gyro offset.
*/
void MPU6050_Calculate_Gyro_Offset(float *gx_offset,float *gy_offset,float *gz_offset, uint16_t steps)
{
	uint16_t i;
	int16_t temp[6];
	int32_t	tempgx = 0, tempgy = 0, tempgz = 0;
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
		delay_ms(3);
	}
	// raw to real deg/sec -> *2000/32767 = 1/16.4

	*gx_offset = tempgx /steps; //MPU6050_FIFO[3][10];
	*gy_offset = tempgy / steps; //MPU6050_FIFO[4][10];
	*gz_offset = tempgz / steps; //MPU6050_FIFO[5][10];
}
