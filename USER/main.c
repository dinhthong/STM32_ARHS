/*

PC13 <-> GY-86 DRY
PB6 <-> SCL
PB7 <-> SDA

This program read the RAW values from GY-86.

Using IOI2C driver ( I2C protocol running using GPIO ).
*/
#include "common.h"
#include <math.h>


GPIO_InitTypeDef GPIO_InitStructure;
	
//float LPF(float x,float pre_value, float CUTOFF,float dt);
void get_Baro(void);
void Initial_System_Timer(void)
{
	RCC->APB1ENR |= 0x0008;	
	TIM5->CR1 = 0x0080; //TIMx_ARR buffered  upcounter
	TIM5->CR2 = 0x0000;
	TIM5->CNT = 0x0000;
	TIM5->ARR = 0xFFFFFFFF;
	TIM5->PSC = 84 - 1;	//1us
	TIM5->EGR = 0x0001;
	TIM5->CR1 |= 0x0001; // Enable
}

// LPF
float cut_off=10;
/*Baro */
double referencePressure;
long realPressure;
float fil_comple_alt,fil_lpf_alt ,EstAlt;

double dt;
float absoluteAltitude,realTemperature;
//float rpy_normal[3];
float rpy[3]; //yaw pitch roll
float rate_rpy[3];
int main(void)
{
	uint32_t loop_4ms;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
	delay_init(168);		

	delay_ms(500);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//
	delay_ms(50);
	
		// GPIO LED
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Configure PD12, PD13 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	Initial_System_Timer();  
	
	IMU_init();
	Initialize_Q();	

	begin();
	referencePressure = readPressure(0);

	while(1)
	{
		GPIO_ToggleBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
		
	//	IMU_getRollPitchYaw(rpy);
	//	IMU_getRATEYawPitchRoll(&rate_roll,&rate_pitch,&rate_yaw);
		IMU_getAttitude(rpy,rate_rpy);
		//IMU_getRPY_normal(rpy_normal);

		get_Baro();
		 while((micros() - loop_4ms )< 4000) {};
        dt=(micros()-loop_4ms)*0.000001;
        loop_4ms= micros();
	}
}
void get_Baro(void){
		update_baro(&realTemperature,&realPressure,&absoluteAltitude);
    fil_lpf_alt =LPF(absoluteAltitude,fil_lpf_alt,cut_off,dt);
    fil_comple_alt =fil_comple_alt*(0.977f)+fil_lpf_alt*(0.023f);
	  EstAlt =fil_comple_alt*10;
}