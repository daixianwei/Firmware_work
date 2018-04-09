/*
 * motor.c
 *
 *  Created on: 2018年1月26日
 *      Author: Administrator
 */

/******************************************************************************/
#include "stm32f10x.h"
#include "RotationMotor.h"
#include "RotationMotorDriver_PositionSensor.h"

/******************************************************************************/
#define Time_10us  (180)
#define Time_Ms    (100)

/******************电机正反转控制*************************************************/
u32 Step_count = RESET;
uint8 Num_Down = 1;
u16 Step1[8] = {0x10,0x30,0x20,0x60,0x40,0xc0,0x80,0x90};   //电机正转
u16 Step2[8] = {0x90,0x80,0xc0,0x40,0x60,0x20,0x30,0x10};   //电机反转

/******************************************************************************/
void RotationMotor_Init(void)
{
	RotaMotorDriver_GPIO_Init();

	RotaMotorDriver_PositionSensor_Init();
}

/******************************************************************************/
void RotaMotorDriver_GPIO_Init(void)
{
   GPIO_InitTypeDef GPIO_InitStruct;

   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStruct.GPIO_Pin  =  MotorDriver_BUTTER_PIN;
   GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(MotorDriver_BUTTER_PORT,&GPIO_InitStruct);

   /* RotationMotor Pin Initialize PE 11 12 13 14 */
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStruct.GPIO_Pin  = RotaMotorDriver_IN1_PIN |
		   RotaMotorDriver_IN2_PIN |RotaMotorDriver_IN3_PIN |
		   RotaMotorDriver_IN4_PIN|RotaMotorDriver_EN_PIN;
   GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOE,&GPIO_InitStruct);

   GPIO_SetBits(RotaMotorDriver_EN_PORT, RotaMotorDriver_EN_PIN);
   GPIO_SetBits(MotorDriver_BUTTER_PORT, MotorDriver_BUTTER_PIN);
}

/******************************************************************************/
void Delay_us(int nCount)
{
	#define SW_72MHZ_1US_COUNT (102)
	int subCount;
	for(; nCount != 0; nCount--)
	{
		subCount = SW_72MHZ_1US_COUNT;
		for(; subCount != 0; subCount--);
	}
}

/******************************************************************************/
void RotationMotor_SelfCheck_StepDrive(void)
{
	if(Num_Down)
	{
		SystemManage_5V_Enabled();
		while(!ROTA_POSSEN_INT_STATE())
		{
			RotationMotor_StepDrive_Min(Foreward_Rotation);
		}
		RotationMotor_Input_StepDrive(Foreward_Rotation,Whole_Circle);
		Num_Down = 0;
		SystemManage_5V_Disabled();
	}
}

/******************************************************************************/
void RotationMotor_Input_StepDrive(uint8 Rotation_Direction,uint16 Step)
{
	for(Step_count = Zero;Step_count < (Step);Step_count++)
	{
		RotationMotor_StepDrive_Min(Rotation_Direction);
	}
}

/******************************************************************************/
static void RotationMotor_PIN(uint8 Rotation_Direction,u8 MUTU)
{
	switch(Rotation_Direction)
	{
		case Reversal_Rotation:
			GPIO_WriteBit(GPIOE,RotaMotorDriver_IN1_PIN,(Step2[MUTU]&0x10));   //电机反转
			GPIO_WriteBit(GPIOE,RotaMotorDriver_IN2_PIN,(Step2[MUTU]&0x20));
			GPIO_WriteBit(GPIOE,RotaMotorDriver_IN3_PIN,(Step2[MUTU]&0x40));
			GPIO_WriteBit(GPIOE,RotaMotorDriver_IN4_PIN,(Step2[MUTU]&0x80));
		break;

		case Foreward_Rotation:
			GPIO_WriteBit(GPIOE,RotaMotorDriver_IN1_PIN,(Step1[MUTU]&0x10));   //电机正转
			GPIO_WriteBit(GPIOE,RotaMotorDriver_IN2_PIN,(Step1[MUTU]&0x20));
			GPIO_WriteBit(GPIOE,RotaMotorDriver_IN3_PIN,(Step1[MUTU]&0x40));
			GPIO_WriteBit(GPIOE,RotaMotorDriver_IN4_PIN,(Step1[MUTU]&0x80));
		break;

		default:
		break;
	}
}

/******************************************************************************/
void RotationMotor_StepDrive_Min(uint8 Rotation_Direction)
{
	uint8 Mutually;
	for(Mutually = Zero;Mutually< 8;Mutually++)
	{
		RotationMotor_PIN(Rotation_Direction,Mutually);
		Delay_us(Time_10us);
	}
}
