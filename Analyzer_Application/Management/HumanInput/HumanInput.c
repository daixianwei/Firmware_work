/*
 * HumanInput.c
 *
 *  Created on: 2018年1月25日
 *      Author: Administrator
 ******************************************************************************/
#include "HumanInput.h"
#include "DisplayDriver.h"
#include "Font.h"
#include "RotationMotor.h"

/******************************************************************************/
extern uint8 Power_Open;
extern uint8 key_fall_flag,short_key_flag,doubleClick,long_key_flag;

/*******************************************************************************
	函数名：HumanInput_Init
	输  入: 无
	输  出: 无
	功能说明：使能按键相应GPIO管脚
	初始化串口硬件设备，未启用中断。
*******************************************************************************/
void HumanInput_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		//设置成上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);				//初始化GPIOE2,3,4

	//初始化 WK_UP-->GPIOA.0	  下拉输入
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		//PA0设置成输入，默认下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);				//初始化GPIOA.0
}

/*******************************************************************************
	函数名：EXTIX_Init
	输  入: 无
	输  出: 无
	功能说明：使能按键相应外部中断    五向
	初始化串口硬件设备，未启用中断。
*******************************************************************************/
void EXTIX_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//GPIOB.4	  中断线以及中断初始化配置 上升沿触发
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource4);

	EXTI_InitStructure.EXTI_Line = EXTI_Line4;  				//中间确认按钮
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	//GPIOD.8 中断线以及中断初始化配置   上升沿触发
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource8);

	EXTI_InitStructure.EXTI_Line = EXTI_Line8;					//向右按钮
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	//GPIOD.9	  中断线以及中断初始化配置  上升沿触发
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource9);

	EXTI_InitStructure.EXTI_Line = EXTI_Line9;					//向左按钮
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;			//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;			//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能外部中断通道

	NVIC_Init(&NVIC_InitStructure);
}

/******************************************************************************/
void EXTI_Key_Confirm_Disable(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	//GPIOB.4	  中断线以及中断初始化配置 上升沿触发
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource4);

	EXTI_InitStructure.EXTI_Line = EXTI_Line4;  				//中间确认按钮
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/******************************************************************************/
void EXTI_Key_Confirm_Enable(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//GPIOB.4	  中断线以及中断初始化配置 上升沿触发
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource4);

	EXTI_InitStructure.EXTI_Line = EXTI_Line4;  				//中间确认按钮
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;			//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
	函数名：KeyX_Work
	输  入: 无
	输  出: 无
	功能说明：相应按键功能
	初始化串口硬件设备，未启用中断。
*******************************************************************************/
void Key_Confirm(void)
{
	if(Exti_lock)
	{
		if(short_key_flag)
		{
			key_state_confirm = ENABLE;
			key_state = ENABLE;
			short_key_flag = 0;
			EXTI_Key_Confirm_Enable();
		}

		if(UI_state != UI_STATE_TESTING && Power_Open && long_key_flag)
		{
			SystemManage_CheckPowerOff();
		}

//		if(UI_state == UI_STATE_RESULT && UI_state == UI_STATE_RESULT_2 &&
//				long_key_flag)
//		{
//			long_key_flag = 0;
		EXTI_Key_Confirm_Enable();
//		}
	}
}

/******************************************************************************/
void Key_Right(void)
{
	switch(Interface_Key)
	{
		case 0:
			key_state = ENABLE;
			key_state_confirm = 0;
			if(Key_control < 4)
			{
				Key_control += 1;
			}
			else
			{
				Key_control = 1;
			}
		break;

		case 1:
		break;

		case 2:
			key_state = ENABLE;
			key_state_confirm = 0;
			Key_control = 2;
		break;

		default:
		break;
	}
}

/******************************************************************************/
void Key_Left(void)
{
	switch(Interface_Key)
	{
		case 0:
			key_state = 1;
			key_state_confirm = 0;
			if(Key_control > 1)
			{
				Key_control -= 1;
			}
			else
			{
				Key_control = 4;
			}
		break;

		case 1:
		break;

		case 2:
			key_state = ENABLE;
			key_state_confirm = 0;
			Key_control = 1;
		break;

		case 3:
			key_state = ENABLE;
			key_state_confirm = 0;
			Key_control = 1;
		break;

		default:
		break;
	}
}

/******************************************************************************/
void SystemManage_CheckPowerOff(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	GPIO_ResetBits(GPIOD, GPIO_Pin_2);
}
