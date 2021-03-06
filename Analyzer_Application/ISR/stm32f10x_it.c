 /*******************************************************************************
 * File name: stm32f10x_it.c
 * Author: Hanson Liu
 * Mail: han_liu_zju@sina.com
 * Date: 2014.12 ~ Now
 ******************************************************************************/
#include "main.h"
#include "stm32f10x_it.h"
#include "DisplayDriver.h"
#include "Interface_main.h"

extern uint8 SignalSample_moveThenSample;
uint16 time_ms = 0;
extern uint8 Power_Open;
extern uint8 Display_Time;
/******************************************************************************/
void Delay_ms_SW(__IO uint32 nCount)
{
#define SW_72MHZ_1MS_COUNT (0x27FF)
	uint32 subCount;
	for(; nCount != 0; nCount--)
	{
		subCount = SW_72MHZ_1MS_COUNT;
		for(; subCount != 0; subCount--);
	}
}
/********************************************************************************/
void NMI_Handler(void)
{
}

/******************************************************************************/
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/******************************************************************************/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/******************************************************************************/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/******************************************************************************/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/******************************************************************************/
void SVC_Handler(void)
{
}

/******************************************************************************/
void DebugMon_Handler(void)
{
}

/******************************************************************************/
void PendSV_Handler(void)
{
}

/******************************************************************************/
void SysTick_Handler(void)
{
	TimingDelay_Decrement();						/* Decrease TimingDelay */
	se_count++;     								//秒钟计数自增
	if(se_count > 99)   							//1秒钟进入一次
	{
		if(Display_Time)
		{
			UI_Draw_Status_Bar();
		}
		se_count = 0;								//重新定时1秒钟
	}

	if(key_fall_flag==1)//发生按键按下事件
	{
		if(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4))//按键持续按下
		{
			if(key_holdon_ms < 2000)
			{
				key_holdon_ms++;
			}
			else 									//按键按下到2000ms就判断长按时间成立，生成长按标志
			{
				key_holdon_ms = 0;
				short_key_flag=0;					//清短按键标志
				long_key_flag = 1;					//长按键标志置位
				key_fall_flag = 0;					//清按键按下标志
			}
		}
		else 										//按键抬起
		{
			if(key_holdon_ms > 50)					//按下时间大于50ms，生成单击标志
			{
				key_holdon_ms=0;
				short_key_flag=1;
				long_key_flag =0;
				key_fall_flag=0;
			}
			else  									//按键持续时间小于50ms，忽略
			{
				key_holdon_ms=0;
				short_key_flag=0;
				long_key_flag=0;
				key_fall_flag=0;
				EXTI_Key_Confirm_Enable();
			}
		}
	}

	Key_Confirm();									//此函数决不能有延时
}

/******************************************************************************/
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		SignalSample_moveThenSample = 1;
	}

	TIM_ClearITPendingBit(TIM3, ENABLE ); 			//清除 TIM3 更新中断标志
}

/******************************************************************************/
void TIM4_IRQHandler(void)
{
}

/******************************************************************************/
void TIM7_IRQHandler(void)
{

}

/******************************************************************************/
void EXTI0_IRQHandler(void)
{

}

/******************************************************************************/
void EXTI1_IRQHandler(void)
{
	ScanMotorDriver_InBasePosition = 1;
	ScanMotorDriver_PositionSensor_Int_Disable();
	EXTI_ClearITPendingBit(EXTI_Line1);		 		//清除LINE1上的中断标志位
}

/******************************************************************************/
void EXTI2_IRQHandler(void)
{
	RotaMotorDriver_PositionSensor_Int_Disable();
	EXTI_ClearITPendingBit(EXTI_Line2);			 	//清除LINE2上的中断标志位
}

/******************************************************************************/
void EXTI3_IRQHandler(void)
{
}

/******************************************************************************/
void EXTI4_IRQHandler(void)
{
	if(Exti_lock)
	{
		if(EXTI_GetITStatus(EXTI_Line4))	 		//向右按键
		{
			key_fall_flag = 1;						//确认按键
			EXTI_ClearITPendingBit(EXTI_Line4);		//清除LINE4上的中断标志位
			EXTI_Key_Confirm_Disable();
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line4);			 	//清除LINE4上的中断标志位
}

/******************************************************************************/
void EXTI9_5_IRQHandler(void)
{	 
	if(Exti_lock)
	{
		if(EXTI_GetITStatus(EXTI_Line8))	 		//向右按键
		{
			Delay_ms_SW(Time_ms);
			if(EXTI_GetITStatus(EXTI_Line8))	 	//向右按键
			{
				Key_Right();
			}
		}

		if(EXTI_GetITStatus(EXTI_Line9))	 		//向右按键
		{
			Delay_ms_SW(Time_ms);
			if(EXTI_GetITStatus(EXTI_Line9))	 	//向左按键
			{
				Key_Left();
			}
			se_count += Time_ms;
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line8);  			//清除LINE8上的中断标志位
	EXTI_ClearITPendingBit(EXTI_Line9);  			//清除LINE9上的中断标志位
}

/******************************************************************************/
void EXTI15_10_IRQHandler(void)
{
}

/******************************************************************************/
void PVD_IRQHandler(void)
{
}
