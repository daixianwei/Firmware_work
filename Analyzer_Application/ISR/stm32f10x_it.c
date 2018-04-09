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
	se_count++;     								//���Ӽ�������
	if(se_count > 99)   							//1���ӽ���һ��
	{
		if(Display_Time)
		{
			UI_Draw_Status_Bar();
		}
		se_count = 0;								//���¶�ʱ1����
	}

	if(key_fall_flag==1)//�������������¼�
	{
		if(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4))//������������
		{
			if(key_holdon_ms < 2000)
			{
				key_holdon_ms++;
			}
			else 									//�������µ�2000ms���жϳ���ʱ����������ɳ�����־
			{
				key_holdon_ms = 0;
				short_key_flag=0;					//��̰�����־
				long_key_flag = 1;					//��������־��λ
				key_fall_flag = 0;					//�尴�����±�־
			}
		}
		else 										//����̧��
		{
			if(key_holdon_ms > 50)					//����ʱ�����50ms�����ɵ�����־
			{
				key_holdon_ms=0;
				short_key_flag=1;
				long_key_flag =0;
				key_fall_flag=0;
			}
			else  									//��������ʱ��С��50ms������
			{
				key_holdon_ms=0;
				short_key_flag=0;
				long_key_flag=0;
				key_fall_flag=0;
				EXTI_Key_Confirm_Enable();
			}
		}
	}

	Key_Confirm();									//�˺�������������ʱ
}

/******************************************************************************/
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		SignalSample_moveThenSample = 1;
	}

	TIM_ClearITPendingBit(TIM3, ENABLE ); 			//��� TIM3 �����жϱ�־
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
	EXTI_ClearITPendingBit(EXTI_Line1);		 		//���LINE1�ϵ��жϱ�־λ
}

/******************************************************************************/
void EXTI2_IRQHandler(void)
{
	RotaMotorDriver_PositionSensor_Int_Disable();
	EXTI_ClearITPendingBit(EXTI_Line2);			 	//���LINE2�ϵ��жϱ�־λ
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
		if(EXTI_GetITStatus(EXTI_Line4))	 		//���Ұ���
		{
			key_fall_flag = 1;						//ȷ�ϰ���
			EXTI_ClearITPendingBit(EXTI_Line4);		//���LINE4�ϵ��жϱ�־λ
			EXTI_Key_Confirm_Disable();
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line4);			 	//���LINE4�ϵ��жϱ�־λ
}

/******************************************************************************/
void EXTI9_5_IRQHandler(void)
{	 
	if(Exti_lock)
	{
		if(EXTI_GetITStatus(EXTI_Line8))	 		//���Ұ���
		{
			Delay_ms_SW(Time_ms);
			if(EXTI_GetITStatus(EXTI_Line8))	 	//���Ұ���
			{
				Key_Right();
			}
		}

		if(EXTI_GetITStatus(EXTI_Line9))	 		//���Ұ���
		{
			Delay_ms_SW(Time_ms);
			if(EXTI_GetITStatus(EXTI_Line9))	 	//���󰴼�
			{
				Key_Left();
			}
			se_count += Time_ms;
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line8);  			//���LINE8�ϵ��жϱ�־λ
	EXTI_ClearITPendingBit(EXTI_Line9);  			//���LINE9�ϵ��жϱ�־λ
}

/******************************************************************************/
void EXTI15_10_IRQHandler(void)
{
}

/******************************************************************************/
void PVD_IRQHandler(void)
{
}
