/*
 * Interface_Testing.c
 *
 *  Created on: 2018年2月26日
 *      Author: Administrator
 */

/******************************************************************************/
#include "Interface_Testing.h"
#include "RotationMotor.h"
#include "ScanMotorDriver.h"

/******************************************************************************/
extern uint8  Cup_Count;
extern SignalSample_count;
extern uint8 QRCode_existed;
extern uint16 SignalProcess_sampleBuffer[SIGNALSAMPLE_MAX_COUNT];

/******************************************************************************/
block_attr_Testing block_Testing_1 = {
	ENABLE,								/*Interface Testing rect*/
	{
		0,   20,
		128, 140,
		BACKCOLOR_CONTENT_BACK
	},
	DISABLE,							/*Display HZ16X8*/
	{0},
};

/******************************************************************************/
block_attr_Testing block_Testing_2 = {
	ENABLE,								/*Interface Testing rect*/
	{
		14,   63,
		101, 15,
		White
	},

	ENABLE,								/*Display HZ16X8*/
	{
		"testing...",
		26,   80,
		Black,BACKCOLOR_CONTENT_BACK,
		BACKCOLOR_CONTENT_BACK
	},
};

/******************************************************************************/
block_attr_Testing* UI_WindowBlocksAttrArray_Testing[] = {/* Window: Testing entry */
		&block_Testing_1,
		&block_Testing_2,
};

/******************************************************************************/
uint8 Interface_Testing(uint16 KeyCode)
{
	uint16 Start_Postion=0;
	Exti_lock = DISABLE;
	UI_WindowBlocks_Testing = sizeof(UI_WindowBlocksAttrArray_Testing) >> 2;
	UI_Draw_Window_Testing(UI_WindowBlocks_Testing);
	Start_Postion = Get_Start_Postion();
	SystemManage_5V_Enabled();
	RotationMotor_Input_StepDrive(Foreward_Rotation,Start_Postion);
	SystemManage_5V_Disabled();
	if(Cup_Count)
	{
		Acquisition_Signal();
		UI_state = UI_STATE_RESULT;
	}
	else
	{
		UI_state = UI_STATE_INSERT_CUP;
	}
	Delay_ms(500);
	return UI_STATE_RERUN;
}

/******************************************************************************/
void UI_Draw_block_Testing(block_attr_Testing* block);

/******************************************************************************/
void UI_Draw_Window_Testing(uint16 blockNum)
{
	uint8 blockIndex = 0;					/* Draw blocks one by one */
	for (blockIndex = 0; blockIndex < blockNum; blockIndex++)
	{
		UI_Draw_block_Testing(UI_WindowBlocksAttrArray_Testing[blockIndex]);
	}
}

/******************************************************************************/
void UI_Draw_block_Testing(block_attr_Testing* block)
{
	Display_Time = 0;
	if (block->rect_enabled)				/* 1. Draw Rect*/
	{
		Lcd_ColorBox(block->rect_attr.startX, block->rect_attr.startY,
				block->rect_attr.width, block->rect_attr.height,
				block->rect_attr.color);
	}
	if (block->char_enabled)				/* 2. Draw character */
	{

			DisplayDriver_Text16_B(
					block->char_attr.offsetX,block->char_attr.offsetY,
					block->char_attr.color,block->char_attr.faceColor,
					block->char_attr.str);
	}
	Display_Time = 1;
	key_state = DISABLE;
}

/******************************************************************************/
void Acquisition_Signal(void)
{
	uint8 i = 0,j = 0,Step_Count = 25,Step_Start = 14;

	SignalSample_count = 0;
	SystemManage_5V_Enabled();
	/*第二步:旋转360度，环绕杯子采集信号*/
	for(i = 0;i<10;i++)
	{
		if(i >= 9)
		{
			RotationMotor_Input_StepDrive(Foreward_Rotation,53);
		}
		else
		{
			RotationMotor_Input_StepDrive(Foreward_Rotation,51);
		}
		SignalSample_SampleStrip(0);

		for(j = Step_Start;j < Step_Count;j++)		/*每次进度条走十格*/
		{
			Delay_ms(30);
			Display_Time = 0;
			Lcd_ColorBox(j,63,1,15,BACKCOLOR_CONTENT_BAR);
			Display_Time = 1;
		}
		Step_Start = Step_Count;					/*重置进度条开始位置*/
		Step_Count += 10;
	}
	SystemManage_5V_Disabled();
}

/******************************************************************************/
uint16 Get_Start_Postion(void)
{
	uint16 i = 0;
	uint16 Start_Postion=0;
	SignalSample_count = 0;
	/* 第一步:扫描电机转到中间位置 */
	SystemManage_5V_Enabled();
	ScanMotorDriver_Goto_CentrePosition();
	SignalSample_Sample_EnterCriticalArea();
	Delay_ms(200);
	for(i = 0;i < Whole_Circle;i++)
	{
		RotationMotor_Input_StepDrive(Foreward_Rotation,1);
		SignalProcess_sampleBuffer[SignalSample_count++]
									= SignalProcess_Collecting_Data();
	}
	SignalSample_Sample_Timer_Disabled();
	ScanMotorDriver_Goto_BasePosition();

	/*第三步:数据处理得到杯子检测的起始位置*/
	/*1.对数据进行移动平均*/
	SignalSample_Moving_Average_Data(SignalProcess_sampleBuffer,SIGNALSAMPLE_MAX_COUNT,5);

	/*2.有无杯子判断*/
	if((Get_sampleBuffer_Max_Value()) < BOUNDARY_VALUE)
	{
		Start_Postion = NO_CUP;
		return Start_Postion;
	}

	/*3.减去临界值，获得易处理的数据*/
	for(i = 0;i < SIGNALSAMPLE_MAX_COUNT;i++)
	{
		if(SignalProcess_sampleBuffer[i] < BOUNDARY_VALUE)
		{
			SignalProcess_sampleBuffer[i] = 0;
		}
		else
		{
			SignalProcess_sampleBuffer[i] = SignalProcess_sampleBuffer[i] - BOUNDARY_VALUE;
		}
	}
	/*4.得到杯子的起始位置*/
	for(i = 0;i < SIGNALSAMPLE_MAX_COUNT;i++)
	{
		if((SignalProcess_sampleBuffer[i] == 0) && (SignalProcess_sampleBuffer[i+1] > 0) )
		{
			Start_Postion = i;
			break;
		}
	}
	return Start_Postion;
}

/******************************************************************************/
uint16 Get_sampleBuffer_Max_Value(void)
{
	int j=0,max = 0;
	for(j = 0;j < SIGNALSAMPLE_MAX_COUNT;j++)
	{
		if(max <= SignalProcess_sampleBuffer[j])
		{
			max = SignalProcess_sampleBuffer[j];
		}

		if(!SignalProcess_sampleBuffer[j])
		{
				Cup_Count--;
		}
	}
	return max;
}

/******************************************************************************/
void SignalSample_Moving_Average_Data(uint16 *Data,uint16 Length,uint16 Period)
{
	uint16 i=0,j=0;
	uint32 Num = 0;
	/*对数据进行移动平均*/
	for(i = 0;i < Length-Period;i++)
	{
		for(j = 0;j < Period;j++)
		{
			Num += Data[i+j];
		}
		Data[i] = Num/Period;
		Num=0;
	}
}
