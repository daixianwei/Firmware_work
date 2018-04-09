/*
 * ScanScanMotorDriver.c
 *
 *  Created on: 2018��3��2��
 *      Author: Administrator
 */

/******************************************************************************/
#include "ScanMotorDriver.h"
#include "ScanMotorDriver_PositionSensor.h"

/******************************************************************************/
uint8 Check_Count = 1;
unsigned int CCR2_Val = 45000;
uint8 ScanMotorDriver_InBasePosition = 0;

/******************************************************************************/
void Delay_SW(__IO uint32 nCount)
{
	for(; nCount != 0; nCount--);
}

/******************************************************************************/
void ScanMotorDriver_NVIC_Configuration() {
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/******************************************************************************/
void ScanMotorDriver_A1(void) {
	GPIO_ResetBits(ScanMotorDriver_IN4_PORT, ScanMotorDriver_IN4_PIN);
	GPIO_SetBits(ScanMotorDriver_IN3_PORT, ScanMotorDriver_IN3_PIN);
	GPIO_SetBits(ScanMotorDriver_IN2_PORT, ScanMotorDriver_IN2_PIN);
	GPIO_ResetBits(ScanMotorDriver_IN1_PORT, ScanMotorDriver_IN1_PIN);
}

/******************************************************************************/
void ScanMotorDriver_A2(void) {
	GPIO_ResetBits(ScanMotorDriver_IN4_PORT, ScanMotorDriver_IN4_PIN);
	GPIO_SetBits(ScanMotorDriver_IN3_PORT, ScanMotorDriver_IN3_PIN);
	GPIO_ResetBits(ScanMotorDriver_IN2_PORT, ScanMotorDriver_IN2_PIN);
	GPIO_SetBits(ScanMotorDriver_IN1_PORT, ScanMotorDriver_IN1_PIN);
}

/******************************************************************************/
void ScanMotorDriver_B1(void) {
	GPIO_SetBits(ScanMotorDriver_IN4_PORT, ScanMotorDriver_IN4_PIN);
	GPIO_ResetBits(ScanMotorDriver_IN3_PORT, ScanMotorDriver_IN3_PIN);
	GPIO_ResetBits(ScanMotorDriver_IN2_PORT, ScanMotorDriver_IN2_PIN);
	GPIO_SetBits(ScanMotorDriver_IN1_PORT, ScanMotorDriver_IN1_PIN);
}

/******************************************************************************/
void ScanMotorDriver_B2(void) {
	GPIO_SetBits(ScanMotorDriver_IN4_PORT, ScanMotorDriver_IN4_PIN);
	GPIO_ResetBits(ScanMotorDriver_IN3_PORT, ScanMotorDriver_IN3_PIN);
	GPIO_SetBits(ScanMotorDriver_IN2_PORT, ScanMotorDriver_IN2_PIN);
	GPIO_ResetBits(ScanMotorDriver_IN1_PORT, ScanMotorDriver_IN1_PIN);
}

/******************************************************************************/
void ScanMotorDriver_Control(uint8 enabled) {
	if (enabled)
		GPIO_SetBits(ScanMotorDriver_EN_PORT, ScanMotorDriver_EN_PIN);
	else
		GPIO_ResetBits(ScanMotorDriver_EN_PORT, ScanMotorDriver_EN_PIN);
}

/******************************************************************************/
void ScanMotorDriver_MoveOneStep(uint8 dir) {
#define MOTOR_QUICK_DELAY   0X4000
#define MOTOR_SLOW_DELAY 	0X4000
uint8 index = 0;

	while (index < 4) {
		if (dir == ScanMotorDriver_DIR_IN) {
			/* Direction: IN */
			switch (index) {
			case 0:
				ScanMotorDriver_A2();
				index++;
				break;
			case 1:
				ScanMotorDriver_A1();
				index++;
				break;
			case 2:
				ScanMotorDriver_B2();
				index++;
				break;
			case 3:
				ScanMotorDriver_B1();
				index++;
				break;
			}
		} else {
			/* Direction: OUT */
			switch (index) {
			case 0:
				ScanMotorDriver_A1();
				index++;
				break;
			case 1:
				ScanMotorDriver_A2();
				index++;
				break;
			case 2:
				ScanMotorDriver_B1();
				index++;
				break;
			case 3:
				ScanMotorDriver_B2();
				index++;
				break;
			}
		}

		/* Delay a while for next phase */
		Delay_SW(MOTOR_QUICK_DELAY);
	}
}

/******************************************************************************/
void ScanMotorDriver_Move(uint8 direction, uint32 steps)
{
	/* Enable motor driver */
	ScanMotorDriver_Control(MOTOR_ENABLED);

	/* Goto base position */
	if (direction == ScanMotorDriver_DIR_IN)
	{
		/* Move until slider reaches base position */
		while (!CAN_POSSEN_INT_STATE())
		{
			ScanMotorDriver_MoveOneStep(ScanMotorDriver_DIR_IN);
		}
	}
	else if ((direction == ScanMotorDriver_DIR_OUT) && (steps > 0))
	{
	#if TIME_MEASUREMENT_ENABLED
			Alarm_Time_Pin_High();
	#endif
			/* Move out per steps */
			while (steps--)
			{
				ScanMotorDriver_MoveOneStep(ScanMotorDriver_DIR_OUT);
			}
	#if TIME_MEASUREMENT_ENABLED
			Alarm_Time_Pin_Low();
	#endif
		}
		else
		{
			/* Do nothing */
		}

		/* Disable motor driver */
	ScanMotorDriver_Control(MOTOR_DISABLED);
}


/******************************************************************************/
void ScanMotorDriver_GPIO_Init() {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* PE6: MOTOR2 EN */
	GPIO_InitStructure.GPIO_Pin = ScanMotorDriver_EN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ScanMotorDriver_EN_PORT, &GPIO_InitStructure);

	/* PA4  5  PB0  1��IN1  2  3  4 */
	GPIO_InitStructure.GPIO_Pin = ScanMotorDriver_IN1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ScanMotorDriver_IN1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ScanMotorDriver_IN2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ScanMotorDriver_IN2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ScanMotorDriver_IN3_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ScanMotorDriver_IN3_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ScanMotorDriver_IN4_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ScanMotorDriver_IN4_PORT, &GPIO_InitStructure);

	/* Disable motor driver */
	ScanMotorDriver_Control(MOTOR_ENABLED);
}

/******************************************************************************/
void ScanMotorDriver_Init(void) {
	/* Initialize position sensor input pin */
	ScanMotorDriver_PositionSensor_Init();
	/* Initialize motor driver pins */
	ScanMotorDriver_GPIO_Init();
}

/******************************************************************************/
void ScanMotorDriver_Goto_BasePosition(void) {
	if (CAN_POSSEN_INT_STATE())
	{
		/* Already in base position:
		 * Disable motor driver to decrease power consumption */
		ScanMotorDriver_Control(MOTOR_DISABLED);
	} else {
		/* Not in base position */
		ScanMotorDriver_InBasePosition = 0;

		ScanMotorDriver_Move(ScanMotorDriver_DIR_IN, 0);
	}
}

/******************************************************************************/
void ScanMotorDriver_Goto_DetectionPosition(void) {
	/* Disable position sensor interrupt to avoid falsely trigger */
	ScanMotorDriver_PositionSensor_Int_Disable();
	/* Move to detection position */
	ScanMotorDriver_Move(ScanMotorDriver_DIR_OUT, MOTOR_TOTAL_STEPS);
	/* Enable sensor position interrupt */
	ScanMotorDriver_PositionSensor_Int_Enable();
}

/******************************************************************************/
void ScanMotorDriver_Goto_CentrePosition(void)
{
	/* Disable position sensor interrupt to avoid falsely trigger */
	ScanMotorDriver_PositionSensor_Int_Disable();
	/* Move to detection position */
	ScanMotorDriver_Move(ScanMotorDriver_DIR_OUT, MOTOR_HALF_STEPS);
	/* Enable sensor position interrupt */
	ScanMotorDriver_PositionSensor_Int_Enable();
}
/******************************************************************************/
void ScanMotorDriver_StartDetection(void) {
	/* Goto base position */
	ScanMotorDriver_Goto_BasePosition();

	Delay_ms(200);

	/* Goto detection start position */
	ScanMotorDriver_Goto_DetectionPosition();
}

/******************************************************************************/
void ScanMotorDriver_SelfCheck_StepDrive(void)
{
	if(Check_Count)
	{
		SystemManage_5V_Enabled();
		ScanMotorDriver_Goto_BasePosition();
		Delay_ms(500);
		ScanMotorDriver_Goto_DetectionPosition();
		Delay_ms(500);
		ScanMotorDriver_Goto_BasePosition();
		Delay_ms(500);
		Check_Count = 0;
		SystemManage_5V_Disabled();
	}
}

/******************************************************************************/
void SystemManage_5V_Enabled(void)
{
	GPIO_SetBits(ScanMotorDriver_BUTTER_PORT, ScanMotorDriver_BUTTER_PIN);
}
/******************************************************************************/
void SystemManage_5V_Disabled(void)
{
	GPIO_ResetBits(ScanMotorDriver_BUTTER_PORT, ScanMotorDriver_BUTTER_PIN);
}
