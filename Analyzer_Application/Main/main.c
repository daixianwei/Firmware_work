/*******************************************************************************
 * File name: main.c
 * Author: Hanson Liu
 * Mail: han_liu_zju@sina.com
 * Date: 2014.12 ~ Now
 ******************************************************************************/
#include "DisplayDriver.h"
#include "main.h"

/******************************************************************************/
#define SYSTICK_FREQ_72MHz  72000000
#define SYSTICK_FREQ_1MS    SYSTICK_FREQ_72MHz/1000

/******************************************************************************/
static u32 TimingDelay;					/* Timer Delay Count */
uint8 Power_Open = 0;
uint8 Display_Time = 1;

extern const unsigned char gImage_Right_arrow[1050];
extern const unsigned char gImage_Left_arrow[1050];
/******************************************************************************/
void main(void)
{
	RCC_Configuration();				/* RCC Initialize */
	HumanInput_Init();					/* KEY HumanInput Initialize */
	DisplayDriver_Init();				/* LCD DisDisplayDriver Initialize */
	SysTick_Init(SYSTICK_FREQ_1MS);		/* Initialize system tick timer */
	PCF8563_Init();
	EXTIX_Init();						/* EXTIX Init */
	QRCode_Init();						/* QRCode Initialize */
	RotationMotor_Init();				/* RotationMotorDriver Initialize */
	ScanMotorDriver_Init();				/* RotationMotor Initialize  */
	SignalSample_Sample_Init();
	USB_VirtualCOM_Init();
	Status_Init();
	HostComm_Init();


	while(1)
	{

//		SignalSample_SampleStrip(0);

		Interface_Process(0);			/* User Interface */


//		USB_VirtualCOM_Process();
//
//		/* Host communication */
//		HostComm_Process();
	}
}

/******************************************************************************/
void SysTick_Init(u32 ticks)
{
	if(SysTick_Config(ticks))
	{
		while (1);						/* Capture error */
	}
}

/*******************************************************************************
* Timer Delay
*******************************************************************************/
void Delay_ms(u32 nTime)
{
	TimingDelay = nTime;

	while(TimingDelay != 0);
}

/******************************************************************************/
void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{
		TimingDelay--;
	}
}

/******************************************************************************/
void Status_Init(void)
{
	Display_Time = 0;
	GPIO_SetBits(GPIOD, GPIO_Pin_2);
	DisplayDriver_DrawPic(0,0,128,160,gImage_Power_on);
	Lcd_ColorBox(0,0,128,20,Dark_Blue);
	Battery_Empty_ICO();						/* Battery Empty ICO  */
	Display_Time = 1;
	RotationMotor_SelfCheck_StepDrive();
	ScanMotorDriver_SelfCheck_StepDrive();
	Power_Open = 1;
}
