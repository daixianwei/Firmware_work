/*
 * DisplayDriver.c
 *
 *  Created on: 2018��2��9��
 *      Author: Administrator
 */

/*******************************************************************************
*��Ҫ˵����
��.h�ļ��У�#define Immediatelyʱ��������ʾ��ǰ����
�����#define Delay����ֻ����ִ����LCD_WR_REG(0x0007,0x0173);
֮��Ż���ʾ��ִ��һ��LCD_WR_REG(0x0007,0x0173)������д����
�ݶ�������ʾ��
#define Delayһ�����ڿ����������ʾ����ֹ��ʾ��ȫ��ͼ���ˢ�¹���
*******************************************************************************/
#include "stm32f10x.h"
#include "DisplayDriver.h"
#include "Font.h"
#include "comDef.h"

/*******************************************************************************
	��������LCD_GPIO_Config
	����  ������FSMC����LCD��I/O
	����  ����
	���  ����
	����  ���ڲ�����
*******************************************************************************/
 void LCD_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* config tft rst gpio */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 |GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* config tft back_light gpio base on the PT4101 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* config tft rst gpio */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* config tft data lines base on FSMC
	 * data lines,FSMC-D0~D15: PD 14 15 0 1,PE 7 8 9 10 11 12 13 14 15,PD 10
	 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 |
    		GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 |
    		GPIO_Pin_10 ;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /* config tft control lines base on FSMC
	 * PD4-FSMC_NOE  :LCD-RD
	 * PD5-FSMC_NWE  :LCD-WR
	 * PD7-FSMC_NE1  :LCD-CS
	 * PD11-FSMC_A16 :LCD-DC
	 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* tft control gpio init */
    GPIO_SetBits(GPIOD, GPIO_Pin_4);		 	// RD = 1
    GPIO_SetBits(GPIOD, GPIO_Pin_5);		 	// WR = 1
    GPIO_SetBits(GPIOD, GPIO_Pin_7);		 	// CS = 1
    GPIO_SetBits(GPIOD, GPIO_Pin_13);			// RST = 1

}

/******************************************************************************/
 void RCC_Configuration(void)
 {
	/* Enable the FSMC Clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB
				| RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD| RCC_APB2Periph_GPIOE
				| RCC_APB2Periph_AFIO , ENABLE);

		/* Disable JTAG but enable SWD because PB3/4 are used for GPIO */
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
 }

/*******************************************************************************
	��������LCD_FSMC_Config
	����  ��LCD  FSMC ģʽ����
	����  ����
	���  ����
	����  ���ڲ�����
*******************************************************************************/
static void LCD_FSMC_Config(void)
{
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  p;


    p.FSMC_AddressSetupTime = 0x02;	 		//��ַ����ʱ��
    p.FSMC_AddressHoldTime = 0x00;	 		//��ַ����ʱ��
    p.FSMC_DataSetupTime = 0x05;			//���ݽ���ʱ��
    p.FSMC_BusTurnAroundDuration = 0x00;
    p.FSMC_CLKDivision = 0x00;
    p.FSMC_DataLatency = 0x00;

    p.FSMC_AccessMode = FSMC_AccessMode_B;	// һ��ʹ��ģʽB������LCD

    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

    /* Enable FSMC Bank1_SRAM Bank */
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
}

/*******************************************************************************
	��������Delay_us
	��  ��: ��
	��  ��: ��
	����˵������ʱ����    ���ڲ�ʹ��
	��ʼ������Ӳ���豸��δ�����жϡ�
*******************************************************************************/
volatile static void Delay(__IO u32 nCount)
{
	volatile int i;
	for(i=0;i<7200;i++)
    for(; nCount != 0; nCount--);
}

/******************************************************************************/
u16 ssd1289_GetPoint(u16 x,u8 y)
{
	 u16 a = 0;

	*(__IO u16 *) (Bank1_LCD_C) = 0x4f;
	*(__IO u16 *) (Bank1_LCD_D) = x;

	*(__IO u16 *) (Bank1_LCD_C) = 0x4e;
	*(__IO u16 *) (Bank1_LCD_D) = y;

	*(__IO u16 *) (Bank1_LCD_C) = 0x22;

	 a = *(__IO u16 *) (Bank1_LCD_D);
   return(a);
}

/******************************************************************************/
volatile static void LCD_Rst(void)
{
    Clr_Rst;
    Delay(10000);
    Set_Rst;
    Delay(10000);
}

/******************************************************************************/
static void WriteComm(u16 CMD)
{
	*(__IO u16 *) (Bank1_LCD_C) = CMD;
}

/******************************************************************************/
static void WriteData(u16 tem_data)
{
	*(__IO u16 *) (Bank1_LCD_D) = tem_data;
}

/******************************************************************************/
u16 ReadPixel(u16 x,u8 y)
{
	u16 dat,temp;
	WriteComm(0x2a);
	WriteData(x>>8);
	WriteData(x&0xff);
	WriteComm(0x2b);
	WriteData(y>>8);
	WriteData(y&0xff);
	WriteComm(0x2e);
	dat = *(__IO u16 *) (Bank1_LCD_D);printf("dat_1=%04x\r\n",dat);
	dat = *(__IO u16 *) (Bank1_LCD_D);printf("dat_2=%04x\r\n",dat);
	temp = *(__IO u16 *) (Bank1_LCD_D);printf("dat_3=%04x\r\n",temp);
	dat = (dat&0xf800)|((dat&0x00fc)<<3)|(temp&0xFF00)>>11;
	return dat;
}

/******************************************************************************/
void DisplayDriver_Init(void)
{
int a;
LCD_GPIO_Config();
LCD_FSMC_Config();
Delay(20);
LCD_Rst();
//************* ILI9163C **********//
WriteComm(0x11);				//Exit Sleep
Delay(20);

WriteComm(0x26); 				//Set Default Gamma
WriteData(0x04);

WriteComm(0xB1);
WriteData(0x08);
WriteData(0x10);

WriteComm(0xC0); 				//Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
WriteData(0x02);
WriteData(0x00);

WriteComm(0xC1); 				//Set BT[2:0] for AVDD & VCL & VGH & VGL
WriteData(0x02);

WriteComm(0xC5); 				//Set VMH[6:0] & VML[6:0] for VOMH & VCOML
WriteData(0x4C);
WriteData(0x5E);

WriteComm(0xC7);
WriteData(0x00);

WriteComm(0x3a); 				//Set Color Format
WriteData(0x05);

WriteComm(0x2A);				//Set Column Address
WriteData(0x00);
WriteData(0x00);
WriteData(0x00);
WriteData(0x7F);

WriteComm(0x2B);				//Set Page Address
WriteData(0x00);
WriteData(0x00);
WriteData(0x00);
WriteData(0x9F);

WriteComm(0x36);				//Set Scanning Direction
WriteData(0xC0);

WriteComm(0xB7);				//Set Source Output Direction
WriteData(0x00);

WriteComm(0xf2);				//Enable Gamma bit
WriteData(0x01);

WriteComm(0xE0);
WriteData(0x3F);				//p1
WriteData(0x22);				//p2
WriteData(0x20);				//p3
WriteData(0x24);				//p4
WriteData(0x20);				//p5
WriteData(0x0C);				//p6
WriteData(0x4E);				//p7
WriteData(0xB7);				//p8
WriteData(0x3C);				//p9
WriteData(0x19);				//p10
WriteData(0x22);				//p11
WriteData(0x1E);				//p12
WriteData(0x02);				//p13
WriteData(0x01);				//p14
WriteData(0x00);				//p15

WriteComm(0xE1);
WriteData(0x00);				//p1
WriteData(0x1B);				//p2
WriteData(0x1F);				//p3
WriteData(0x0B);				//p4
WriteData(0x0F);				//p5
WriteData(0x13);				//p6
WriteData(0x31);				//p7
WriteData(0x84);				//p8
WriteData(0x43);				//p9
WriteData(0x06);				//p10
WriteData(0x1D);				//p11
WriteData(0x21);				//p12
WriteData(0x3D);				//p13
WriteData(0x3E);				//p14
WriteData(0x3F);				//p15

WriteComm(0x29);				// Display On
//Lcd_Light_ON;

	Delay_ms_SW(2000);
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4))
    {
    	GPIO_SetBits(GPIOB, GPIO_Pin_3);
    }
}

/*******************************************************************************
��������Lcdд�����
���ܣ���Lcdָ��λ��д��Ӧ�����������
��ڲ�����Index ҪѰַ�ļĴ�����ַ
          ConfigTemp д������ݻ�����ֵ
*******************************************************************************/
void LCD_WR_REG(u16 Index,u16 CongfigTemp)
{
	*(__IO u16 *) (Bank1_LCD_C) = Index;
	*(__IO u16 *) (Bank1_LCD_D) = CongfigTemp;
}

/*******************************************************************************
��������Lcdд��ʼ����
���ܣ�����Lcd�������� ִ��д����
*******************************************************************************/
void Lcd_WR_Start(void)
{
*(__IO u16 *) (Bank1_LCD_C) = 0x2C;
}

/*******************************************************************************
��������Lcd��ѡ����
���ܣ�ѡ��Lcd��ָ���ľ�������

ע�⣺xStart��yStart��Xend��Yend������Ļ����ת���ı䣬λ���Ǿ��ο���ĸ���

��ڲ�����xStart x�������ʼ��
          ySrart y�������ʼ��
          Xend   y�������ֹ��
          Yend   y�������ֹ��
����ֵ����
*******************************************************************************/
void BlockWrite(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend)
{
	WriteComm(0x2a);
	WriteData(Xstart>>8);
	WriteData(Xstart&0xff);
	WriteData(Xend>>8);
	WriteData(Xend&0xff);

	WriteComm(0x2b);
	WriteData(Ystart>>8);
	WriteData(Ystart&0xff);
	WriteData(Yend>>8);
	WriteData(Yend&0xff);

	WriteComm(0x2c);
}

/*******************************************************************************
��������Lcd��ѡ����
���ܣ�ѡ��Lcd��ָ���ľ�������

ע�⣺xStart��yStart��Xend��Yend������Ļ����ת���ı䣬λ���Ǿ��ο���ĸ���

��ڲ�����xStart x�������ʼ��
          ySrart y�������ʼ��
          Xend   y�������ֹ��
          Yend   y�������ֹ��
����ֵ����
*******************************************************************************/
void BlockWrite_Test(unsigned int Xstart,unsigned int Ystart)
{
	WriteComm(0x2a);
	WriteData(Xstart);
	WriteData(Xstart&0xff);

	WriteComm(0x2b);
	WriteData(Ystart);
	WriteData(Ystart&0xff);

	WriteComm(0x2c);
}

/*******************************************************************************
��������Lcd��ѡ����
���ܣ�ѡ��Lcd��ָ���ľ�������

ע�⣺xStart�� yStart������Ļ����ת���ı䣬λ���Ǿ��ο���ĸ���

��ڲ�����xStart x�������ʼ��
        ySrart y�������ֹ��
        xLong Ҫѡ�����ε�x���򳤶�
        yLong  Ҫѡ�����ε�y���򳤶�
����ֵ����
*******************************************************************************/
void Lcd_ColorBox(u16 xStart,u16 yStart,u16 xLong,u16 yLong,u16 Color)
{
	u32 temp;
	BlockWrite(xStart,xStart+xLong-1,yStart,yStart+yLong-1);
	for (temp=0; temp<xLong*yLong; temp++)
	{
		*(__IO u16 *) (Bank1_LCD_D) = Color>>8;
		*(__IO u16 *) (Bank1_LCD_D) = Color;
	}
}

/*******************************************************************************
��������Lcd����
���ܣ����LCD������������
��ڲ�������
����ֵ����
*******************************************************************************/
void Lcd_Display_Clear(void)
{
	u32 temp;
	BlockWrite(0,127,0,159);
	for (temp=0; temp<128*160; temp++)
	{
		*(__IO u16 *) (Bank1_LCD_D) = 0>>8;
		*(__IO u16 *) (Bank1_LCD_D) = 0;
	}
}

/*******************************************************************************
��������Lcd��ѡ����
���ܣ�ѡ��Lcd��ָ���ĵ�

ע�⣺xStart�� yStart������Ļ����ת���ı䣬λ���Ǿ��ο���ĸ���

��ڲ�����xStart x�������ʼ��
          ySrart y�������ֹ��
����ֵ����
*******************************************************************************/
void Lcd_ColorSpot(u16 xStart,u16 yStart,u16 Color)
{

	BlockWrite_Test(xStart,yStart);
	*(__IO u16 *) (Bank1_LCD_D) = Color>>8;
	*(__IO u16 *) (Bank1_LCD_D) = Color;
}

/*******************************************************************************
��������Lcdͼ�����100*100
���ܣ���Lcdָ��λ�����ͼ��
��ڲ�����Index ҪѰַ�ļĴ�����ַ
        ConfigTemp д������ݻ�����ֵ
*******************************************************************************/
void DisplayDriver_DrawPic(u16 x, u16 y,u16 pic_H, u16 pic_V, const unsigned char* pic)
{
	unsigned long i;
	unsigned int j;

	WriteComm(0x36); 					//Set_address_mode
	WriteData(0xC0); 					//����
	BlockWrite(x,x+pic_H-1,y,y+pic_V-1);
	for (i = 0; i < pic_H*pic_V*2; i++)
	{
		*(__IO u16 *) (Bank1_LCD_D) = pic[i];
	}
	WriteComm(0x36); 					//Set_address_mode
	WriteData(0xC0);					//����
}

/******************************************************************************/
void DrawPixel(u16 x, u16 y, u16 Color)
{
	BlockWrite(x,x+1,y,y+1);
	*(__IO u16 *) (Bank1_LCD_D) = Color;
}

/******************************************************************************/
typedef __packed struct
{
	u8  pic_head[2];				//1
	u16 pic_size_l;			    	//2
	u16 pic_size_h;			   		//3
	u16 pic_nc1;				    //4
	u16 pic_nc2;				    //5
	u16 pic_data_address_l;	    	//6
	u16 pic_data_address_h;			//7
	u16 pic_message_head_len_l;		//8
	u16 pic_message_head_len_h;		//9
	u16 pic_w_l;					//10
	u16 pic_w_h;				    //11
	u16 pic_h_l;				    //12
	u16 pic_h_h;				    //13
	u16 pic_bit;				    //14
	u16 pic_dip;				    //15
	u16 pic_zip_l;			    	//16
	u16 pic_zip_h;			    	//17
	u16 pic_data_size_l;		    //18
	u16 pic_data_size_h;		    //19
	u16 pic_dipx_l;			   		//20
	u16 pic_dipx_h;			    	//21
	u16 pic_dipy_l;			    	//22
	u16 pic_dipy_h;			    	//23
	u16 pic_color_index_l;	    	//24
	u16 pic_color_index_h;	    	//25
	u16 pic_other_l;			    //26
	u16 pic_other_h;			    //27
	u16 pic_color_p01;		    	//28
	u16 pic_color_p02;		    	//29
	u16 pic_color_p03;		    	//30
	u16 pic_color_p04;		    	//31
	u16 pic_color_p05;		    	//32
	u16 pic_color_p06;		    	//33
	u16 pic_color_p07;		   		//34
	u16 pic_color_p08;				//35
}BMP_HEAD;

BMP_HEAD bmp;

