/*
 * Font.c
 *
 *  Created on: 2018��2��9��
 *      Author: Administrator
 */
/*
 * Font.c
 *
 *  Created on: 2018��1��25��
 *      Author: Administrator
 *****************************************************************************/
#include "Font.h"
#include "DisplayDriver.h"
#include "DisplayDriver_FontLib.h"
#include "comDef.h"

/*****************************************************************************
��������8X16������ʾ
���ܣ�ѡ��Lcd��ָ����������ʾһ�������ַ�
ע�⣺x��y��d������Ļ����ת���ı䣬λ���Ǿ��ο���ĸ���
��ڲ�����x x�������ʼ��
          y y�������ʼ��
          fc ������ɫ
����ֵ����    �ޱ���ɫ
*******************************************************************************/
void DisplayDriver_Text16(unsigned int x, unsigned int y, unsigned int Color,u8 *s)
{
	unsigned char i,j;
	unsigned int k;
	BlockWrite(0,127,0,159);
    if(*s != '\0')
	{
		while(*s)
		{
			if( *s < 0x80 )
			{
				k=*s;
				if (k>32)
					k-=32;
				else
					k=0;

				for(i=0;i<16;i++)
					for(j=0;j<8;j++)
					{
						if(asc16[k*16+i]&(0x80>>j))
							Lcd_ColorSpot(x+j,y+i,Color);
					}
				s++;x+=8;
			}
			else
			{
				for (k=0;k<HZ16_NUM;k++)
				{
					if ((hz16[k].Index[0]==*(s))&&(hz16[k].Index[1]==*(s+1)))
					{
						for(i=0;i<16;i++)
						{
							for(j=0;j<8;j++)
							{
								if(hz16[k].Msk[i*2]&(0x80>>j))
									Lcd_ColorSpot(x+j,y+i,Color);
							}
							for(j=0;j<8;j++)
							{
								if(hz16[k].Msk[i*2+1]&(0x80>>j))
									Lcd_ColorSpot(x+j+8,y+i,Color);
							}
						}
						break;
					}
				}
				s+=2;x+=16;
			}
		}
	}
}

/*****************************************************************************
��������8X16������ʾ
���ܣ�ѡ��Lcd��ָ����������ʾһ�������ַ�
ע�⣺x��y��d������Ļ����ת���ı䣬λ���Ǿ��ο���ĸ���
��ڲ�����x x�������ʼ��
          y y�������ʼ��
          fc ������ɫ
����ֵ����    �б���ɫ
*******************************************************************************/
void DisplayDriver_Text16_B(uint16 x, uint16 y, uint16 fc,
		uint16 bc, u8 *s)
{
	unsigned char i,j;
	unsigned int k;

	BlockWrite(0,128,0,160);

	while(*s)
	{
		if( *s < 0x80 )
		{
			k=*s;
			if (k>32)
				k-=32;
			else
				k=0;

			for(i=0;i<16;i++)
				for(j=0;j<8;j++)
				{
					if(asc16[k*16+i]&(0x80>>j))
						Lcd_ColorSpot(x+j,y+i,fc);
					else
					{
						if (fc!=bc)
							Lcd_ColorSpot(x+j,y+i,bc);
					}
				}
			s++;x+=8;
		}
		else
		{
			for (k=0;k<HZ16_NUM;k++)
			{
				if ((hz16[k].Index[0]==*(s))&&(hz16[k].Index[1]==*(s+1)))
				{
					for(i=0;i<16;i++)
					{
						for(j=0;j<8;j++)
						{
							if(hz16[k].Msk[i*2]&(0x80>>j))
								Lcd_ColorSpot(x+j,y+i,fc);
							else
							{
								if (fc!=bc)
									Lcd_ColorSpot(x+j,y+i,bc);
							}
						}
						for(j=0;j<8;j++)
						{
							if(hz16[k].Msk[i*2+1]&(0x80>>j))
								Lcd_ColorSpot(x+j+8,y+i,fc);
							else
							{
								if (fc!=bc)
									Lcd_ColorSpot(x+j+8,y+i,bc);
							}
						}
					}
					break;
				}
			}
			s+=2;x+=16;
		}
	}
}

/*****************************************************************************
�������������Ļ
���ܣ����ѡ��Lcd��ָ��������

ע�⣺xStart��xStart��xLong��yLong��Color������Ļ����ת���ı䣬λ���Ǿ��ο���ĸ���
     �������ʾΪ��ɫ
��ڲ�����x x�������ʼ��
          y y�������ʼ��
          fc ������ɫ
����ֵ����    �ޱ���ɫ
*****************************************************************************/

void DisplayDriver_Clear(u16 xStart,u16 yStart,u16 xLong,u16 yLong,u16 Color)
{
	BlockWrite(0,127,0,159);
	Lcd_ColorBox(xStart,yStart,xLong,yLong,Color);
}

/*****************************************************************************
������������
���ܣ����������������
ע�⣺�������ʾΪ��ɫ
��ڲ�������
����ֵ����
*******************************************************************************/
void LCD_Display_Clear()
{
	BlockWrite(0,127,0,159);
	Lcd_Display_Clear();
}

/*****************************************************************************
������������
���ܣ����������������
ע�⣺�������ʾΪ��ɫ
��ڲ�������
����ֵ����
*******************************************************************************/
void DisplayDriver_DrawLine(u16 start_x, u16 start_y, u16 end_x, u16 end_y, u16 color)
{
	u16 t;
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	BlockWrite(0,127,0,159);
	delta_x=end_x-start_x;
	delta_y=end_y-start_y;
	uRow=start_x;
	uCol=start_y;
	if(delta_x>0)incx=1;
	else if(delta_x==0)incx=0;
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if(delta_y==0)incy=0;
	else{incy=-1;delta_y=-delta_y;}
	if( delta_x>delta_y)distance=delta_x;
	else distance=delta_y;
	for(t=0;t<=distance+1;t++ )
	{
		Lcd_ColorSpot(uRow,uCol, color);
		xerr+=delta_x ;
		yerr+=delta_y ;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}
