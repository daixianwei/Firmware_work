/*
 * Interface_Standard.c
 *
 *  Created on: 2018Äê2ÔÂ24ÈÕ
 *      Author: Administrator
 */

/******************************************************************************/
#include "Interface_Standard.h"

/******************************************************************************/
uint8 QR_Code = 0;

/******************************************************************************/
block_attr_Standard block_Standard_Back = {
	ENABLE,									/*Interface Standard rect*/
	{
		0,   20,
		128, 160,
		BACKCOLOR_CONTENT_BACK
	},

	DISABLE,								/*Display HZ16X8*/
	{0},
};

/******************************************************************************/
block_attr_Standard block_Standard_Number = {
	ENABLE,									/*Interface Standard rect*/
	{
		3,   23,
		122, 116,
		White
	},

	ENABLE,									/*Display HZ16X8*/
	{
		"SN:",
		4,   40,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Standard block_Standard_Name = {
	DISABLE,							/*Interface Standard rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"Name:",
		4,   24,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Standard block_Standard_1 = {
	DISABLE,							/*Interface Standard rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"1:AMP",
		4,   56,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Standard block_Standard_2 = {
	DISABLE,							/*Interface Standard rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"2:COC",
		4,   72,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Standard block_Standard_3 = {
	DISABLE,							/*Interface Standard rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"3:THC",
		4,   88,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Standard block_Standard_4 = {
	DISABLE,							/*Interface Standard rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"4:MET",
		4,   104,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Standard block_Standard_5 = {
	DISABLE,							/*Interface Standard rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"5:OPI",
		4,   120,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Standard block_Standard_6 = {
	DISABLE,							/*Interface Standard rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"6:WCG",
		68,   56,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Standard block_Standard_7 = {
	DISABLE,							/*Interface Standard rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"7:EDG",
		68,   72,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Standard block_Standard_8 = {
	DISABLE,							/*Interface Standard rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"8:UZI",
		68,   88,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Standard block_Standard_9 = {
	DISABLE,							/*Interface Standard rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"9:XZG",
		68,   104,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Standard block_Standard_10 = {
	DISABLE,							/*Interface Standard rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"10:PCY",
		68,   120,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Standard block_Standard_Confirm = {
	DISABLE,							/*Interface Standard rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"Please Confirm!",
		4,   141,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Standard* UI_WindowBlocksAttrArray_Standard[][14] = {/* Window: Standard entry */
{&block_Standard_Back,&block_Standard_Number,&block_Standard_Name,&block_Standard_Confirm,
		&block_Standard_1},
{&block_Standard_Back,&block_Standard_Number,&block_Standard_Name,&block_Standard_Confirm,
		&block_Standard_1,&block_Standard_2},
{&block_Standard_Back,&block_Standard_Number,&block_Standard_Name,&block_Standard_Confirm,
		&block_Standard_1,&block_Standard_2,&block_Standard_3},
{&block_Standard_Back,&block_Standard_Number,&block_Standard_Name,&block_Standard_Confirm,
		&block_Standard_1,&block_Standard_2,&block_Standard_3,&block_Standard_4},
{&block_Standard_Back,&block_Standard_Number,&block_Standard_Name,&block_Standard_Confirm,
		&block_Standard_1,&block_Standard_2,&block_Standard_3,&block_Standard_4,&block_Standard_5},
{&block_Standard_Back,&block_Standard_Number,&block_Standard_Name,&block_Standard_Confirm,
		&block_Standard_1,&block_Standard_2,&block_Standard_3,&block_Standard_4,&block_Standard_5,
		&block_Standard_6},
{&block_Standard_Back,&block_Standard_Number,&block_Standard_Name,&block_Standard_Confirm,
		&block_Standard_1,&block_Standard_2,&block_Standard_3,&block_Standard_4,&block_Standard_5,
		&block_Standard_6,&block_Standard_7},
{&block_Standard_Back,&block_Standard_Number,&block_Standard_Name,&block_Standard_Confirm,
		&block_Standard_1,&block_Standard_2,&block_Standard_3,&block_Standard_4,&block_Standard_5,
		&block_Standard_6,&block_Standard_7,&block_Standard_8},
{&block_Standard_Back,&block_Standard_Number,&block_Standard_Name,&block_Standard_Confirm,
		&block_Standard_1,&block_Standard_2,&block_Standard_3,&block_Standard_4,&block_Standard_5,
		&block_Standard_6,&block_Standard_7,&block_Standard_8,&block_Standard_9},
{&block_Standard_Back,&block_Standard_Number,&block_Standard_Name,&block_Standard_Confirm,
		&block_Standard_1,&block_Standard_2,&block_Standard_3,&block_Standard_4,&block_Standard_5,
		&block_Standard_6,&block_Standard_7,&block_Standard_8,&block_Standard_9,&block_Standard_10},
};

/******************************************************************************/
uint8 Interface_Standard(uint16 KeyCode)
{
	Exti_lock = DISABLE;
	if(Cup_Count)
	{
		UI_WindowBlocks_Standard = sizeof(UI_WindowBlocksAttrArray_Standard[Cup_Count-1]) >> 2;
		UI_Draw_Window_Standard(UI_WindowBlocks_Standard);
		UI_state = UI_STATE_START;
	}
	else
	{
		UI_state = UI_STATE_QUICK;
	}
	Delay_ms(3000);
	return UI_STATE_RERUN;
}

/******************************************************************************/
void UI_Draw_Block_Standard(block_attr_Standard* block);

/******************************************************************************/
void UI_Draw_Window_Standard(uint16 blockNum)
{
	uint8 blockIndex = 0;					/* Draw blocks one by one */
	for (blockIndex = 0; blockIndex < blockNum; blockIndex++)
	{
		if(UI_WindowBlocksAttrArray_Standard[Cup_Count-1][blockIndex])
		{
			UI_Draw_Block_Standard(UI_WindowBlocksAttrArray_Standard[Cup_Count-1][blockIndex]);
		}
	}
}

/******************************************************************************/
void UI_Draw_Block_Standard(block_attr_Standard* block)
{
	Display_Time = 0;
	if (block->rect_enabled)				/* 1. Draw Rect */
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
