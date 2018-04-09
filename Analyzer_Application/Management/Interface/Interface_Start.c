/*
 * Interface_Start.c
 *
 *  Created on: 2018Äê2ÔÂ26ÈÕ
 *      Author: Administrator
 */

/******************************************************************************/
#include "Interface_Start.h"

/******************************************************************************/
block_attr_Start block_Start_Back = {
	ENABLE,								/*Interface Start rect*/
	{
		0,   20,
		128, 140,
		BACKCOLOR_CONTENT_BACK
	},
	DISABLE,							/*Display HZ16X8*/
	{0},
};

/******************************************************************************/
block_attr_Start block_Start_Name = {
	ENABLE,								/*Interface Start rect*/
	{
		3,   30,
		122, 116,
		White
	},
	ENABLE,								/*Display HZ16X8*/
	{
		"Name:",
		4,   32,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Start block_Start_Number = {
	DISABLE,							/*Interface Start rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"SN:",
		4,   48,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Start block_Start_1 = {
	DISABLE,							/*Interface Start rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
			"1:AMP",
			4,   64,
			Black,White,
			White
	},
};

/******************************************************************************/
block_attr_Start block_Start_2 = {
	DISABLE,							/*Interface Start rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"2:COC",
		4,   80,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Start block_Start_3 = {
	DISABLE,							/*Interface Start rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"3:THC",
		4,   96,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Start block_Start_4 = {
	DISABLE,							/*Interface Start rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"4:MET",
		4,   112,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Start block_Start_5 = {
	DISABLE,							/*Interface Start rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"5:OPI",
		4,   128,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Start block_Start_6 = {
	DISABLE,							/*Interface Start rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"6:WCG",
		68,   64,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Start block_Start_7 = {
	DISABLE,							/*Interface Start rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"7:EDG",
		68,   80,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Start block_Start_8 = {
	DISABLE,							/*Interface Start rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"8:UZI",
		68,   96,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Start block_Start_9 = {
	DISABLE,							/*Interface Start rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"9:XZG",
		68,   112,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Start block_Start_10 = {
	DISABLE,							/*Interface Start rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"10:PCY",
		68,   128,
		Black,White,
		White
	},
};

/******************************************************************************/
block_attr_Start* UI_WindowBlocksAttrArray_Start[10][13] = {/* Window: Start entry */
{&block_Start_Back,&block_Start_Name,&block_Start_Number,&block_Start_1},
{&block_Start_Back,&block_Start_Name,&block_Start_Number,&block_Start_1,&block_Start_2},
{&block_Start_Back,&block_Start_Name,&block_Start_Number,&block_Start_1,&block_Start_2,
		&block_Start_3},
{&block_Start_Back,&block_Start_Name,&block_Start_Number,&block_Start_1,&block_Start_2,
		&block_Start_3,&block_Start_4},
{&block_Start_Back,&block_Start_Name,&block_Start_Number,&block_Start_1,&block_Start_2,
		&block_Start_3,&block_Start_4,&block_Start_5},
{&block_Start_Back,&block_Start_Name,&block_Start_Number,&block_Start_1,&block_Start_2,
		&block_Start_3,&block_Start_4,&block_Start_5,&block_Start_6},
{&block_Start_Back,&block_Start_Name,&block_Start_Number,&block_Start_1,&block_Start_2,
		&block_Start_3,&block_Start_4,&block_Start_5,&block_Start_6,
		&block_Start_7},
{&block_Start_Back,&block_Start_Name,&block_Start_Number,&block_Start_1,&block_Start_2,
		&block_Start_3,&block_Start_4,&block_Start_5,&block_Start_6,
		&block_Start_7,&block_Start_8},
{&block_Start_Back,&block_Start_Name,&block_Start_Number,&block_Start_1,&block_Start_2,
		&block_Start_3,&block_Start_4,&block_Start_5,&block_Start_6,
		&block_Start_7,&block_Start_8,&block_Start_9},
{&block_Start_Back,&block_Start_Name,&block_Start_Number,&block_Start_1,&block_Start_2,
		&block_Start_3,&block_Start_4,&block_Start_5,&block_Start_6,
		&block_Start_7,&block_Start_8,&block_Start_9,&block_Start_10},
};

/******************************************************************************/
uint8 Interface_Start(uint16 KeyCode)
{
	Exti_lock = DISABLE;
	Interface_Key = 1;
	QRCode_Trigger_Disabled();
	UI_WindowBlocks_Start = sizeof(UI_WindowBlocksAttrArray_Start[Cup_Count-1]) >> 2;
	UI_Draw_Window_Start(UI_WindowBlocks_Start);
	Exti_lock = ENABLE;
	UI_state = UI_STATE_KEY_STATE;
	while(!key_state);
	return UI_STATE_RERUN;
}

/******************************************************************************/
void UI_Draw_block_Start(block_attr_Start* block);

/******************************************************************************/
void UI_Draw_Window_Start(uint16 blockNum)
{
	uint8 blockIndex = 0;					/* Draw blocks one by one */
	for (blockIndex = 0; blockIndex < blockNum; blockIndex++)
	{
		if(UI_WindowBlocksAttrArray_Start[Cup_Count-1][blockIndex])
		{
			UI_Draw_block_Start(UI_WindowBlocksAttrArray_Start[Cup_Count-1][blockIndex]);
		}
	}
}

/******************************************************************************/
void UI_Draw_block_Start(block_attr_Start* block)
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
