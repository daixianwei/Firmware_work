/*
 * Interface_Record.c
 *
 *  Created on: 2018Äê2ÔÂ24ÈÕ
 *      Author: Administrator
 ******************************************************************************/
#include "Interface_Record.h"

/******************************************************************************/
block_attr_Record block_Record_1 = {
	ENABLE,								/*Interface Record rect */
	{
		0,   20,
		128, 140,
		BACKCOLOR_CONTENT_BACK
	},

	DISABLE,							/*Display HZ16X8*/
	{0},

	DISABLE,							/*Parting line*/
	{0},
};

/******************************************************************************/
block_attr_Record block_Record_2 = {
	ENABLE,								/*Interface Record rect */
	{
		7,   25,
		114, 130,
		White
	},

	ENABLE,								/*Display HZ16X8*/
	{
		"ID",
		9,   25,
		Black,White,
		White
	},

	ENABLE,								/*Parting line*/
	{
		7,    42,
		120,  42,
		BACKCOLOR_CONTENT_BACK
	},
};

/******************************************************************************/
block_attr_Record block_Record_3 = {
	DISABLE,							/*Interface Record rect */
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"Name",
		9,   43,
		Black,White,
		White
	},

	ENABLE,								/*Parting line*/
	{
		7,    59,
		120,  59,
		BACKCOLOR_CONTENT_BACK
	},
};

/******************************************************************************/
block_attr_Record block_Record_4 = {
	DISABLE,							/*Interface Record rect */
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"Time",
		9,   60,
		Black,White,
		White
	},

	ENABLE,								/*Parting line*/
	{
		7,    76,
		120,  76,
		BACKCOLOR_CONTENT_BACK
	},
};

/******************************************************************************/
block_attr_Record block_Record_5 = {
	DISABLE,							/*Interface Record rect */
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"Item Result",
		15,   77,
		Black,White,
		White
	},

	ENABLE,								/*Parting line*/
	{
		7,    93,
		120,  93,
		BACKCOLOR_CONTENT_BACK
	},
};

/******************************************************************************/
block_attr_Record block_Record_6 = {
	DISABLE,							/*Interface Record rect */
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"Item Result",
		15,   116,
		Black,White,
		White
	},

	ENABLE,								/*Parting line*/
	{
		7,    116,
		120,  116,
		BACKCOLOR_CONTENT_BACK
	},
};

/******************************************************************************/
block_attr_Record block_Record_7 = {
	DISABLE,							/*Interface Record rect */
	{0},

	DISABLE,							/*Display HZ16X8*/
	{0},

	ENABLE,								/*Parting line*/
	{
		7,    132,
		120,  132,
		BACKCOLOR_CONTENT_BACK
	},
};

/******************************************************************************/
block_attr_Record block_Record_8 = {
	DISABLE,							/*Interface Record rect */
	{0},

	DISABLE,							/*Display HZ16X8*/
	{0},

	ENABLE,								/*Parting line*/
	{
		43,  25,
		43,  75,
		BACKCOLOR_CONTENT_BACK
	},
};

/******************************************************************************/
block_attr_Record* UI_WindowBlocksAttrArray_Record[] = {/* Window: Standard entry */
		&block_Record_1,
		&block_Record_2,
		&block_Record_3,
		&block_Record_4,
		&block_Record_5,
		&block_Record_6,
		&block_Record_7,
		&block_Record_8,
};

/******************************************************************************/
void UI_Draw_Block_Record(block_attr_Record* block);

/******************************************************************************/
uint8 Interface_Record(uint16 KeyCode)
{
	Exti_lock = DISABLE;
	UI_WindowBlocks_Record = sizeof(UI_WindowBlocksAttrArray_Record) >> 2;
	UI_Draw_Window_Record(UI_WindowBlocks_Record);
	Delay_ms(2000);
	Exti_lock = ENABLE;
	UI_state = UI_STATE_MAIN_WINDOW;
	key_state_confirm = DISABLE;
	Interface_Key = DISABLE;
	key_state = ENABLE;
	return UI_STATE_RERUN;
}

/******************************************************************************/
void UI_Draw_Window_Record(uint16 blockNum)
{
	uint8 blockIndex = 0;					/* Draw blocks one by one */
	for (blockIndex = 0; blockIndex < blockNum; blockIndex++)
	{
		UI_Draw_Block_Record(UI_WindowBlocksAttrArray_Record[blockIndex]);
	}
	key_state = ENABLE;
}

/******************************************************************************/
void UI_Draw_Block_Record(block_attr_Record* block)
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

	if (block->line_enabled)				/*3.Parting line*/
	{
		DisplayDriver_DrawLine(block->Parting_line_attr.startX,
				block->Parting_line_attr.startY,
				block->Parting_line_attr.endX,
				block->Parting_line_attr.endY,
				block->Parting_line_attr.color);
	}
	Display_Time = 1;
}
