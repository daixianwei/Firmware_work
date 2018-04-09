/*
 * Interface_Result.c
 *
 *  Created on: 2018年2月26日
 *      Author: Administrator
 */

/******************************************************************************/
#include "Interface_Result.h"

/******************************************************************************/
uint16 UI_WindowBlocks_Result = 0;
extern uint8 QRCode_received;
extern const unsigned char gImage_Right_arrow[1050];

/******************************************************************************/
block_attr_Result block_Result_Back = {
	ENABLE,									/*Interface Result rect*/
	{
		0,   20,
		128, 140,
		BACKCOLOR_CONTENT_BACK
	},

	DISABLE,								/*Display HZ16X8*/
	{0},

	DISABLE,								/*Display HZ16X8*/
	{0},
};

/******************************************************************************/
block_attr_Result block_Result_Item = {
	ENABLE,									/*Interface Result rect*/
	{
		7,   30,
		114, 22,
		BACKCOLOR_CONTENT_BAR
	},

	ENABLE,									/*Display HZ16X8*/
	{
		"Item",
		9,   34,
		Black,White,
		BACKCOLOR_CONTENT_BAR
	},
	DISABLE,								/*Display HZ16X8*/
	{0},
};

/******************************************************************************/
block_attr_Result block_Result_AMP = {
	DISABLE,								/*Display HZ16X8*/
	{0},

	ENABLE,									/*Display HZ16X8*/
	{
		"AMP:   INV",
		12,   54,
		Black,White,
		White
	},
	DISABLE,								/*Display HZ16X8*/
	{0}
};

/******************************************************************************/
block_attr_Result block_Result_COC = {
	DISABLE,								/*Display HZ16X8*/
	{0},

	ENABLE,									/*Display HZ16X8*/
	{
		"COC:   INV",
		12,   70,
		Black,White,
		White
	},
	DISABLE,								/*Display HZ16X8*/
	{0}
};

/******************************************************************************/
block_attr_Result block_Result_THC = {
	DISABLE,								/*Display HZ16X8*/
	{0},

	ENABLE,									/*Display HZ16X8*/
	{
		"THC:   INV",
		12,   86,
		Black,White,
		White
	},
	DISABLE,								/*Display HZ16X8*/
	{0}
};

/******************************************************************************/
block_attr_Result block_Result_MET = {
	DISABLE,								/*Display HZ16X8*/
	{0},

	ENABLE,									/*Display HZ16X8*/
	{
		"MET:   INV",
		12,   102,
		Black,White,
		White
	},
	DISABLE,								/*Display HZ16X8*/
	{0}
};

/******************************************************************************/
block_attr_Result block_Result_OPI = {
	DISABLE,								/*Display HZ16X8*/
	{0},

	ENABLE,									/*Display HZ16X8*/
	{
		"OPI:   INV",
		12,   118,
		Black,White,
		White
	},
	DISABLE,								/*Display HZ16X8*/
	{0}
};

/******************************************************************************/
block_attr_Result block_Result_Right_arrow = {
	ENABLE,								/*Interface Result rect*/
	{
		7,   52,
		114, 85,
		White
	},

	ENABLE,									/*Display HZ16X8*/
	{
		"Result",
		60,  34,
		Black,White,
		BACKCOLOR_CONTENT_BAR
	},

	ENABLE,									/* Display Picture */
	{
		gImage_Right_arrow,
		90, 142,
		35, 15
	},
};

/******************************************************************************/
block_attr_Result* UI_WindowBlocksAttrArray_Result[] = {/* Window: Result entry */
		&block_Result_Back,
		&block_Result_Item,
		&block_Result_Right_arrow,
		&block_Result_AMP,
		&block_Result_COC,
		&block_Result_THC,
		&block_Result_MET,
		&block_Result_OPI,
};

/******************************************************************************/
uint8 Interface_Result(uint16 KeyCode)
{
	Exti_lock = DISABLE;
	Page_Flag = 0;
	Interface_Key = 2;
	UI_WindowBlocks_Result = sizeof(UI_WindowBlocksAttrArray_Result) >> 2;
	UI_Draw_Window_Result(UI_WindowBlocks_Result);
	Exti_lock = ENABLE;
	while(!key_state);
	if(Char_Count)							/* 字符数计算判断未完成  */
	{
		UI_state = UI_STATE_KEY_STATE;
	}
	return UI_STATE_RERUN;
}

/******************************************************************************/
void UI_Draw_block_Result(block_attr_Result* block);

/******************************************************************************/
void UI_Draw_Window_Result(uint16 blockNum)
{
	uint8 blockIndex = 0;					/* Draw blocks one by one */
	for (blockIndex = 0; blockIndex < blockNum; blockIndex++)
	{
		UI_Draw_block_Result(UI_WindowBlocksAttrArray_Result[blockIndex]);
	}
}

/******************************************************************************/
void UI_Draw_block_Result(block_attr_Result* block)
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

//	if(Char_Count)					/* 字符数计算判断未完成  */
//	{
		if (block->pic_enabled)				/* 2. Draw picture */
		{
			DisplayDriver_DrawPic(block->pic_attr.offsetX,
					block->pic_attr.offsetY, block->pic_attr.width,
					block->pic_attr.height,block->pic_attr.src);
		}
//	}

	key_state = DISABLE;
	Display_Time = 1;
}
