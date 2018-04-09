/*
 * Interface_Result_2.c
 *
 *  Created on: 2018Äê3ÔÂ16ÈÕ
 *      Author: Administrator
 */

/******************************************************************************/
#include "Interface_Result_2.h"

/******************************************************************************/
extern uint8 Page_Flag;
uint16 UI_WindowBlocks_Result_2 = 0;
extern const unsigned char gImage_Left_arrow[1050];

/******************************************************************************/
block_attr_Result block_Result_WCG = {
	DISABLE,							/* Display HZ16X8 */
	{0},

	ENABLE,								/* Display HZ16X8 */
	{
		"WCG:   INV",
		12,   54,
		Black,White,
		White
	},

	DISABLE,							/* Display Picture */
	{0}
};

/******************************************************************************/
block_attr_Result block_Result_EDG = {
	DISABLE,							/* Display HZ16X8 */
	{0},

	ENABLE,								/* Display HZ16X8 */
	{
		"EDG:   INV",
		12,   70,
		Black,White,
		White
	},

	DISABLE,							/* Display Picture */
	{0}
};

/******************************************************************************/
block_attr_Result block_Result_UZI = {
	DISABLE,							/* Display HZ16X8 */
	{0},

	ENABLE,								/* Display HZ16X8 */
	{
		"UZI:   INV",
		12,   86,
		Black,White,
		White
	},

	DISABLE,							/* Display Picture */
	{0}
};

/******************************************************************************/
block_attr_Result block_Result_XZG = {
	DISABLE,							/* Display HZ16X8 */
	{0},

	ENABLE,								/* Display HZ16X8 */
	{
		"XZG:   INV",
		12,   102,
		Black,White,
		White
	},

	DISABLE,							/* Display Picture */
	{0}
};

/******************************************************************************/
block_attr_Result block_Result_PCY = {
	DISABLE,							/* Display HZ16X8 */
	{0},

	ENABLE,								/* Display HZ16X8 */
	{
		"PCY:  INV",
		12,   118,
		Black,White,
		White
	},

	DISABLE,							/* Display Picture */
	{0}
};

/******************************************************************************/
block_attr_Result block_Result_Arrow = {
	ENABLE,								/* Interface Result rect */
	{
		89, 140,
		37, 19,
		BACKCOLOR_CONTENT_BACK
	},

	DISABLE,							/* Display HZ16X8 */
	{0},

	DISABLE,							/* Display Picture */
	{0},
};

///******************************************************************************/
//block_attr_Result block_Result_Item_2 = {
//	DISABLE,							/* Display Picture */
//	{0},
//
//	ENABLE,									/*Display HZ16X8*/
//	{
//		"Item",
//		9,   34,
//		Black,White,
//		BACKCOLOR_CONTENT_BAR
//	},
//
//	DISABLE,							/* Display Picture */
//	{0},
//};

/******************************************************************************/
block_attr_Result block_Result_Left_arrow = {
	ENABLE,								/* Interface Result rect */
	{
		7,   52,
		114, 85,
		White
	},

	DISABLE,							/* Display Picture */
	{0},

	ENABLE,								/* Display Picture */
	{
		gImage_Left_arrow,
		5, 142,
		35, 15
	},
};


/******************************************************************************/
block_attr_Result* UI_WindowBlocksAttrArray_Result_2[] = {/* Window: Result entry */
		&block_Result_Arrow,
		&block_Result_Left_arrow,
		&block_Result_WCG,
		&block_Result_EDG,
		&block_Result_UZI,
};

/******************************************************************************/
void UI_Draw_block_Result_2(block_attr_Result* block);

/******************************************************************************/
uint8 Interface_Result_2(uint16 KeyCode)
{
	Page_Flag = 1;
	Exti_lock = DISABLE;
	Interface_Key = 2;
	UI_WindowBlocks_Result_2 = sizeof(UI_WindowBlocksAttrArray_Result_2) >> 2;
	UI_Draw_Window_Result_2(UI_WindowBlocks_Result_2);
	Exti_lock = ENABLE;
	while(!key_state);
	UI_state = UI_STATE_KEY_STATE;
	return UI_STATE_RERUN;
}

/******************************************************************************/
void UI_Draw_Window_Result_2(uint16 blockNum)
{
	uint8 blockIndex = 0;					/* Draw blocks one by one */
	for (blockIndex = 0; blockIndex < blockNum; blockIndex++)
	{
		UI_Draw_block_Result_2(UI_WindowBlocksAttrArray_Result_2[blockIndex]);
	}
}

/******************************************************************************/
void UI_Draw_block_Result_2(block_attr_Result* block)
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

	if (block->pic_enabled)					/* 2. Draw picture */
	{
		DisplayDriver_DrawPic(block->pic_attr.offsetX,
				block->pic_attr.offsetY, block->pic_attr.width,
				block->pic_attr.height,block->pic_attr.src);
	}
	key_state = 0;
	Display_Time = 1;
}
