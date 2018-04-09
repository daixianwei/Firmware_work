/*
 * Interface_Setting.c
 *
 *  Created on: 2018��2��24��
 *      Author: Administrator
 */

/******************************************************************************/
#include "Interface_Setting.h"

/******************************************************************************/
block_attr_Setting block_Setting_1 = {
	ENABLE,								/*Interface Setting rect*/
	{
		0,   20,
		128, 140,
		BACKCOLOR_CONTENT_BACK
	},

	DISABLE,							/*Display HZ16X8*/
	{0},

	DISABLE,
	{0},
};

/******************************************************************************/
block_attr_Setting block_Setting_2 = {
	DISABLE,							/*Interface Setting rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"System Time",
		20,   73,
		White,BACKCOLOR_CONTENT_BAR,
		White
	},

	ENABLE,								/*Display Picture*/
	{
		gImage_PIC_System_Time,
		41, 28,
		45, 45
	},
};

/******************************************************************************/
block_attr_Setting block_Setting_3 = {
	DISABLE,							/*Interface Setting rect*/
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"About Machine",
		15,   137,
		White,BACKCOLOR_CONTENT_BAR,
		White
	},

	ENABLE,								/*Display Picture*/
	{
		gImage_PIC_About_Machine,
		41,  92,
		45,  45
	},
};

/******************************************************************************/
block_attr_Setting* UI_WindowBlocksAttrArray_Setting[] = {/* Window: Standard entry */
		&block_Setting_1,
		&block_Setting_2,
		&block_Setting_3,
};

/******************************************************************************/
void UI_Draw_Block_Setting(block_attr_Setting* block);

/******************************************************************************/
uint8 Interface_Setting(uint16 KeyCode)
{
	Exti_lock = DISABLE;
	UI_WindowBlocks_Setting = sizeof(UI_WindowBlocksAttrArray_Setting) >> 2;
	UI_Draw_Window_Setting(UI_WindowBlocks_Setting);
	Delay_ms(3000);
	Exti_lock = ENABLE;
	UI_state = UI_STATE_MAIN_WINDOW;
	key_state_confirm = DISABLE;
	Interface_Key = DISABLE;
	key_state = ENABLE;
	return UI_STATE_RERUN;
}

/******************************************************************************/
void UI_Draw_Window_Setting(uint16 blockNum)
{
	uint8 blockIndex = 0;					/* Draw blocks one by one */
	for (blockIndex = 0; blockIndex < blockNum; blockIndex++)
	{
		UI_Draw_Block_Setting(UI_WindowBlocksAttrArray_Setting[blockIndex]);
	}
	key_state = ENABLE;
}

/******************************************************************************/
void UI_Draw_Block_Setting(block_attr_Setting* block)
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

	if (block->pic_enabled)				/* 2. Draw picture */
	{
		DisplayDriver_DrawPic(block->pic_attr.offsetX,
				block->pic_attr.offsetY, block->pic_attr.width,
				block->pic_attr.height,block->pic_attr.src);
	}
	Display_Time = 1;
}
