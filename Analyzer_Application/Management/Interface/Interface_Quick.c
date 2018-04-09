/*
 * Interface_Quick.c
 *
 *  Created on: 2018��2��24��
 *      Author: Administrator
 ******************************************************************************/
#include "Interface_Quick.h"
#include "RotationMotor.h"
#include "ScanMotorDriver.h"

/******************************************************************************/
uint8 Cup_Exist = 1;
extern uint8 QRCode_existed;
extern const unsigned char gImage_Left_arrow[1050];

/******************************************************************************/
block_attr_Quick block_Quick_1 = {
	ENABLE,								/* Interface Quick rect */
	{
		0,   20,
		128, 140,
		BACKCOLOR_CONTENT_BACK
	},

	DISABLE,							/* Display HZ16X8 */
	{0},

	DISABLE,							/* Display Picture */
	{0},
};

/******************************************************************************/
block_attr_Quick block_Quick_2 = {
	ENABLE,								/* Interface Quick rect */
	{
		7,   62,
		114, 78,
		White
	},


	ENABLE,								/* Display HZ16X8 */
	{
		"Scan the QR",
		15,   82,
		Black,White,
		White
	},

	DISABLE,							/* Display Picture */
	{0},
};

/******************************************************************************/
block_attr_Quick block_Quick_3 = {
	ENABLE,								/* Interface Quick rect */
	{
		7,   40,
		114, 22,
		BACKCOLOR_CONTENT_BAR
	},
	ENABLE,								/*Display HZ16X8*/
	{
		"Notice",
		9,   43,
		Red,BACKCOLOR_CONTENT_BAR,
		BACKCOLOR_CONTENT_BAR
	},

	DISABLE,							/* Display Picture */
	{0},
};

/******************************************************************************/
block_attr_Quick block_Quick_4 = {
	DISABLE,							/*Interface Quick rect */
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"code",
		52,   100,
		Black,White,
		White
	},

//	DISABLE,							/*Interface Quick rect */
//	{0},

	ENABLE,								/* Display Picture */
	{
		gImage_Left_arrow,
		5, 142,
		35, 15
	}
};

/******************************************************************************/
block_attr_Quick block_Quick_5 = {
	ENABLE,								/*Interface Quick rect */
	{
		7,   62,
		114, 78,
		White
	},

	ENABLE,								/*Display HZ16X8*/
	{
		"Invalid QR",
		15,   70,
		Red,White,
		White
	},

	DISABLE,							/* Display Picture */
	{0},
};

/******************************************************************************/
block_attr_Quick block_Quick_6 = {
	DISABLE,							/*Interface Quick rect */
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"code",
		52,   88,
		Red,White,
		White
	},

	DISABLE,							/* Display Picture */
	{0},
};

/******************************************************************************/
block_attr_Quick block_Quick_7 = {
	DISABLE,							/*Interface Quick rect */
	{0},

	ENABLE,								/*Display HZ16X8*/
	{
		"Scan Again!",
		20,   108,
		Red,White,
		White
	},

	DISABLE,							/* Display Picture */
	{0},
};


/******************************************************************************/
block_attr_Quick* UI_WindowBlocksAttrArray_Quick[] = {/* Window: Standard entry */
		&block_Quick_1,
		&block_Quick_2,
		&block_Quick_3,
		&block_Quick_4,
};

/******************************************************************************/
block_attr_Quick* UI_WindowBlocksAttrArray_Quick_font[] = {/* Window: Standard entry */
		&block_Quick_5,
		&block_Quick_6,
		&block_Quick_7,
};

/******************************************************************************/
void UI_Draw_Block_Quick(block_attr_Quick* block);
void UI_Draw_Block_Quick_Font(block_attr_Quick* block);

/******************************************************************************/
uint8 Interface_Quick(uint16 KeyCode)
{
	Key_control = 2;
	Exti_lock = DISABLE;
	QRCode_existed = 0;
	Interface_Key = 3;
	UI_WindowBlocks_Quick = sizeof(UI_WindowBlocksAttrArray_Quick) >> 2;
	UI_Draw_Window_Quick(UI_WindowBlocks_Quick);
	QRCode_Trigger_Enabled();
	Exti_lock = ENABLE;
	while(!QRCode_existed)
	{
		UI_state = UI_STATE_INSERT_CUP;
		if(Key_control == 1)
		{
			UI_state = UI_STATE_KEY_STATE;
			QRCode_existed = 1;
		}
		key_state_confirm = 0;
	}
	Exti_lock = DISABLE;
	return UI_STATE_RERUN;
}

/******************************************************************************/
void UI_Draw_Window_Quick(uint16 blockNum)
{
	uint8 blockIndex = 0;					/* Draw blocks one by one */
	for (blockIndex = 0; blockIndex < blockNum; blockIndex++)
	{
		UI_Draw_Block_Quick(UI_WindowBlocksAttrArray_Quick[blockIndex]);
	}
}

/******************************************************************************/
void UI_Draw_Block_Quick(block_attr_Quick* block)
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

	if (block->pic_enabled)						/* 2. Draw picture */
	{
		DisplayDriver_DrawPic(block->pic_attr.offsetX,
				block->pic_attr.offsetY, block->pic_attr.width,
				block->pic_attr.height,block->pic_attr.src);
	}

	key_state = DISABLE;
	Display_Time = 1;
}

/******************************************************************************/
uint8 Interface_Quick_font(uint16 KeyCode)
{
	UI_WindowBlocks_Quick_font = sizeof(UI_WindowBlocksAttrArray_Quick_font) >> 2;
	UI_Draw_Window_Quick_font(UI_WindowBlocks_Quick_font);
	UI_state = UI_STATE_QUICK;
	Exti_lock = ENABLE;
	Delay_ms(2000);
	return UI_STATE_RERUN;
}

/******************************************************************************/
void UI_Draw_Window_Quick_font(uint16 blockNum)
{
	uint8 blockIndex = 0;					/* Draw blocks one by one */
	for (blockIndex = 0; blockIndex < blockNum; blockIndex++)
	{
		UI_Draw_Block_Quick_Font(UI_WindowBlocksAttrArray_Quick_font[blockIndex]);
	}
}

/******************************************************************************/
void UI_Draw_Block_Quick_Font(block_attr_Quick* block)
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
	key_state = DISABLE;
	Display_Time = 1;
}
