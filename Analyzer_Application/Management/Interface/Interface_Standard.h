/*
 * Interface_Standard.h
 *
 *  Created on: 2018Äê2ÔÂ24ÈÕ
 *      Author: Administrator
 */

#ifndef MANAGEMENT_INTERFACE_INTERFACE_STANDARD_H_
#define MANAGEMENT_INTERFACE_INTERFACE_STANDARD_H_

/******************************************************************************/
#include "Interface_main.h"

/******************************************************************************/
extern uint8 key_state;
extern uint8 Exti_lock;
extern uint8 Key_control;
uint16 UI_WindowBlocks_Standard = 0;
extern uint8  Cup_Count;

/******************************************************************************/
typedef struct {
	uint8 rect_enabled; 				/* Support rectangular or not */
	rect_attr rect_attr;				/* Rectangular attribute */
	uint8 char_enabled;					/* Support char or not */
	char_attr char_attr;				/* char attribute */
} block_attr_Standard;

/******************************************************************************/
typedef struct {
	uint8 char1_enabled;					/* Support char or not */
	char_attr char1_attr;				/* char attribute */
	uint8 char2_enabled;					/* Support char or not */
	char_attr char2_attr;				/* char attribute */
} block_font_Standard;

/******************************************************************************/
extern void UI_Draw_Window_Standard(uint16 blockNum);

#endif /* MANAGEMENT_INTERFACE_INTERFACE_STANDARD_H_ */
