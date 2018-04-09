/*
 * Interface_Start.h
 *
 *  Created on: 2018Äê2ÔÂ26ÈÕ
 *      Author: Administrator
 */

#ifndef MANAGEMENT_INTERFACE_INTERFACE_START_H_
#define MANAGEMENT_INTERFACE_INTERFACE_START_H_

/******************************************************************************/
#include "Interface_main.h"

/******************************************************************************/
extern uint8 key_state;
extern uint8 Exti_lock;
extern uint8 Key_control;
uint16 UI_WindowBlocks_Start = 0;
uint8  Cup_Count = 8;

/******************************************************************************/
typedef struct {
	uint8 rect_enabled; 				/* Support rectangular or not */
	rect_attr rect_attr;				/* Rectangular attribute */
	uint8 char_enabled;					/* Support char or not */
	char_attr char_attr;				/* char attribute */
} block_attr_Start;

/******************************************************************************/
extern void UI_Draw_Window_Start(uint16 blockNum);

#endif /* MANAGEMENT_INTERFACE_INTERFACE_START_H_ */


