/*
 * Interface_Result_2.h
 *
 *  Created on: 2018��3��16��
 *      Author: Administrator
 */

#ifndef MANAGEMENT_INTERFACE_INTERFACE_RESULT_2_H_
#define MANAGEMENT_INTERFACE_INTERFACE_RESULT_2_H_

/******************************************************************************/
#include "Interface_main.h"
#include "comDef.h"

/******************************************************************************/
extern uint8 UI_state;
extern uint8 key_state;
extern uint8 Exti_lock;
extern uint8 Key_control;
extern uint8 Interface_Key;

/******************************************************************************/
typedef struct {
	uint8 rect_enabled; 				/* Support rectangular or not */
	rect_attr rect_attr;				/* Rectangular attribute */
	uint8 char_enabled;					/* Support char or not */
	char_attr char_attr;				/* char attribute */
	uint8 pic_enabled;     				/* Support picture or not */
	pic_attr pic_attr;
} block_attr_Result;

/******************************************************************************/
extern void UI_Draw_Window_Result_2(uint16 blockNum);

#endif /* MANAGEMENT_INTERFACE_INTERFACE_RESULT_2_H_ */
