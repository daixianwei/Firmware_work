/*
 * Interface_Testing.h
 *
 *  Created on: 2018��2��26��
 *      Author: Administrator
 */

#ifndef MANAGEMENT_INTERFACE_INTERFACE_TESTING_H_
#define MANAGEMENT_INTERFACE_INTERFACE_TESTING_H_

/******************************************************************************/
#include "Interface_main.h"

/******************************************************************************/
#define  NO_CUP        		   		(0)
#define  BOUNDARY_VALUE        		(500)
#define SIGNALSAMPLE_MAX_COUNT 		512

/******************************************************************************/
extern uint8 Exti_lock;
extern uint8 key_state;
extern uint8 Key_control;
uint16 UI_WindowBlocks_Testing = 0;

/******************************************************************************/
typedef struct {
	uint8 rect_enabled; 				/* Support rectangular or not */
	rect_attr rect_attr;				/* Rectangular attribute */
	uint8 char_enabled;					/* Support char or not */
	char_attr char_attr;				/* char attribute */
} block_attr_Testing;

/******************************************************************************/
extern uint16 Get_Start_Postion(void);
extern void Acquisition_Signal(void);
extern uint16 Get_sampleBuffer_Max_Value(void);
extern void Get_sampleBuffer_Start_Position(void);
extern void UI_Draw_Window_Testing(uint16 blockNum);
extern void SignalSample_Moving_Average_Data(uint16 *Data,uint16 Length,uint16 Period);

#endif /* MANAGEMENT_INTERFACE_INTERFACE_TESTING_H_ */
