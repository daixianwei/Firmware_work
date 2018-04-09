/*******************************************************************************
 * File name: stm32f10x_it.h
 * Author: Hanson Liu
 * Mail: han_liu_zju@sina.com
 * Date: 2014.12 ~ Now
 ******************************************************************************/
#ifndef ISR_STM32F10x_IT_H
#define ISR_STM32F10x_IT_H

/******************************************************************************/
#include "stm32f10x.h"
#include "comDef.h"

/******************************************************************************/
char tbuf1[10] = {0};
uint16 Count_Down = 0;
uint8 Exti_lock = DISABLE;
uint16 hours = 11,minutes =11,seconds = 01; 		//ʱ,��,��
extern uint8 ScanMotorDriver_InBasePosition;
uint16 se_count = 0,keyupCnt = 0,key_holdon_ms = 0;
uint8 key_fall_flag = 0,short_key_flag = 0,doubleClick = 0,
		keyUpFlag = 0,long_key_flag = 0;

/******************************************************************************/
#define Time_ms    (180)

/******************************************************************************/
void SVC_Handler(void);
void NMI_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void BusFault_Handler(void);
void DebugMon_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void UsageFault_Handler(void);
					 
#endif /* ISR_STM32F10x_IT_H */
