/*
 * QRCode.h
 *
 *  Created on: 2018Äê3ÔÂ7ÈÕ
 *      Author: Administrator
 */

#ifndef MANAGEMENT_QRCODE_QRCODE_H_
#define MANAGEMENT_QRCODE_QRCODE_H_

/******************************************************************************/
#include "comDef.h"
#include "stm32f10x.h"
#include "Interface_main.h"

/******************************************************************************/
#define QRCODE_BUFFER_SIZE           (400)

#define QRCODE_TRIG_PORT      		 (GPIOA)
#define QRCODE_TRIG_PIN         	 (GPIO_Pin_3)

#define QRCODE_TX_PORT      		 (GPIOA)
#define QRCODE_TX_PIN         		 (GPIO_Pin_9)

#define QRCODE_RX_PORT     			 (GPIOA)
#define QRCODE_RX_PIN        		 (GPIO_Pin_10)

#define QRCODE_USART          		 (USART1)
#define QRCODE_USART_IRQN          	 (USART1_IRQn)

/******************************************************************************/
uint16 QRCode_count = 0;
uint8 QRCode_existed = 0;
uint8 QRCode_received = 0;
uint8 QRCode_Buffer[QRCODE_BUFFER_SIZE];

/******************************************************************************/
extern void QRCode_Init(void);
extern void QRCode_Restart(void);
extern void QRCode_Received(void);
extern void QRCode_GPIO_Init(void);
extern void QRCode_Uart_Init(void);
extern uint8 QRCode_Identify(void);
extern void QRCode_Uart_Config(void);
extern void QRCode_Uart_GPIO_Init(void);
extern void QRCode_Trigger_Enabled(void);
extern void QRCode_Trigger_Disabled(void);
extern void QRCode_Uart_NVIC_Configuration(void);

#endif /* MANAGEMENT_QRCODE_QRCODE_H_ */
