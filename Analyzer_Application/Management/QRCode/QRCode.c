/*
 * QRCode.c
 *
 *  Created on: 2018年3月7日
 *      Author: Administrator
 */

/******************************************************************************/
#include "QRCode.h"
#include "DisplayDriver.h"

/******************************************************************************/
void QRCode_Init(void)
{
	QRCode_GPIO_Init();

	QRCode_Uart_Init();
}

/******************************************************************************/
void QRCode_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin =  QRCODE_TRIG_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(QRCODE_TRIG_PORT, &GPIO_InitStructure);

	QRCode_Trigger_Disabled();
}

/******************************************************************************/
void QRCode_Uart_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	QRCode_Uart_GPIO_Init();					/* Initialize GPIOs */
	QRCode_Uart_NVIC_Configuration();			/* Enable interrupt */
	QRCode_Uart_Config();						/* Configure parameters */
}

/******************************************************************************/
void QRCode_Uart_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/*Enable interrupt*/
	NVIC_InitStructure.NVIC_IRQChannel = QRCODE_USART_IRQN;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/******************************************************************************/
void QRCode_Uart_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*Configure USART2 Tx as alternate function push-pull*/
	GPIO_InitStructure.GPIO_Pin = QRCODE_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(QRCODE_TX_PORT, &GPIO_InitStructure);

	/*Configure USART2 Rx as input floating*/
	GPIO_InitStructure.GPIO_Pin = QRCODE_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(QRCODE_RX_PORT, &GPIO_InitStructure);
}

/******************************************************************************/
void QRCode_Uart_Config(void)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Configure */
	USART_Init(QRCODE_USART, &USART_InitStructure);

	/* Enable Receive and Transmit interrupts */
	USART_ITConfig(QRCODE_USART, USART_IT_RXNE, ENABLE);

	/* Enable */
	USART_Cmd(QRCODE_USART, ENABLE);

	/* To avoid the first byte missing issue */
	USART_ClearFlag(QRCODE_USART, USART_FLAG_TC);
}

/******************************************************************************/
void QRCode_Trigger_Enabled(void)
{
	GPIO_ResetBits(QRCODE_TRIG_PORT, QRCODE_TRIG_PIN);
}

/******************************************************************************/
void QRCode_Trigger_Disabled(void)
{
	GPIO_SetBits(QRCODE_TRIG_PORT, QRCODE_TRIG_PIN);
}

/******************************************************************************/
void QRCode_Restart(void)
{
	QRCode_Trigger_Disabled();
	Delay_ms_SW(1000);
	QRCode_Trigger_Enabled();
}

/******************************************************************************/
void USART1_IRQHandler(void)
{
	uint8 value;

	if(USART_GetITStatus(QRCODE_USART, USART_IT_RXNE) != RESET)
	{
		value=USART_ReceiveData(QRCODE_USART);

		if (value != '\r')
		{
			QRCode_Buffer[QRCode_count++] = value;
		}
		else
		{
			QRCode_received = 1;
			QRCode_existed = 1;

		}
	}
	USART_ClearITPendingBit(QRCODE_USART, USART_IT_RXNE);	//清除USART1上的中断标志位
}

/******************************************************************************/
void QRCode_Received(void)
{
/* Scan interface: receive data from QR code scanner */
	if (QRCode_received == 1)
	{
		QRCode_received = 0;

		if (QRCode_Identify())						/* Decode */
		{
			DisplayDriver_Text16_B(35, 75, Red, White, "Valid QR");
			DisplayDriver_Text16_B(45, 95, Red, White, "Code!");
			QRCode_existed = 1;

			/* If meet rules, switch to next interface */
			UI_state = UI_STATE_MAIN_WINDOW;
		}
		else
		{
			DisplayDriver_Text16_B(27, 85, Red, White, "Invalid QR");
			DisplayDriver_Text16_B(47, 105, Red, White, "Code!");
//			QRCode_Restart();
			QRCode_existed = 1;
			UI_state = UI_STATE_TESTING;
		}

		QRCode_count = 0;							/* Clear size */
	}
}

/******************************************************************************/
uint8 QRCode_Identify(void)
{
	uint8 status = 0;
	uint16 crcCalc = 0;
	uint16 crcRec = 0;
	uint16 count = 0;

//	memset(&QRCode_data, 0, sizeof(QRCODE_STRUCT));

	/* Calculate CRC */
	crcCalc = Common_CalculateCRC(&QRCode_Buffer[2], QRCode_count - 2, 0xFFFF, 0x0000);

	/* Get CRC from command */
	crcRec = (QRCode_Buffer[1] << 8) | QRCode_Buffer[0];

	if (crcRec == crcCalc)
	{
		status = 1;
		return status;
	}
	QRCode_existed = 0;								/* Wrong QR code */
	return status;
}
