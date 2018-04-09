/*
 * Comm.h
 *
 *  Created on: 2016��8��17��
 *      Author: Administrator
 */

#ifndef _COMM_FIFO_H_
#define _COMM_FIFO_H_

/******************************************************************************/
#include "stm32f10x.h"
#include "comDef.h"

/******************************************************************************/
/* ���漶�� */
#define FIFO_Index    (3)
/*��֡�����ݳ���*/
#define FIFO_LenMax      (2000)

/******************************************************************************/
/* ��֡FIFO���� */
typedef struct
{
	/* ���ݻ�����:   ���漶��        ����������   */
	uint8 Buffer[FIFO_Index][FIFO_LenMax];
	/* ����λ�õ�ַ ����λ�õ�ַ */
	uint8 PutPos, GetPos;
	/* ����������  ���������໺������ �����־λ */
	uint8  Size, Free, Error;
} FrameFIFOTypedef;

/******************************************************************************/
#define StructRecvDataLenth (14)

/******************************************************************************/
typedef struct
{	
	/* �����͵�Data���� */
	uint16 len;
	/* �����͵�Data */
	uint8 Data[FIFO_LenMax];
} DataTypedef;

/******************************************************************************/
extern FrameFIFOTypedef RxDataFIFO;
extern DataTypedef TxData;
extern uint8 Comm_RequestTX;

/******************************************************************************/
extern void Comm_FIFO_Init(FrameFIFOTypedef *FIFO);
extern ErrorStatus Comm_FIFO_RxDataPut(DataTypedef *SourceData,
							  FrameFIFOTypedef *DestinationData);
extern ErrorStatus Comm_FIFO_RxDataGet(FrameFIFOTypedef *SourceData,
							  DataTypedef *DestinationData);
extern ErrorStatus Comm_FIFO_TxDataPut(DataTypedef *SourceData,
							  FrameFIFOTypedef *DestinationData);
extern ErrorStatus Comm_FIFO_TxDataGet(FrameFIFOTypedef *SourceData,
							  DataTypedef *DestinationData);

#endif /* SOURCE_MODULE_COMM_COMM_FIFO_H_ */
