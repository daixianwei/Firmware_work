#include "Comm_FIFO.h"
#include "comDef.h"

/******************************************************************************/
/* ����/��������FIFO */
FrameFIFOTypedef RxDataFIFO;
DataTypedef TxData;
uint8 Comm_RequestTX = 0;

/******************************************************************************/
void Comm_FIFO_Init(FrameFIFOTypedef *FIFO)
{
	uint8 i;
	
	FIFO->Error = RESET;
	FIFO->Free = FIFO_Index;
	FIFO->GetPos = RESET;
	FIFO->PutPos = RESET;
	FIFO->Size = FIFO_Index;
	
	/* FIFO->Buffer���� */
	for(i = 0; i < FIFO_Index; i++)
	{
		memset(&(FIFO->Buffer[i]), 0, FIFO_LenMax);
	}
}

/******************************************************************************/
ErrorStatus Comm_FIFO_RxDataPut(DataTypedef *SourceData,FrameFIFOTypedef *DestinationData)
{	
	/* û�п��໺��,�޷�����,���ش��� */
	if(DestinationData->Free == RESET)
	{
		return ERROR;
	}
	
	/* ���ݳ��ȵ�FIFO */
	memcpy((uint8*)&(DestinationData->Buffer[DestinationData->PutPos]),SourceData, SourceData->len+2);
	
	/* ���е�ַ����,�հ׻������ */
	DestinationData->PutPos++;
	DestinationData->Free--;
	
	/* ��ַ��������ĩβ,��ͷ�ٿ�ʼ */
	if(DestinationData->PutPos >= DestinationData->Size)
	{
		DestinationData->PutPos = RESET;
	}
	
	/* �������гɹ� */
	return SUCCESS; 
}

/******************************************************************************/
ErrorStatus Comm_FIFO_RxDataGet(FrameFIFOTypedef *SourceData, 
							        DataTypedef *DestinationData)
{
	uint8* pData;
	uint8* pDest;
	uint16 i;
	
	/* û�����ݿɶ�,���ش��� */
	if(SourceData->Free == SourceData->Size)
	{
		return ERROR;
	}
	
	/* ��ȡԴ���ݺ�Ŀ�������׵�ַ */
	pData = (uint8*)&(SourceData->Buffer[SourceData->GetPos]);
	pDest = (uint8*)(DestinationData);
	
	for(i = 0; i<(*(uint16*)&(SourceData->Buffer[SourceData->GetPos][0]))+2; i++)
	{
		*pDest = *pData;
		pDest++;
		pData++;
	}

	/* ���е�ַ����,�հ׻����ͷ� */
	SourceData->GetPos++;
	SourceData->Free++;
	
	/* ��ַ��������ĩβ,��ͷ�ٿ�ʼ */
	if(SourceData->GetPos >= SourceData->Size)
	{
		SourceData->GetPos = RESET;
	}
	
	/* ���ݳ��гɹ� */
	return SUCCESS;
}

/******************************************************************************/
ErrorStatus Comm_FIFO_TxDataPut(DataTypedef *SourceData,
							        FrameFIFOTypedef *DestinationData)
{	
	uint8* pDest;
	uint8* pData;
	uint8 i;
	
	/* û�п��໺��,�޷�����,���ش��� */
	if(DestinationData->Free == RESET)
	{
		return ERROR;
	}
	
	/* ��ȡԴ���ݺ�Ŀ�������׵�ַ */
	pData = (uint8*)SourceData;
	pDest = (uint8*)&(DestinationData->Buffer[DestinationData->PutPos]);
	
	/* Դ���ݺ�Ŀ������һһ��Ӧ,ֱ��Copy */
	for(i = 0; i < SourceData->len+2; i++)
	{
		*pDest = *pData;
		pDest++;
		pData++;	
	}
	
	/* ���е�ַ����,�հ׻������ */
	DestinationData->PutPos++;
	DestinationData->Free--;
	
	/* ��ַ��������ĩβ,��ͷ�ٿ�ʼ */
	if(DestinationData->PutPos >= DestinationData->Size)
	{
		DestinationData->PutPos = RESET;
	}
	
	/* �������гɹ� */	
	return SUCCESS;
}

/******************************************************************************/
ErrorStatus Comm_FIFO_TxDataGet(FrameFIFOTypedef *SourceData, 
							        DataTypedef *DestinationData)
{
	/* û�����ݿɶ�,���ش��� */
	if(SourceData->Free == SourceData->Size)
	{
		return ERROR;
	}
	
	/* ��FIFO Copy Data�������� */
	memcpy(DestinationData,&SourceData->Buffer[SourceData->GetPos][0],(*(uint16*)&(SourceData->Buffer[SourceData->GetPos][0]))+2);
	
	/* ���е�ַ����,�հ׻����ͷ� */
	SourceData->GetPos++;
	SourceData->Free++;
	
	/* ��ַ��������ĩβ,��ͷ�ٿ�ʼ */
	if(SourceData->GetPos >= SourceData->Size)
	{
		SourceData->GetPos = RESET;
	}
	
	/* ���ݳ��гɹ� */
	return SUCCESS;
}
