/*******************************************************************************
 * File name: HostComm.h
 * Author: Hanson Liu
 * Mail: han_liu_zju@sina.com
 * Date: 2014.12 ~ Now
 ******************************************************************************/
#ifndef __MANAGEMENT_HOSTCOMM_HOSTCOMM_H_
#define __MANAGEMENT_HOSTCOMM_HOSTCOMM_H_

/******************************************************************************/
#include "main.h"
#include "Interface_main.h"

/******************************************************************************/
#define HOSTCOMM_USART (USART2)
#define HOSTCOMM_USART_IRQN (USART2_IRQn)
#define HOSTCOMM_TX_PORT (GPIOA)
#define HOSTCOMM_TX_PIN (GPIO_Pin_2)
#define HOSTCOMM_RX_PORT (GPIOA)
#define HOSTCOMM_RX_PIN (GPIO_Pin_3)
#define PASSWORD_MAX_LENGTH (100)
#define TESTNAME_MAX_LENGTH (100)
#define UI_DATA_TRANSFER_ON (1)
#define UI_DATA_TRANSFER_OFF (0)

#define DEVICE_AREA_SIZE           (50)
#define FLASH_CALI_STATUS_ADDR     ((0X08000000 + 0x80000) - DEVICE_AREA_SIZE)

/******************************************************************************/
/* Size information */
#define SIZE_REC_BUFFER              (3*1024)
#define SIZE_CMD_BUFFER              (2*1024 + 100)
#define SIZE_RESP_BUFFER             (100)
#define SIZE_HEAD_LEN                (3)
#define SIZE_LEN_HEAD_CMD_CRC        (6)
#define SIZE_PASSWORD                (14)
#define SIZE_FILE                    (4)
#define SIZE_PACK_NUM                (2)
#define SIZE_HEAD_TAIL               (2)

/* Application version major and minor */
#define APP_REVISION                 (1)
#define APP_VERSION_MINOR            (0)
#define APP_VERSION_MAJOR            (1)

/* Error response mask */
#define CMD_ERROR_MASK               (0x80)
#define CRC_ERROR_MASK               (0x40)

/******************************************************************************/
/* Offset in command and response array */
enum offset {
	OFFSET_HEADER = 0,
	OFFSET_LEN_LO = 1,
	OFFSET_LEN_HI = 2,
	OFFSET_CMD_TYPE = 3,
	OFFSET_CMD_CODE = 4,
	OFFSET_CMD_DATA = 5,
};

enum offsetRX {
	OFFSET_CMD_TYPE_RX = 0,
	OFFSET_CMD_CODE_RX = 1,
	OFFSET_CMD_DATA_RX = 2,
};


/* Command type */
enum cmdType {
	CMD_TYPE_NONE = 0,
	CMD_TYPE_APP = 0XFE,
	CMD_TYPE_BL = 0xFF,
};

/* Command code */
enum cmdCode {
	CMD_CODE_STATUS = 0X00,
	CMD_CODE_BL_SYSINFO = 0X01,
	CMD_CODE_BL_VERIFY_KEY,
	CMD_CODE_BL_FLASH,
	CMD_CODE_BL_VERIFY_CRC,
	CMD_CODE_BL_LAUNCH_APP,
	CMD_CODE_APP_SYSINFO = 0X10,
	CMD_CODE_APP_LAUNCH_BL,
	CMD_CODE_APP_READ_IDCARD,
	CMD_CODE_APP_WRITE_IDCARD,
	CMD_CODE_APP_SET_TIME,
	CMD_CODE_APP_SET_MODE = 0x15,
	CMD_CODE_APP_SET_MFG,
	CMD_CODE_APP_SET_LANGUAGE,
	CMD_CODE_APP_SET_PRINTER,
	CMD_CODE_APP_SET_OUT_FAB,
	CMD_CODE_APP_START_STOP_READ_RECORD = 0x1A,
	CMD_CODE_APP_READ_RECORDS_PERDATE,
	CMD_CODE_APP_READ_RECORD,
	CMD_CODE_APP_READ_RECORD_LASTONE,
	CMD_CODE_APP_SET_BLE_NAME,
	CMD_CODE_APP_SET_INSTITUTE = 31,
	CMD_CODE_APP_CLEAR_IDCARD,
	CMD_CODE_APP_SET_OUTPUT_DATA,
	CMD_CODE_APP_READ_RESISTOR,
	CMD_CODE_APP_WRITE_RESISTOR,

	CMD_CODE_APP_SET_CALC_PARAMETERS = 40,
};

/* Bin file attribute */
struct binFileAttr {
	uint32 fileSize;
	uint16 packageNum;
	uint16 packageSize;
};

typedef struct {
	uint8 order;
	uint8 id[PASSWORD_MAX_LENGTH];
	uint8 name[PASSWORD_MAX_LENGTH];
	uint8 testName[TESTNAME_MAX_LENGTH];
	uint8 testUnit[TESTNAME_MAX_LENGTH];
	uint8 testCase;
	uint8 sign;
	float result;
	float result2;
	uint8 hour;
	uint8 min;
	uint8 sec;
} HOSTCOMM_REPORT_ATTR;

/******************************************************************************/
extern uint8 cmdBuffer[SIZE_CMD_BUFFER];
extern uint8 HostComm_RecBufAvailable;
extern uint16 HostComm_RecBufSize;

/******************************************************************************/
extern void HostComm_Init(void);
extern void HostComm_Process(void);
extern void HostComm_Send_LIS(uint8 *data);
extern void HostComm_Send_Char(uint8 data);
extern void HostComm_Send_String(uint8 *strPtr);
extern void HostComm_Send(USART_TypeDef* USARTx, uint8 *Data,...);
char *itoa(int32 value, char *string, int radix);
uint16 HostComm_CalculateCRC(uint8 * message,uint32 length,
		uint16 remainder, uint16 xorMask);
extern void HostComm_SendThrUSB(uint16 len, uint8 *srcPtr);
extern void HostComm_Report_ReadRecords_LastOne(void);
extern void HostComm_Report_ReadRecords(HOSTCOMM_REPORT_ATTR data);

#endif /* __MANAGEMENT_HOSTCOMM_HOSTCOMM_H_ */
