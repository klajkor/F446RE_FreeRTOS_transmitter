/*
 * protocol.h
 *
 *  Created on: 2021 March 13.
 *      Author: Robert_Klajko
 */

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

#include <stdio.h>
#include <stdint.h>

#define FRAME_START_BYTE_1 		( 0x55 )
#define FRAME_START_BYTE_2 		( 0x56 )
#define FRAME_STOP_BYTE_1 		( 0x0D )
#define FRAME_STOP_BYTE_2 		( 0x0A )
#define FRAME_CRC_BYTE_POS		( 9 )
#define CRC_DEFAULT_VALUE		( 0xFF )
#define CRC_POLYNOM				( 0x31 )

typedef union unionFloatUint8_t
{
	float f;
	uint8_t u8[4];
} unionFloatUint8_t;


typedef struct
{
	char Header[3];
	char Payload[21];
} structMessageBuffer_t;

typedef struct structUARTmessage_t
{
	uint8_t Start_Byte_1;
	uint8_t Start_Byte_2;
	uint8_t Device_ID;
	uint8_t Cmd_ID;
	unionFloatUint8_t Data1;
	int8_t Data2;
	uint8_t Crc_Byte;
	uint8_t Stop_Byte_1;
	uint8_t Stop_Byte_2;
} structUARTmessage_t;

typedef uint8_t messageFrame_t[sizeof(structUARTmessage_t)];

typedef union unionUARTmessage_t
{
	structUARTmessage_t structMessage;
	messageFrame_t arrayMessage;
} unionUARTmessage_t;

typedef enum protocolRetVal_enum {
    protocolRetVal_OK = 0,
    protocolRetVal_NOK,
    protocolRetVal_End
} protocolRetVal_enum;

typedef enum Frame_Cmd_enum {
	eFRAME_CMD_BTN_STATUS 	= 0x42,
	eFRAME_CMD_LED_STATUS 	= 0x4C,
	eFRAME_CMD_ADC_VALUE  	= 0x41,
	eFRAME_CMD_ADC_RAW		= 0x52
} Frame_Cmd_enum;

typedef enum Frame_Btn_Status_enum {
	eBTN_STATUS_OFF = 0x30,
	eBTN_STATUS_ON  = 0x31
} Frame_Btn_Status_enum;

typedef enum Frame_Led_Status_enum {
	eLED_STATUS_OFF = 0x30,
	eLED_STATUS_ON  = 0x31
} Frame_Led_Status_enum;

typedef enum Frame_Device_Id_enum {
	eDEVICE_ID_F446RE 	= 0x21,
	eDEVICE_ID_WB55		= 0x22
} Frame_Device_Id_enum;

typedef enum messageValidateRetVal_enum {
    validate_OK = 0,
	validate_MsgLenError,
	validate_StartByteError,
	validate_StopByteError,
	validate_CRCError
} messageValidateRetVal_enum;


protocolRetVal_enum buildFrameToSend_old(uint8_t frameCmdID, unionFloatUint8_t frameData1, int8_t frameData2, uint8_t *pFrame);
protocolRetVal_enum buildFrameToSend(uint8_t frameCmdID, unionFloatUint8_t frameData1, int8_t frameData2, messageFrame_t pFrame);

messageValidateRetVal_enum messageValidate(messageFrame_t msgFrame);

uint8_t gencrc8(uint8_t *data, uint8_t len);

#endif /* INC_PROTOCOL_H_ */
