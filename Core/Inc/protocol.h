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
#define FRAME_CMD_BTN_STATUS 	( 0x31 )
#define FRAME_CMD_LED_STATUS 	( 0x32 )
#define FRAME_CMD_ADC_VALUE 	( 0x33 )
#define FRAME_CMD_ADC_RAW 		( 0x34 )
#define FRAME_STATUS_OFF 		( 0x30 )
#define FRAME_STATUS_ON 		( 0x31 )
#define FRAME_EXT1 				( 0x00 )
#define FRAME_EXT2 				( 0x00 )
#define FRAME_STOP_BYTE_1 		( 0x0D )
#define FRAME_STOP_BYTE_2 		( 0x0A )
#define CRC_DEFAULT_VALUE		( 0xFF )
#define CRC_POLYNOM				( 0x31 )

typedef union {
	float f;
	uint8_t u8[4];
} unionFloatUint8_t;


typedef struct
{
	char Header[3];
	char Payload[21];
} structMessageBuffer_t;

typedef uint8_t messageFrame_t[12];

typedef enum {
    protocolRetVal_NOK = 0,
    protocolRetVal_OK,
    protocolRetVal_End
} protocolRetVal_enum;


protocolRetVal_enum buildFrameToSend(uint8_t frameCmdID, unionFloatUint8_t frameData, uint8_t *pFrame);


uint8_t gencrc8(uint8_t *data, uint8_t len);

#endif /* INC_PROTOCOL_H_ */
