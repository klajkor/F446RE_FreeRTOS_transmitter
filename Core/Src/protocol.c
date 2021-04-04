/*
 * protocol.c
 *
 *  Created on: 2021 March 13.
 *      Author: Robert_Klajko
 */
#include <stdio.h>
#include <string.h>
#include "protocol.h"


protocolRetVal_enum buildFrameToSend(uint8_t frameCmdID, unionFloatUint8_t frameData, uint8_t *pFrame)
{
	uint16_t i;
	uint8_t crc_byte;
	protocolRetVal_enum retval;

	if ( NULL != pFrame)
	{
		memset((char *)pFrame, 0, sizeof(uint8_t));

		pFrame[0]=FRAME_START_BYTE_1;
		pFrame[1]=FRAME_START_BYTE_2;
		pFrame[2]=frameCmdID;
		for(i=0;i<4;i++)
		{
			pFrame[i+3]=frameData.u8[i];
		}
		pFrame[7]=FRAME_EXT1;
		pFrame[8]=FRAME_EXT2;
		crc_byte=gencrc8(pFrame, 9);
		pFrame[9]=crc_byte;
		pFrame[10]=FRAME_STOP_BYTE_1;
		pFrame[11]=FRAME_STOP_BYTE_2;
		retval = protocolRetVal_OK;
	}
	else
	{
		retval = protocolRetVal_NOK;
	}

	return retval;
}

uint8_t gencrc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = CRC_DEFAULT_VALUE;
    uint8_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ CRC_POLYNOM);
            else
                crc <<= 1;
        }
    }
    return crc;
}

