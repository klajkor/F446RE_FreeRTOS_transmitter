/*
 * protocol.c
 *
 *  Created on: 2021 March 13.
 *      Author: Robert_Klajko
 */
#include <stdio.h>
#include <string.h>
#include "protocol.h"

Frame_Device_Id_enum This_Device_ID = eDEVICE_ID_F446RE;

protocolRetVal_enum buildFrameToSend_old(uint8_t frameCmdID, unionFloatUint8_t frameData, uint8_t *pFrame)
{
	uint16_t i;
	uint8_t crc_byte;
	protocolRetVal_enum retval;

	if ( NULL != pFrame)
	{
		memset((char *)pFrame, 0, sizeof(uint8_t));

		pFrame[0]=FRAME_START_BYTE_1;
		pFrame[1]=FRAME_START_BYTE_2;
		pFrame[2]=This_Device_ID;
		pFrame[3]=frameCmdID;
		for(i=0;i<4;i++)
		{
			pFrame[i+4]=frameData.u8[i];
		}
		pFrame[8]=FRAME_EXT1;
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

protocolRetVal_enum buildFrameToSend(uint8_t frameCmdID, unionFloatUint8_t frameData, messageFrame_t pFrame)
{
	uint16_t i;
	uint8_t crc_byte;
	protocolRetVal_enum retval;
	unionUARTmessage_t messageFrame;


	if ( NULL != pFrame)
	{
		memset((char *)pFrame, 0, sizeof(messageFrame_t)*sizeof(uint8_t));

		messageFrame.structMessage.Start_Byte_1=FRAME_START_BYTE_1;
		messageFrame.structMessage.Start_Byte_2=FRAME_START_BYTE_2;
		messageFrame.structMessage.Device_ID=This_Device_ID;
		messageFrame.structMessage.Cmd_ID=frameCmdID;
		for(i=0;i<4;i++)
		{
			messageFrame.structMessage.Data.u8[i]=frameData.u8[i];
		}
		messageFrame.structMessage.Ext_Byte_1=FRAME_EXT1;
		crc_byte=gencrc8(messageFrame.arrayMessage, 9);
		messageFrame.structMessage.Crc_Byte=crc_byte;
		messageFrame.structMessage.Stop_Byte_1=FRAME_STOP_BYTE_1;
		messageFrame.structMessage.Stop_Byte_2=FRAME_STOP_BYTE_2;
		for(i=0;i<12;i++)
		{
			pFrame[i]=messageFrame.arrayMessage[i];
		}
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

