#include "Comm.h"
#include <Strsafe.h>

#define SYNCRONIZATION_BYTE 0x22

bool CommSendBytes( CSerial* pComm, unsigned char cmdCode, unsigned char* buffer, unsigned char len ) {
	CommPacketHeader packetHeader;
	CommPacketFooter packetFooter;

	packetHeader.start_byte_ = SYNCRONIZATION_BYTE;
	packetHeader.cmd_code_ = cmdCode;
	packetHeader.length_ = len;

	DWORD sended;

	if( pComm->Write( &packetHeader.start_byte_, 1, &sended ) != ERROR_SUCCESS ) {
		return false;
	}

	if( pComm->Write( &packetHeader.cmd_code_, 1, &sended ) != ERROR_SUCCESS ) {
		return false;
	}

	if( pComm->Write( &packetHeader.length_, 1, &sended ) != ERROR_SUCCESS ) {
		return false;
	}

	if( pComm->Write( buffer, len, &sended ) != ERROR_SUCCESS ) {
		return false;
	}

	packetFooter.check_sum_ = 0;
	for ( unsigned int i=0; i<packetHeader.length_; i++ ) {
		packetFooter.check_sum_ |= buffer[i];
	}
	packetFooter.end_byte = SYNCRONIZATION_BYTE;

	if( pComm->Write( &packetFooter.check_sum_, 1, &sended ) != ERROR_SUCCESS ) {
		return false;
	}

	if( pComm->Write( &packetFooter.end_byte, 1, &sended ) != ERROR_SUCCESS ) {
		return false;
	}

	return true;
}

bool CommWaitSynchroByte( CSerial* pComm ) {
	unsigned char byteVal;

	while ( 1 ) {
		if( pComm->Read( &byteVal, 1 ) == ERROR_SUCCESS ) {
			if( byteVal == SYNCRONIZATION_BYTE ) {
				return true;
			}
		}
	}

	return false;
}

bool CommReceiveBytes( CSerial* pComm, unsigned char* dstCmdCode, unsigned char* dstBuffer, unsigned char* dstLength ) {
	if( !CommWaitSynchroByte( pComm ) ) {
		return false;
	}

	//read buffer length
	if( pComm->Read( dstCmdCode, 1 ) != ERROR_SUCCESS ) {
		return false;
	}

	//read buffer length
	if( pComm->Read( dstLength, 1 ) != ERROR_SUCCESS ) {
		return false;
	}

	//read buffer
	if( pComm->Read( dstBuffer, *dstLength ) != ERROR_SUCCESS ) {
		return false;
	}

	CommPacketFooter packetFooter;
	//read check sum
	if( pComm->Read( &packetFooter.check_sum_, 1 ) != ERROR_SUCCESS ) {
		return false;
	}

	//calculate checksum from received data
	unsigned char sum = 0;
	for ( unsigned int i=0; i<*dstLength; i++ ) {
		sum |= dstBuffer[i];
	}

	if( sum != packetFooter.check_sum_ ) {
		return false;
	}

	//read end of packet
	if( pComm->Read( &packetFooter.end_byte, 1 ) != ERROR_SUCCESS ) {
		return false;
	}

	if( packetFooter.end_byte != SYNCRONIZATION_BYTE ) {
		return false;
	}

	return true;
}

void CommBuildMsgSetServoAngles( unsigned char theta1, unsigned char theta2, unsigned char theta3, unsigned char* dstBuffer, unsigned char* dstlength ) {
	sprintf( (char*)dstBuffer, "%u %u %u", (unsigned int)theta1, (unsigned int)theta2, (unsigned int)theta3 );
	*dstlength = strlen( (char*)dstBuffer );
}