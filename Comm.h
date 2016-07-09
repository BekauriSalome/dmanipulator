#ifndef _COMM_H_
#define _COMM_H_
#include "Serial.h"

#define CMD_CODE_SET_SERVO_ANGLES	1
#define CMD_CODE_SET_SERVO_ANGLE	2

struct CommPacketHeader {
	unsigned char start_byte_;
	unsigned char cmd_code_;
	unsigned char length_;
};

struct CommPacketFooter {
	unsigned char check_sum_;
	unsigned char end_byte;
};

bool CommSendBytes( CSerial* pComm, unsigned char cmdCode, unsigned char* buffer, unsigned char len );
bool CommReceiveBytes( CSerial* pComm, unsigned char* dstCmdCode, unsigned char* dstBuffer, unsigned char* dstLength );

void CommBuildMsgSetServoAngles( unsigned char theta1, unsigned char theta2, unsigned char theta3, unsigned char* dstBuffer, unsigned char* dstlength );

#endif