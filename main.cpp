#include <stdio.h>
#include <stdlib.h>
#include "DeltaManipulator.h"
#include "Serial.h"
#include <windows.h>
#include <tchar.h>
#include "Comm.h"
#include <stdio.h>
#include <stdio.h>

#define PI 3.1415926
#define USER_CMD_MOVE_TO_POSITION	1 
#define USER_CMD_SET_MOTOR_ANGLES	2
#define USER_CMD_MOVE_ON_LINE		3
#define USER_CMD_MOUSE_CONTROL		4
#define USER_CMD_MOVE_ON_CIRCLE		5

const float Zmin = -300.0f;
const float Zmax = -120.0f;
const float ThetaMin = 5.0f;
const float ThetaMax = 75.0f;

bool loadRobotParamsFromFile(	const char* filename,
								double& e,     // end effector
								double& f,     // base
								double& re,
								double& rf ) {
	FILE* file = fopen( filename, "r" );
	if(!file) {
		printf("konfiguraciis faili ver moiZebna!\n");
		return false;
	}

	int res = fscanf( file, "%lg %lg %lg %lg", &e, &f, &re, &rf );
	if( res != 4 ) {
		fclose( file );
		return false;
	}
	fclose( file );
	return true;
}

void CmdMoveToPosition( DeltaManipulator* pRobot, CSerial* pComm ) {
	float x, y, z;
	double theta1, theta2, theta3;
	printf("SemoitaneT sasurveli poziciis koordinati!\n");
	printf("x: ");
	scanf( "%f", &x );

	printf("y: ");
	scanf( "%f", &y );

	printf("z: ");
	scanf( "%f", &z );

	if( pRobot->Inverse( vec3f(x,y,z), theta1, theta2, theta3 ) ) {
		printf("Secdoma: miTiTebul pozicias roboti ver wvdeba!\n");
	} else {
		unsigned char buffer[256];
		unsigned char contentLength;
		CommBuildMsgSetServoAngles( (unsigned char)theta1, (unsigned char)theta2, (unsigned char)theta3, buffer, &contentLength );
		//printf((const char*)buffer);
		if( !CommSendBytes( pComm, CMD_CODE_SET_SERVO_ANGLES, buffer, contentLength ) ) {
			printf("Secdoma monacemebis gadacemisas!\n");
		}
	}
}

void CmdSetMotorAngles( DeltaManipulator* pRobot, CSerial* pComm ) {
	unsigned int angle1, angle2, angle3;
	printf("Zravebis kuTxeebi unda SeitanoT [10, 180] diapazonSi!\n");

	printf("pirveli Zravis kuTxe : ");
	scanf( "%u", &angle1 );
	if( angle1 < 10 ) {
		printf("Secdom!\n");
		return;
	}
	
	printf("meore Zravis kuTxe : ");
	scanf( "%u", &angle2 );
	if( angle2 < 10 ) {
		printf("Secdom!\n");
		return;
	}

	printf("mesame Zravis kuTxe : ");
	scanf( "%u", &angle3 );
	if( angle3 < 10 ) {
		printf("Secdom!\n");
		return;
	}

	unsigned char buffer[256];
	unsigned char contentLength;
	CommBuildMsgSetServoAngles( angle1, angle2, angle3, buffer, &contentLength );
	//printf((const char*)buffer);
	if( !CommSendBytes( pComm, CMD_CODE_SET_SERVO_ANGLES, buffer, contentLength ) ) {
		printf("Secdoma monacemebis gadacemisas!\n");
	}
}

void CmdMoveOnLine( DeltaManipulator* pRobot, CSerial* pComm ) {
	float x1, y1, z1, x2, y2, z2;
	double theta1, theta2, theta3;
	printf("SemoitaneT monakveTis pirveli wertili!\n");
	printf("x: ");
	scanf( "%f", &x1 );

	printf("y: ");
	scanf( "%f", &y1 );

	printf("z: ");
	scanf( "%f", &z1 );

	printf("SemoitaneT monakveTis meore wertili!\n");
	printf("x: ");
	scanf( "%f", &x2 );

	printf("y: ");
	scanf( "%f", &y2 );

	printf("z: ");
	scanf( "%f", &z2 );

	unsigned int steps;
	printf("bijebis raodenoba: ");
	scanf( "%u", &steps );

	unsigned int timeout;
	printf("dayovneba TiToeul bijze(ms): ");
	scanf( "%u", &timeout );

	float deltaX = x2 - x1;
	float deltaY = y2 - y1;
	float deltaZ = z2 - z1;
	
	for ( unsigned int i=0; i<steps; i++ ) {
		float p = i/(float)(steps-1);
		float x = x1 + deltaX*p;
		float y = y2 + deltaY*p;
		float z = z2 + deltaZ*p;

		if( pRobot->Inverse( vec3f(x,y,z), theta1, theta2, theta3 ) ) {
			unsigned char buffer[256];
			unsigned char contentLength;
			CommBuildMsgSetServoAngles( (unsigned char)theta1, (unsigned char)theta2, (unsigned char)theta3, buffer, &contentLength );
			//printf((const char*)buffer);
			if( !CommSendBytes( pComm, CMD_CODE_SET_SERVO_ANGLES, buffer, contentLength ) ) {
				printf("Secdoma monacemebis gadacemisas!\n");
			}
		} else {
			printf("Secdoma: pozicia gacda samuSao areals!\n");
			break;
		}

		if( i == 0 ){
			Sleep( 2000 );
		} else {
			Sleep( timeout );
		}
	}

	printf("\n");
}

void CmdMoveOnCircle( DeltaManipulator* pRobot, CSerial* pComm ) {
	float r;
	float x, y, z;
	double theta1, theta2, theta3;
	printf("r: ");
	scanf( "%f", &r );

	printf("z: ");
	scanf( "%f", &z );

	float speed = 0.2;

	float angle = 0;
	while( 1 ) {
		x = cos(angle)*r;
		y = sin(angle)*r;

		if( GetAsyncKeyState(VK_ESCAPE) & 0x8000 ) {
			break;
		} else if( GetAsyncKeyState(VK_UP) & 0x8000 ) {
			if( speed + 0.01 < 0.8 ) {
				speed += 0.01;
			}
		} else if( GetAsyncKeyState(VK_DOWN) & 0x8000 ) {
			if( speed - 0.01 > 0.01 ) {
				speed -= 0.01;
			}
		}

		double theta1, theta2, theta3;

		if( pRobot->Inverse( vec3f(x,y,z), theta1, theta2, theta3 ) ) {

			if( (theta1 < ThetaMin) || (theta2 < ThetaMin) || (theta3 < ThetaMin) ||
				(theta1 > ThetaMax) || (theta2 > ThetaMax) || (theta3 > ThetaMax)) {
				system( "cls" );
				printf( "<< DIAPAZONIS GARETAA >>\n" );
				continue;
			}

			unsigned char buffer[256];
			unsigned char contentLength;
			CommBuildMsgSetServoAngles( (unsigned char)theta1, (unsigned char)theta2, (unsigned char)theta3, buffer, &contentLength );
			//printf((const char*)buffer);
			if( !CommSendBytes( pComm, CMD_CODE_SET_SERVO_ANGLES, buffer, contentLength ) ) {
				printf("Secdoma monacemebis gadacemisas!\n");
			}

			system( "cls" );
			printf("x=%.1f\ny=%.1f\nz=%.1f\n\ntheta 1: %02u\ntheta 2: %02u\ntheta 3: %02u\n", x, y, z, (unsigned int)theta1, (unsigned int)theta2, (unsigned int)theta3 );
		} else {
			system( "cls" );
			printf("<< %.1f\t %.1f\t %.1f >>\n", x, y, z );
			//break;
		}
		Sleep( 1.0/speed );

		angle += PI*0.1;
		if( angle > 2.0*PI ) {
			angle -= 2.0*PI;
		}
	}
}

void CmdControlWithMouse( DeltaManipulator* pRobot, CSerial* pComm ) {
	int dwWidth = GetSystemMetrics(SM_CXSCREEN);
	int dwHeight = GetSystemMetrics(SM_CYSCREEN);


	float Z = -200.0;
	while( 1 ) {
		POINT mousePos;
		GetCursorPos( &mousePos );

		float x = -(dwWidth/2 - mousePos.x)*0.5;
		float y = (dwHeight/2 - mousePos.y)*0.5;

		if( GetAsyncKeyState(VK_ESCAPE) & 0x8000 ) {
			break;
		} else if( GetAsyncKeyState(VK_UP) & 0x8000 ) {
			if( Z + 2.0 < Zmax ) {
				Z += 2.0;
			}
		} else if( GetAsyncKeyState(VK_DOWN) & 0x8000 ) {
			if( Z - 2.0 > Zmin ) {
				Z -= 2.0;
			}
		}

		float z = Z;
		double theta1, theta2, theta3;

		if( pRobot->Inverse( vec3f(x,y,z), theta1, theta2, theta3 ) ) {

			if( (theta1 < ThetaMin) || (theta2 < ThetaMin) || (theta3 < ThetaMin) ||
				(theta1 > ThetaMax) || (theta2 > ThetaMax) || (theta3 > ThetaMax)) {
				system( "cls" );
				printf( "<< DIAPAZONIS GARETAA >>\n" );
				continue;
			}

			unsigned char buffer[256];
			unsigned char contentLength;
			CommBuildMsgSetServoAngles( (unsigned char)theta1, (unsigned char)theta2, (unsigned char)theta3, buffer, &contentLength );
			//printf((const char*)buffer);
			if( !CommSendBytes( pComm, CMD_CODE_SET_SERVO_ANGLES, buffer, contentLength ) ) {
				printf("Secdoma monacemebis gadacemisas!\n");
			}

			system( "cls" );
			printf("x=%.1f\ny=%.1f\nz=%.1f\n\ntheta 1: %02u\ntheta 2: %02u\ntheta 3: %02u\n", x, y, z, (unsigned int)theta1, (unsigned int)theta2, (unsigned int)theta3 );
		} else {
			system( "cls" );
			printf("<< %.1f\t %.1f\t %.1f >>\n", x, y, z );
			//break;
		}

		
		

		Sleep( 20 );
	}

	printf("\n");
}

void printHelp() {

	printf( "brZanebebis CamonaTvali:\n" );
	printf( "\t[%u] miTiTebul poziciaze misvla\n", USER_CMD_MOVE_TO_POSITION );
	printf( "\t[%u] Zravebis kuTxeebis dayeneba\n", USER_CMD_SET_MOTOR_ANGLES );
	printf( "\t[%u] xazze moZraoba\n", USER_CMD_MOVE_ON_LINE );
	printf( "\t[%u] kontroli mausis saSualebiT\n", USER_CMD_MOUSE_CONTROL );
	printf( "\t[%u] swewirze moZraoba\n", USER_CMD_MOVE_ON_CIRCLE );
}

int main( int argc, char** argv ) {
	DeltaManipulator robot;

	double robotParams[4];
	if( !loadRobotParamsFromFile( "DeltaRobotParams.dlt", robotParams[0], robotParams[1], robotParams[2], robotParams[3] ) ) {
		printf( "Secdoma dafiqsirda delta robotis parametrebis chatvirtvisas!\n" );
		system("pause");
		return 0;
	}

	robot.Confogure( robotParams[0], robotParams[1], robotParams[2], robotParams[3] );

	CSerial comm;
	comm.Setup(CSerial::EBaud9600);
	unsigned int portNumber;
	printf("COM portis nomeri: ");
	scanf( "%u", &portNumber );

	TCHAR serialPortName[8];
	_stprintf( serialPortName, _T("COM%u"), portNumber );
	if( comm.Open ( serialPortName ) != ERROR_SUCCESS ) {
		printf("Secdoma COM portis gaxsnisas!\n");
		system("pause");
		return 0;
	}

	bool done = false;
	unsigned int cmdCode;
	char cmdCodeStr[64];
	do 
	{

		printf("akrifeT brZanebis kodi, 'exit' an 'help': ");
		scanf("%s", cmdCodeStr );
		if( cmdCodeStr[0] == 'h' || cmdCodeStr[0] == 'H' ) {
			printHelp();
		} else if( cmdCodeStr[0] == 'e' || cmdCodeStr[0] == 'E' ) {
			done = true;
		} else {
			sscanf( cmdCodeStr, "%u", &cmdCode );

			switch ( cmdCode )
			{
			case USER_CMD_MOVE_TO_POSITION:
				CmdMoveToPosition( &robot, &comm );
				break;

			case USER_CMD_SET_MOTOR_ANGLES:
				CmdSetMotorAngles( &robot, &comm );
				break;

			case USER_CMD_MOVE_ON_LINE:
				CmdMoveOnLine( &robot, &comm );
				break;

			case USER_CMD_MOUSE_CONTROL:
				CmdControlWithMouse( &robot, &comm );
				break;

			case USER_CMD_MOVE_ON_CIRCLE:		
				CmdMoveOnCircle( &robot, &comm );
				break;
			}
		}
	} while ( !done );
	
	comm.Close();
	return 0;
}