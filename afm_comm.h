#ifndef AFM_COMM
#define AFM_COMM

#include <stdio.h>

#define VERSION "0.0.3"
#define AUTHOR "LI Yuan"
#define LAST_REVISED "2008.04.22"

#define PI 3.14159265358979
#define SERVER_ADDR "192.168.1.199"
#define SERVER_PORT 7000

#define MAX_LOOP 30
#define ERROR_EPS 20


typedef struct command
{
	unsigned short cmd;//命令字
	unsigned short para1;//参数 1
	unsigned short para2;//参数 2
	unsigned short para3;//参数 3
	unsigned short para4;//参数 4
	unsigned short para5;//参数 5
} COMMAND;//总共12个字节

typedef struct response
{
	unsigned short cmd;//命令字
	short para1;//参数 1
	short para2;//参数 2
	short para3;//参数 3
	short para4;//参数 4
	short para5;//参数 5
} RESPONSE;//总共12个字节

#define MOTOR_STEP_FORWARD	0x10
#define MOTOR_STEP_BACKWARD 	0x11
#define MOTOR_AUTO_FORWARD	0x12
#define MOTOR_AUTO_BACKWARD	0x17
#define MOTOR_GET_STEPS		0x13
#define MOTOR_FAST_FORWARD	0x14
#define MOTOR_FAST_BACKWARD 	0x15
#define MOTOR_STOP		0x16

#define LASER_ON		0x40
#define LASER_OFF		0x41
#define GET_LASER_POS		0x42
#define SET_MODE		0x43
#define SET_VOLTAGE		0x44

#define SET_HV_ON		0x49
#define SET_HV_OFF		0x4A
/*------------------------------------------------------*/
#define CMD_FREQ_SCAN		0x45
#define SET_WORKING_FREQ	0x46
#define SET_FREQ_AMPLITUDE	0x47
#define CMD_FREQ_STOP		0x48
#define SET_FREQ_RANGE		0x4B
/*------------------------------------------------------*/
#define CMD_FORCE_CURVE	0x2A
#define CMD_LINESCAN_START	0x20
#define CMD_FAST_SCAN		0x22
#define CMD_SCAN_WHOLE		0x23
#define CMD_SCAN_STOP		0x21
#define SET_SCAN_RANGE		0x24
#define SET_SCAN_PIXEL		0x25
#define SET_FEEDBACK_MODE	0x26
#define SET_PID_PARA		0x27
#define SET_PID_OTHER		0x28
#define SET_WORKING_POINT	0x29

/*------------------------------------------------------*/
#define LINE_SCAN_ONCE 		0x01
#define LINE_SCAN_NOMAL 	0x02
#define FAST_SCAN 		0x03
#define NOMAL_SCAN 		0x04
#define STOP 			0x00
#define LASER_POS 		0x06
/*------------------------------------------------------*/
#define SET_LCD_PIC 0x30
#define SET_LCD_BEEP 0x31
#define SET_CORRECTION_PARA 0x32
/*------------------------------------------------------*/
#define EXPERT_MODE_LED_ON	0xE0
#define EXPERT_MODE_LED_OFF	0xE1
#define EXPERT_MODE_AD		0xf0
#define EXPERT_MODE_DA		0xf1
#define EXPERT_MODE_DDS_FREQ	0xf2
#define EXPERT_MODE_DDS_AMPL	0xf3
#define EXPERT_MODE_DDS_STOP	0xf4
#define EXPERT_MODE_PID		0xf5
#define EXPERT_MODE_LASER_POS	0xf6
#define EXPERT_MODE_IO_OUT1	0xf7
#define EXPERT_MODE_IO_OUT8	0xf8
#define EXPERT_MODE_IO_OUT16	0xf9
#define EXPERT_MODE_DDS_PHASE	0xfA
#define EXPERT_MODE_RS232_OUT 0xE2

/* test only. */
#define EXPERT_MODE_TEST 0xfB
#define EXPERT_MODE_GET_ERR 0xFC
#define EXPERT_MODE_PID_ERR 0xFD

/* test only. */

/*------------------------------------------------------*/
#define MODE_STM		0x00
#define MODE_AFM_CONTACT	0x01
#define MODE_AFM_TAPPING	0x02
#define MODE_SHEAR_FORCE	0x03

/*------------------------------------------------------*/

#ifdef _DEBUG_
#define DEBUG0( msg ) puts(msg);
#define DEBUG1(msg, val) printf(msg, val);
#define DEBUG2(msg, val1, val2) printf(msg, val1, val2);
#else
#define DEBUG0( msg )
#define DEBUG1(msg, val)
#define DEBUG2(msg, val1, val2)
#endif

#endif
