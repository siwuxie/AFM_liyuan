#include "hardware.h"

#define	ADDR_DDS				0x304
#define	ADDR_DO_L				0x302
#define	ADDR_DO_H				0x303
#define	ADDR_CTR				0x305
#define	ADDR_MOTOR			0x306
#define	ADDR_COUNTER		0x307


static unsigned char io_l, io_h;
void IO_Out(unsigned char channel, unsigned char value)
{
	if(channel < 8)
	{
		(value & 0x01) ? setBitToOne(io_l, channel) : setBitToZero(io_l, channel);
		outb_p(io_l, ADDR_DO_L);
	}
	else if(channel < 16)
	{
		channel -= 8;
		(value & 0x01) ? setBitToOne(io_h, channel) : setBitToZero(io_h, channel);
		outb_p(io_h, ADDR_DO_H);
	}
}

void IO_Out8(unsigned char channel, unsigned char value)
{
	if(channel == 0)
	{
		io_l = value;
		outb_p(io_l, ADDR_DO_L);
	}
	else if(channel == 1)
	{
		io_h = value;
		outb_p(io_h, ADDR_DO_H);
	}
}

void IO_Out16(unsigned short value)
{
	
	unsigned char pValue[2];
	memcpy(pValue, &value, sizeof(unsigned short));
	io_l = pValue[0];
	outb_p(io_l, ADDR_DO_L);
	io_h = pValue[1];
	outb_p(io_h, ADDR_DO_H);
}
const char g_motorSteps[8] = {0x08, 0x09, 0x01, 0x05, 0x04, 0x06, 0x02, 0x0A};
static short g_steps = 0;

void motor_backward_one_step( void )
{
	int step;
	g_steps ++;
	step = g_steps + 30000;
	step %= 8;
	outb_p(g_motorSteps[step], ADDR_MOTOR);
};

void motor_forward_one_step( void )
{
	int step;
	g_steps --;
	step = g_steps + 30000;
	step %= 8;
	outb_p(g_motorSteps[step], ADDR_MOTOR);
};

void motor_stop( void )
{
	outb_p(0x00, ADDR_MOTOR);
}

short motor_get_steps( void )
{
	return g_steps;
}

void setLaserOn( void )
{
	setBitToOne(io_l, 1); 
	outb_p(io_l, ADDR_DO_L);
}
void setLaserOff( void )
{
	setBitToZero(io_l, 1);
	outb_p(io_l, ADDR_DO_L);
}
void setHighVoltageOn( void )
{
	setBitToOne(io_l, 2); 
	outb_p(io_l, ADDR_DO_L);
}
void setHighVoltageOff( void )
{
	setBitToZero(io_l, 2);
	outb_p(io_l, ADDR_DO_L);
}

