#include "hardware.h"

#define PM511P_ADDR	0x100
#define IO_PA (PM511P_ADDR + 0x08)
#define IO_PB (PM511P_ADDR + 0x09)
#define IO_PC (PM511P_ADDR + 0x0A)
#define IO_CONTROL_WORD (PM511P_ADDR + 0x0B) 
#define	ADDR_MOTOR	(PM511P_ADDR + 0x09)

static unsigned char buf_IO_PB = 0;//, buf_IO_PA = 0, buf_IO_PC = 0;
const char g_motorSteps[8] = {0x08, 0x09, 0x01, 0x05, 0x04, 0x06, 0x02, 0x0A};
static unsigned short g_steps = 32000;

void motor_backward_one_step()
{
	int step;
	g_steps --;
	step = g_steps % 8;
	buf_IO_PB &= 0xF0;
	buf_IO_PB |= g_motorSteps[step];
	outb_p(buf_IO_PB, ADDR_MOTOR);
};

void motor_forward_one_step()
{
	int step;
	g_steps ++;
	step = g_steps % 8;
	buf_IO_PB &= 0xF0;
	buf_IO_PB |= g_motorSteps[step];
	outb_p(buf_IO_PB, ADDR_MOTOR);
};

void motor_stop()
{
	buf_IO_PB &= 0xF0;
	outb_p(buf_IO_PB, ADDR_MOTOR);
}

short motor_get_steps()
{
	return g_steps;
}


void setLaserOn()
{
	setBitToOne(buf_IO_PB, 6); 
	outb_p(buf_IO_PB, ADDR_MOTOR);
}
void setLaserOff()
{
	setBitToZero(buf_IO_PB, 6);
	outb_p(buf_IO_PB, ADDR_MOTOR);
}
void IO_Out(unsigned char channel, unsigned char value)
{
}
void IO_Out8(unsigned char channel, unsigned char value)
{
}
void IO_Out16(unsigned short value)
{
}
void setHighVoltageOn()
{
}
void setHighVoltageOff()
{
}
