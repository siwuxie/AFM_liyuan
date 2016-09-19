#include "hardware.h"

#define	ADDR_DDS				0x304
#define	ADDR_CTR				0x305
#define	ADDR_COUNTER		0x307

#define	BIT_50RST				0x00
#define	BIT_COUNTERRST	0x01
#define	BIT_LED					0x02
#define	BIT_52SCLK			0x03
#define	BIT_52CS				0x04
#define	BIT_52DIN				0x05

void dds_Out(double freq)
{
	const unsigned int DIV = 0xffffffff;
	const double Ratio = 40000000.0 / DIV;
	unsigned int Data = (unsigned int) (freq / Ratio);
	unsigned char* pData = (unsigned char*)(&Data);
	unsigned char control_word;
	control_word = inb_p(ADDR_CTR);
	//reset DDS
	setBitToZero(control_word , BIT_50RST);
	outb_p(control_word, ADDR_CTR);
	setBitToOne(control_word , BIT_50RST);
	outb_p(control_word, ADDR_CTR);
	setBitToZero(control_word, BIT_50RST);
	outb_p(control_word, ADDR_CTR);
	//Write FRQ
	outb_p(0x00, ADDR_DDS);
	outb_p(pData[3], ADDR_DDS);
	outb_p(pData[2], ADDR_DDS);
	outb_p(pData[1], ADDR_DDS);
	outb_p(pData[0], ADDR_DDS);
	// Update Frq
	inb_p(ADDR_DDS);	 
}
void dds_Stop()
{
	unsigned char control_word = inb_p(ADDR_CTR);
	setBitToOne(control_word , BIT_50RST);
	outb_p(control_word, ADDR_CTR);
}

void setLedOn()
{
	unsigned char control_word = inb_p(ADDR_CTR);
	setBitToOne(control_word, BIT_LED);
	outb_p(control_word, ADDR_CTR);
}

void setLedOff()
{
	unsigned char control_word = inb_p(ADDR_CTR);
	setBitToZero(control_word, BIT_LED);
	outb_p(control_word, ADDR_CTR);
}

void setWaveAmplitude(unsigned char amplitude)
{
	int i;
	unsigned char control_word = inb_p(ADDR_CTR);
	//Set CS signal of the DAC to low level
	setBitToZero(control_word, BIT_52CS);
	outb_p(control_word, ADDR_CTR);

	unsigned short data = 0x2300 | amplitude;
	
	//write data 
	for(i = 15; i >= 0; i--)
	{
		//put a bit to DIN	
		((data & (1 << i)) == 0) ? setBitToZero(control_word, BIT_52DIN) : setBitToOne(control_word, BIT_52DIN);
		outb_p(control_word, ADDR_CTR);
		
		//Generate the SCLK signal
		setBitToZero(control_word, BIT_52SCLK);
		outb_p(control_word, ADDR_CTR);
		setBitToOne(control_word, BIT_52SCLK);
		outb_p(control_word, ADDR_CTR);
		setBitToZero(control_word, BIT_52SCLK);
		outb_p(control_word, ADDR_CTR);
	}
	//update DAC
	setBitToOne(control_word, BIT_52CS);
	outb_p(control_word, ADDR_CTR);
	//DEBUG1("setWaveAmplitude(%d)\n", (int) amplitude);
}

void setPhase(unsigned short phase)
{
	unsigned char* p = (unsigned char*)(&phase);
	//Reset Counter
	unsigned char control_word = inb_p(ADDR_CTR);
	
	setBitToOne(control_word, BIT_COUNTERRST);
	outb_p(control_word, ADDR_CTR);
	setBitToZero(control_word, BIT_COUNTERRST);
	outb_p(control_word, ADDR_CTR);
	setBitToOne(control_word, BIT_COUNTERRST);
	outb_p(control_word, ADDR_CTR);

	//write the end count (the counter starts from 0x0000);
	outb_p(p[1], ADDR_COUNTER);
	outb_p(p[0], ADDR_COUNTER);
}





