/* 只被 hardware.c 调用，请不要在项目中直接使用本文件
 *
 * PM 511P AD/DA 采集卡是 12 位 AD/DA 采集卡，
 * AD 单一通道的最高采集速度约为 80 KHz。
 * DA 的最高输出速度约为 200 KHz,（DA建立时间 5 us）。
 *
 */
#include "hardware.h"

#define PM511P_ADDR	0x100
#define IO_CONTROL_WORD (PM511P_ADDR + 0x0B) 
#define AD_CHANNEL	0x00
	
#define CHANNEL_DELAY 10
#define AD_CONVERSION_DELAY 9
#define DA_CONVERSION_DELAY 9

const unsigned char AD_LASER_A = 0x02;
const unsigned char AD_LASER_B = 0x01;
const unsigned char AD_LASER_C = 0x04;
const unsigned char AD_LASER_D = 0x03;
const unsigned char AD_LASER_SUM = 0x00;
const unsigned char AD_OSAP = 0x07;
const unsigned char AD_FICTION = 0x06; // not real
const unsigned char AD_PHASE = 0x0B;
const unsigned char AD_ERROR = 0x06;
const unsigned char AD_STM = 0x0C;
const unsigned char DA_X = 0x00;
const unsigned char DA_Y = 0x01;
const unsigned char DA_Z = 0x02;

void write_DA( unsigned char channel, unsigned short value )
{
	outb_p( channel, PM511P_ADDR + 4 );	/*设置 D/A 通道*/
	outw_p( value >> 4, PM511P_ADDR + 6 );	/*启动 D/A 输出*/
	udelay(DA_CONVERSION_DELAY);//waiting for D/A conversion finish
}

short read_AD(unsigned char channel)
{
	short value;
	outb( channel , PM511P_ADDR );	/*设置 A/D 采样通道*/
	udelay(CHANNEL_DELAY);//waiting for setting the channel of A/D
	inb( PM511P_ADDR );	/*启动 A/D 转换*/
	
#ifdef _CONST_DELAY_
	udelay(AD_CONVERSION_DELAY);//waiting for A/D conversion finish
#else
	while( inb( PM511P_ADDR + 1 ) &  0x01);	/*判断 A/D 转换是否完成*/
#endif	
	
	value = inw( PM511P_ADDR + 2 ) & 0xfff;
	return (value - 2048) * 16;
}

short read_AD_N( unsigned char channel, int samplingNum )
{
	short i, value;
	int total_value = 0;
	outb( channel , PM511P_ADDR );	/*设置 A/D 采样通道*/
	udelay(CHANNEL_DELAY);//waiting for setting the channel of A/D
	
	for(i = 0; i < samplingNum; i++)
	{
		inb( PM511P_ADDR );	/*启动 A/D 转换*/
		
#ifdef _CONST_DELAY_
		udelay(AD_CONVERSION_DELAY);//waiting for A/D conversion finish
#else
		while( inb( PM511P_ADDR + 1 ) &  0x01);	/*判断 A/D 转换是否完成*/
#endif	

		value = inw( PM511P_ADDR + 2 ) & 0xfff;
		total_value += value;
	}
	value = total_value / samplingNum;
	return (value - 2048) * 16;
};

short fast_AD()
{
	short value;
	inb( PM511P_ADDR );	/*启动 A/D 转换*/
	
#ifdef _CONST_DELAY_
	udelay(AD_CONVERSION_DELAY);//waiting for A/D conversion finish
#else
	while( inb( PM511P_ADDR + 1 ) &  0x01);	/*判断 A/D 转换是否完成*/
#endif	
	
	value = inw( PM511P_ADDR + 2 ) & 0xfff;
	return (value - 2048) * 16;
}

short fast_AD_N(int samplingNum)
{
	int total_value = 0;
	short i, value;

	for(i = 0; i < samplingNum; i++)
	{
		inb( PM511P_ADDR );	/*启动 A/D 转换*/

#ifdef _CONST_DELAY_
		udelay(AD_CONVERSION_DELAY);//waiting for A/D conversion finish
#else
		while( inb( PM511P_ADDR + 1 ) &  0x01);	/*判断 A/D 转换是否完成*/
#endif	

		value = inw( PM511P_ADDR + 2 ) & 0xfff;
		total_value += value;
		
	}
	value = total_value / samplingNum;
	return (value - 2048) * 16;
}

void init_Hardware()
{
	iopl(3);
	outb(0x80, IO_CONTROL_WORD); // 所有 IO 口都设为输出模式
	
	write_DA(DA_X, 32768); 
	write_DA(DA_Y, 32768); 
	write_DA(DA_Z, 32768); 
}

