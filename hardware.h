/*
 * hardware.h hardware.c 包含各种硬件相关函数，包括 AD／DA 采集卡， DDS，数字 IO量的相关例程
 * Originated by 李渊 on 2007-08-23
 * Version 0.0.2
 * Copyright (C) 2007, by LI Yuan <liyuan@ss.buaa.edu.cn>.
 * Licensed under GPLv2 or later, see file LICENSE for details.


  * Last Modified on 2008-02-19
*/

#ifndef HARDWARE_H
#define HARDWARE_H

#include<sys/io.h>
#include <string.h>
#include <pthread.h>

extern const unsigned char AD_LASER_A;
extern const unsigned char AD_LASER_B;
extern const unsigned char AD_LASER_C;
extern const unsigned char AD_LASER_D;
extern const unsigned char AD_LASER_SUM;
extern const unsigned char AD_OSAP;
extern const unsigned char AD_FICTION; //not true
extern const unsigned char AD_PHASE;
extern const unsigned char AD_ERROR;
extern const unsigned char AD_STM;
extern const unsigned char DA_X;
extern const unsigned char DA_Y;
extern const unsigned char DA_Z;

//#ifdef _HT7489_
//// for HT7489 AD/DA
//	#define AD_LASER_A	0x00
//	#define AD_LASER_B	0x01
//	#define AD_LASER_C	0x02
//	#define AD_LASER_D	0x03
//	#define AD_LASER_SUM	0x04
//	#define AD_OSAP		0x05
//	#define AD_PHASE	0x0B
//	#define AD_ERROR	0x0A
//	#define AD_STM		0x0C
//
//	#define DA_X		0x04
//	#define DA_Y		0x05
//	#define DA_Z		0x06
//#endif
//
//#ifdef _PM511P_
//// for PM511P AD/DA
//	#define DA_X		0x00
//	#define DA_Y		0x01
//	#define DA_Z		0x02
//
//	#define AD_LASER_A	0x02
//	#define AD_LASER_B	0x01
//	#define AD_LASER_C	0x04
//	#define AD_LASER_D	0x03
//	#define AD_LASER_SUM	0x00
//	#define AD_OSAP		0x07
//	#define AD_ERROR	0x06
//// not true
//	#define AD_PHASE	0x0B
//	#define AD_STM		0x0C
//
////#define BASEADDR 0x100
//#endif

short read_AD( unsigned char channel );
short read_AD_TS( unsigned char channel );
short read_AD_N( unsigned char channel, int samplingNum );
short fast_AD();
short fast_AD_TS();
short fast_AD_N(int samplingNum);
void write_DA( unsigned char channel, unsigned short value );
void write_DA_TS( unsigned char channel, unsigned short value );

void init_Hardware();

void IO_Out(unsigned char channel, unsigned char value);
void IO_Out8(unsigned char channel, unsigned char value);
void IO_Out16(unsigned short value);
void dds_Out(double freq);
void dds_Stop();
void setWaveAmplitude(unsigned char amplitude);
void setPhase(unsigned short phase);
void setLedOn();
void setLedOff();
void setLaserOn();
void setLaserOff();
void setHighVoltageOn();
void setHighVoltageOff();
void motor_forward_one_step();
void motor_backward_one_step();
short motor_get_steps();
void motor_stop();


inline static void udelay(int usecs)
{
	int i; for(i = 0; i < usecs; i++) outb(0, 0x80);
}

#define setBitToOne(Data, nBit)  ( Data |= (1 << nBit) )
#define setBitToZero(Data, nBit) ( Data &= ~(1 << nBit) )

#endif


