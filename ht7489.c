/*ֻ�� hardware.c ���ã��벻Ҫ����Ŀ��ֱ��ʹ�ñ��ļ�*/

#include "hardware.h"

#define HT7489_ADDR 	0x1B0
	
#define CHANNEL_DELAY 5
#define AD_CONVERSION_DELAY 9
#define DA_CONVERSION_DELAY 9

const unsigned char AD_LASER_A = 0x00;
const unsigned char AD_LASER_B = 0x01;
const unsigned char AD_LASER_C = 0x02;
const unsigned char AD_LASER_D = 0x03;
const unsigned char AD_LASER_SUM = 0x04;
const unsigned char AD_OSAP = 0x05;
const unsigned char AD_FICTION = 0x08; // not real
const unsigned char AD_PHASE = 0x0B;
const unsigned char AD_ERROR = 0x09;
const unsigned char AD_STM = 0x0C;
const unsigned char DA_X = 0x04;
const unsigned char DA_Y = 0x05;
const unsigned char DA_Z = 0x06;

short read_AD( unsigned char channel )
{
	outb(channel, HT7489_ADDR);/*���� A/D ����ͨ��, ͨ�� channel*/
	udelay(CHANNEL_DELAY);//waiting for setting the channel of A/D
	outb(0, HT7489_ADDR + 1);/*���� A/D ת��*/
	

#ifdef _CONST_DELAY_
	udelay(AD_CONVERSION_DELAY);//waiting for A/D conversion finish
#else
	while( (inb( HT7489_ADDR ) & 0x01) );/*�ж� A/D �����Ƿ����*/
#endif

	return inw(HT7489_ADDR + 2) & 0xffff;
};

static pthread_mutex_t g_ad_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_da_mutex = PTHREAD_MUTEX_INITIALIZER;


short read_AD_TS( unsigned char channel )
{
	short value;
	pthread_mutex_lock(&g_ad_mutex);
	outb(channel, HT7489_ADDR);/*���� A/D ����ͨ��, ͨ�� channel*/
	udelay(CHANNEL_DELAY);//waiting for setting the channel of A/D
	outb(0, HT7489_ADDR + 1);/*���� A/D ת��*/

#ifdef _CONST_DELAY_
	udelay(AD_CONVERSION_DELAY);//waiting for A/D conversion finish
#else
	while( (inb( HT7489_ADDR ) & 0x01) );/*�ж� A/D �����Ƿ����*/
#endif

	value = inw(HT7489_ADDR + 2) & 0xffff;
	pthread_mutex_unlock(&g_ad_mutex);
	return value;
};

short read_AD_N( unsigned char channel, int samplingNum )
{
	short i, value;
	int total_value = 0;
	outb(channel, HT7489_ADDR);/*���� A/D ����ͨ��, ͨ�� channel*/
	udelay(CHANNEL_DELAY);//waiting for setting the channel of A/D
		
	for(i = 0; i < samplingNum; i++)
	{
		outb(0, HT7489_ADDR + 1);/*���� A/D ת��*/
		
#ifdef _CONST_DELAY_
		udelay(AD_CONVERSION_DELAY);//waiting for A/D conversion finish
#else
		while( (inb( HT7489_ADDR ) & 0x01) );/*�ж� A/D �����Ƿ����*/
#endif
		value = inw(HT7489_ADDR + 2) & 0xffff;
		total_value += value;
	}
	value = total_value / samplingNum;
	return value;
};



short fast_AD()
{
	outb(0, HT7489_ADDR + 1);/*���� A/D ת��*/
	
#ifdef _CONST_DELAY_
	udelay(AD_CONVERSION_DELAY);//waiting for A/D conversion finish
#else
	while( (inb( HT7489_ADDR ) & 0x01) );/*�ж� A/D �����Ƿ����*/
#endif

	return inw(HT7489_ADDR + 2) & 0xffff;
}

short fast_AD_N(int samplingNum)
{
	int total_value = 0;
	short i, value;

	for(i = 0; i < samplingNum; i++)
	{
		outb(0, HT7489_ADDR + 1);/*���� A/D ת��*/

#ifdef _CONST_DELAY_
	udelay(AD_CONVERSION_DELAY);//waiting for A/D conversion finish
#else
	while( (inb( HT7489_ADDR ) & 0x01) );/*�ж� A/D �����Ƿ����*/
#endif
		value = inw(HT7489_ADDR + 2) & 0xffff;
		total_value += value;
	}
	value = total_value / samplingNum;
	return value;
}

short fast_AD_TS()
{
	short value;
	pthread_mutex_lock(&g_ad_mutex);
	outb_p(0, HT7489_ADDR + 1);/*���� A/D ת��*/

#ifdef _CONST_DELAY_
	udelay(AD_CONVERSION_DELAY);//waiting for A/D conversion finish
#else
	while( (inb( HT7489_ADDR ) & 0x01) );/*�ж� A/D �����Ƿ����*/
#endif

	value = inw(HT7489_ADDR + 2) & 0xffff;
	pthread_mutex_unlock(&g_ad_mutex);
	return value;
}

void write_DA_TS( unsigned char channel, unsigned short value )
{
	pthread_mutex_lock(&g_da_mutex);
	outw(value, HT7489_ADDR + 4);/* д D/A ת������*/
	inb(HT7489_ADDR + channel);/*���� D/A ת��*/
	pthread_mutex_unlock(&g_da_mutex);
	udelay(DA_CONVERSION_DELAY);//waiting for D/A conversion finish
};
void write_DA( unsigned char channel, unsigned short value )
{
	
	outw(value, HT7489_ADDR + 4);/* д D/A ת������*/
	inb(HT7489_ADDR + channel);/*���� D/A ת��*/
	udelay(DA_CONVERSION_DELAY);//waiting for D/A conversion finish
};
void init_Hardware()
{
	iopl(3);	/*ʹ��ֱ�Ӳ����˿ڣ���Ҫ*/
	//У�� HT 7489 AD ͨ��
	outb(0, HT7489_ADDR + 2);//����У������
	while((inb( HT7489_ADDR ) & 0x01));//�ж�У�������Ƿ����

};

