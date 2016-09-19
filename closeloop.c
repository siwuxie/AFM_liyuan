/*
 * algorithm.h algorithm.c ����������ջ��㷨
 * Originated by ��Ԩ on 2007��07��11��
 * Version 0.0.1
 * Copyright (C) 2007, by LI Yuan <liyuan@ss.buaa.edu.cn>.
 * Licensed under GPLv2 or later, see file LICENSE for details.
*/

/*
 *  ����ֻ������򵥵� PID �㷨�������㷨������
*/
#include "afm_comm.h"
#include "closeloop.h"
#include "hardware.h"
#include <stdlib.h>
#include <stdio.h>


//---PID����------
pthread_mutex_t g_PID_mutex = PTHREAD_MUTEX_INITIALIZER;
int PID_Kp;
int PID_Ki;
int PID_Kd;
int PID_eps;
int PID_errorThresh;
int PID_loopCircles; //ѭ������
int PID_samplingTimes;//AD ��������
int PID_delay;//�ȴ�ʱ��

unsigned char PID_channel;
short PID_setPoint;

short (*pid_func)();
//PID_FUNC PID_LIST[5];

#define PID_DELAY 10
void setFeedBackMode(short mode)
{
	switch(mode)
	{
		case 0:
			pid_func = PID_function00;
			DEBUG0("Set PID Func to PID_function00");
			break;
		case 1:
			pid_func = PID_function01;
			DEBUG0("Set PID Func to PID_function01");
			break;
		case 2:
			pid_func = PID_function02;
			DEBUG0("Set PID Func to PID_function02");
			break;		
		case 3:
			pid_func = PID_function03;
			DEBUG0("Set PID Func to PID_function03");
			break;
	}
}
void setPIDParaOther(short eps, short times, short delay, short errorThresh)
{
	PID_eps = eps;
	PID_delay = delay;
	PID_errorThresh = errorThresh;
	if(times <= 0) PID_samplingTimes = 1;
	else PID_samplingTimes = times;
		
	DEBUG1("Set PID_eps to %d\n", PID_eps);
	DEBUG1("Set PID_delay to %d\n", PID_delay);
	DEBUG1("Set PID_errorThresh to %d\n", PID_errorThresh);
	DEBUG1("Set PID_samplingTimes to %d\n\n", PID_samplingTimes);
}
void setPIDPara(short Kp, short Ki, short Kd, short loopCircles)
{
	PID_Kp = Kp;
	PID_Ki = Ki;
	PID_Kd = Kd;
	PID_loopCircles = loopCircles;
	DEBUG0("PID parameters have been changed");
	DEBUG1("Set PID_Kp to %d\n", PID_Kp);
	DEBUG1("Set PID_Ki to %d\n", PID_Ki);
	DEBUG1("Set PID_Kd to %d\n", PID_Kd);
	DEBUG1("Set PID_loopCircles to %d\n\n", PID_loopCircles);
}
void setPIDChannel(unsigned char channel)
{
	PID_channel = channel;
	DEBUG1("Set PID_channel to %d\n", PID_channel);
}
unsigned char getPIDChannel(void)
{
	return PID_channel;
}
void setPIDSetPoint(short setPoint)
{
	PID_setPoint = setPoint;
	DEBUG1("Set PID_setPoint to %d\n", PID_setPoint);
}
// DA_Z ͨ·��һ�������·
// g_DA_z = 0 (-10V) ��ʾ�մɹ��쵽�
// g_DA_z = 65535 (10V) ��ʾ�մɹ��������
/**
*@brief  ֻ�л��ֻ��ڵ�����ʽ�㷨
*@param  void
*@return  ����ź� short 
*/
short PID_function00(void)
{
	int error = 0, zValue, deltaZ = 0, loop = 0, delay ;
	int Ki, loopCircles, channel, setPoint;

/* Initialization of PID parameters */	
	Ki = PID_Ki;
	setPoint = PID_setPoint;
	channel = PID_channel;
	loopCircles = PID_loopCircles;	
//	delay = PID_delay;
	
	zValue = g_DA_z;
	//DEBUG0("in simplePID()");
	error = -(read_AD(channel) - setPoint);
	while(loop < loopCircles)
	{
		if(abs(error) < 100) error /= 4;
		deltaZ = error * Ki / 1000.0;
		zValue += deltaZ;
		if(zValue > 65535)	zValue = 65535;
		if(zValue < 0)		zValue = 0;
		g_DA_z = zValue;
		write_DA( DA_Z, g_DA_z);
//		udelay(delay); //waiting for a short time
		error = -(fast_AD( ) - setPoint);
		loop++;
	}
	return -error;	
}

/**
*@brief  ֻ�б����ͻ��ֻ��ڵ�����ʽ�㷨
*@param  void
*@return  ����ź� short 
*/
short PID_function01(void)
{
	int error = 0, zValue, deltaZ = 0, loop = 0;
	static int last_error = 0;
	int Ki, Kp, loopCircles, channel, setPoint, count, delay;
/* Initialization of PID parameters */	
	Ki = PID_Ki;
	Kp = PID_Kp;
	setPoint = PID_setPoint;
	channel = PID_channel;
	loopCircles = PID_loopCircles;
	count = PID_samplingTimes;	
//	delay = PID_delay;
	
	zValue = g_DA_z;
	//DEBUG0("in simplePID()");
	last_error = 0;
	error = -(read_AD_N(channel, count) - setPoint);
	last_error = error;
	while(loop < loopCircles)
	{
		deltaZ = ((error + last_error) * Ki + Kp * (error - last_error)) / 1000.0;
		zValue += deltaZ;
		if(zValue > 65535)	zValue = 65535;
		if(zValue < 0)		zValue = 0;
		g_DA_z = zValue;
		write_DA( DA_Z, g_DA_z);
//		udelay(delay); //waiting for a short time
		last_error = error;
		error = -(fast_AD_N(count) - setPoint);
		loop++;
	}
	return -error;		
}


short PID_function00_N(void)
{
	int error = 0, zValue, deltaZ = 0, loop = 0;
	int Ki, loopCircles, channel, setPoint, count, delay;

/* Initialization of PID parameters */	
	Ki = PID_Ki;
	setPoint = PID_setPoint;
	channel = PID_channel;
	loopCircles = PID_loopCircles;
	count = PID_samplingTimes;	
	delay = PID_delay;
	
	zValue = g_DA_z;
	//DEBUG0("in simplePID()");
	error = -(read_AD_N(channel, count) - setPoint);
	while(loop < loopCircles)
	{
		if(abs(error) < 250) error /= 4;
		deltaZ = error * Ki / 1000.0;
		zValue += deltaZ;
		if(zValue > 65535)	zValue = 65535;
		if(zValue < 0)		zValue = 0;
		g_DA_z = zValue;
		write_DA( DA_Z, g_DA_z);
		udelay(delay); //waiting for a short time
		error = -(fast_AD_N(count) - setPoint);
		loop++;
	}
	return -error;	
}
short PID_function04_N(void)
{
	int error = 0, zValue, deltaZ = 0, loop = 0, last_error;
	int Ki, Kp, loopCircles, channel, setPoint, count, delay;

/* Initialization of PID parameters */	
	Ki = PID_Ki;
	Kp = PID_Kp;
	setPoint = PID_setPoint;
	channel = PID_channel;
	loopCircles = PID_loopCircles;
	count = PID_samplingTimes;	
	delay = PID_delay;
	
	zValue = g_DA_z;
	//DEBUG0("in simplePID()");
	last_error = 0;
	error = -(read_AD_N(channel, count) - setPoint);
	last_error = error;
	while(loop < loopCircles)
	{
		deltaZ = error * Ki / 1000.0 + Kp * (error - last_error) / 1000;
		zValue += deltaZ;
		if(zValue > 65535)	zValue = 65535;
		if(zValue < 0)		zValue = 0;
		g_DA_z = zValue;
		write_DA( DA_Z, g_DA_z);
		udelay(delay); //waiting for a short time
		last_error = error;
		error = -(fast_AD_N(count) - setPoint);
		loop++;
	}
	return -error;	
}
//original PI controller, const loop circles
//unsigned short PID_function01(void)
//{
//	int Kp, Ki, loopCircles, channel, setPoint, delay;
//	static int integral = 0;
//	int loop = 0, zValue, error;
//
///* Initialization of PID parameters */	
//	Kp = PID_Kp;
//	Ki = PID_Ki;
//	setPoint = PID_setPoint;
//	channel = PID_channel;
//	loopCircles = PID_loopCircles;
//	delay = PID_delay;
//	
//	error = -(read_AD(channel) - setPoint);// �õ����ֵ
//	while(loop < loopCircles)
//	{
//		integral += Ki * error / 1000.0; 
//		if(integral > 65535) integral = 65535;
//		if(integral < 0) integral = 0;
//		/* PID ���������� */
//		zValue = Kp * error / 1000.0  + integral;
//		// ���� zValue �� DA ������Χ֮��	
//		if(zValue > 65535)		zValue = 65535;
//		if(zValue < 0)			zValue = 0;
//		
//		g_DA_z = zValue;
//		write_DA( DA_Z, g_DA_z);
//		udelay(delay); //waiting for a short time
//		error = -(read_AD(channel) - setPoint);// �õ����ֵ
//		loop++;
//	}
//	return (error + 32768) ;    
//}

short PID_function01_N(void)
{
	int Kp, Ki, loopCircles, channel, setPoint, count, delay;
	static int integral = 0;
	int loop = 0, zValue, error;

/* Initialization of PID parameters */	
	Kp = PID_Kp;
	Ki = PID_Ki;
	setPoint = PID_setPoint;
	channel = PID_channel;
	loopCircles = PID_loopCircles;
	count = PID_samplingTimes;	
	delay = PID_delay;
	
	error = -(read_AD_N(channel, count) - setPoint);// �õ����ֵ
	while(loop < loopCircles)
	{
		integral += Ki * error / 1000.0; 
		if(integral > 65535) integral = 65535;
		if(integral < 0) integral = 0;
		/* PID ���������� */
		zValue = Kp * error / 1000.0  + integral;
		// ���� zValue �� DA ������Χ֮��	
		if(zValue > 65535)		zValue = 65535;
		if(zValue < 0)			zValue = 0;
		
		g_DA_z = zValue;
		write_DA( DA_Z, g_DA_z);
		udelay(delay); //waiting for a short time
		
		error = -(fast_AD_N(count) - setPoint);// �õ����ֵ
		loop++;
	}
	return -error;    
}


short PID_function01_debug(short new_setPoint, short record_dots)
{
	short *err_array, *zValue_array, *integ_array;
	int Kp, Ki, loopCircles, channel, setPoint, delay;
	static int integral = 0;
	int loop = 0, zValue, error;
	FILE *fp;
	
	err_array = (short *)malloc(sizeof(short) * record_dots);
	zValue_array = (short *)malloc(sizeof(short) * record_dots);
	integ_array = (short *)malloc(sizeof(short) * record_dots);
/* Initialization of PID parameters */	
	Kp = PID_Kp;
	Ki = PID_Ki;
	setPoint = PID_setPoint;
	channel = PID_channel;
	loopCircles = PID_loopCircles;
	delay = PID_delay;
	
	error = -(read_AD(channel) - setPoint);// �õ����ֵ
	while(loop < loopCircles)
	{
		integral += Ki * error / 1000.0; 
		if(integral > 65535) integral = 65535;
		if(integral < 0) integral = 0;
		/* PID ���������� */
		zValue = Kp * error / 1000.0  + integral;
		// ���� zValue �� DA ������Χ֮��	
		if(zValue > 65535)		zValue = 65535;
		if(zValue < 0)			zValue = 0;
		
		g_DA_z = zValue;
		write_DA( DA_Z, g_DA_z);
		udelay(delay); //waiting for a short time
		
		error = -(read_AD(channel) - setPoint);// �õ����ֵ
		loop++;
	}
	
	loop =0;
	setPoint = new_setPoint;
	error = -(read_AD(channel) - setPoint);// �õ����ֵ
	while(loop < record_dots)
	{
		integral += Ki * error / 1000.0; 
		if(integral > 65535) integral = 65535;
		if(integral < 0) integral = 0;
		/* PID ���������� */
		zValue = Kp * error / 1000.0  + integral;
		// ���� zValue �� DA ������Χ֮��	
		if(zValue > 65535)		zValue = 65535;
		if(zValue < 0)			zValue = 0;
		
		err_array[loop] = error;		
		integ_array[loop] = integral;	
		zValue_array[loop]=zValue;		
		g_DA_z = zValue;
		write_DA( DA_Z, g_DA_z);
		udelay(delay); //waiting for a short time
		
		error = -(read_AD(channel) - setPoint);// �õ����ֵ
		loop++;
	}	


	fp = fopen("pid_error.dat","w");
	fprintf(fp, "# Kp = %d, Ki = %d\n", Kp, Ki);
	fprintf(fp, "# original set point = %d\n", setPoint);
	fprintf(fp, "# new set point = %d\n", new_setPoint);
	fprintf(fp, "# error, \tintegral, \tzValue\n");
	for(loop = 0; loop < record_dots; loop++)
	{
		fprintf(fp, "%d, \t%d, \t%d\n", err_array[loop], integ_array[loop], zValue_array[loop]);
	}
	fclose(fp);	
	
	free(integ_array);
	free(zValue_array);
	free(err_array);
	return 0;//(unsigned short)(error + 32767);    
}
//original PID controller, const loop circles
short PID_function02(void)
{
	int Kp, Ki, Kd, loopCircles, channel, setPoint;
	int error, prevError = 0, deriv = 0;
	static int integral = 0;
	int loop = 0, zValue, started = 0, delay;

/* Initialization of PID parameters*/	
	Kp = PID_Kp;
	Ki = PID_Ki;
	Kd = PID_Kd;
	setPoint = PID_setPoint;
	channel = PID_channel;
	loopCircles = PID_loopCircles;
	delay = PID_delay;
	
	error = -(read_AD(channel) - setPoint);// �õ����ֵ
	while(loop < loopCircles)
	{
	  integral += Ki * error / 1000.0; 
	  if(integral > 65535) integral = 65535;
		if(integral < 0) integral = 0;
	  /* �������΢�� */
	  if(!started)
	  {
	  	started = 1;  	deriv = 0;
	  }
	  else
	  {
	  	deriv = (error - prevError);
	  }
		prevError = error;
		/* PID ���������� */
		zValue = (Kp * error + Kd * deriv) / 1000.0  + integral;
	
		// ���� zValue �� DA ������Χ֮��	
		if(zValue > 65535)		zValue = 65535;
		if(zValue < 0)			zValue = 0;
		
		g_DA_z = zValue;
		write_DA( DA_Z, g_DA_z);
		udelay(delay); //waiting for a short time
		
		error = -(read_AD(channel) - setPoint);// �õ����ֵ
		loop++;
	}
	return -error;    
}

//�����������ͣ�integrator windup��PID ������ 
short PID_function03(void)
{
	int Kp, Ki, Kd, loopCircles, eps, errorThresh, channel, setPoint;
	int error, prevError = 0, deriv = 0;
	static int integral = 0;
	int loop = 0, zValue, started = 0, delay;

/* Initialization of PID parameters*/	
	Kp = PID_Kp;
	Ki = PID_Ki;
	Kd = PID_Kd;
	eps = PID_eps;
	errorThresh = PID_errorThresh;
	loopCircles = PID_loopCircles;	
	delay = PID_delay;
	
	setPoint = PID_setPoint;
	channel = PID_channel;	
	
//	m_integral = g_DA_z * 1000 / Kd;
	error = -(read_AD(channel) - setPoint);// �õ����ֵ
//	if(Ki == 0) max_of_integral = 100;
//	else max_of_integral = 65535000 / Ki;
	
	while( (loop < loopCircles))// && ( abs(error) > eps) )
	{
		/* ��������ȵ������ޣ� ����������� */
		if( abs(error) < errorThresh)  
		{  	
			integral += Ki * error / 1000.0;  
			if(integral > 65535) integral = 65535;
			if(integral < 0) integral = 0;			
		}
//		if(integral > max_of_integral) integral = max_of_integral;
//		if(integral < 0) integral = 0;
		/* �������΢�� */
		if(!started)
	  {
		started = 1;
	  	deriv = 0;
	  }
	  else
	  {
	  	deriv = (error - prevError);
	  }
		prevError = error;
		/* PID ���������� */
		zValue = (Kp * error + Kd * deriv) / 1000.0 + integral;
	
		// ���� zValue �� DA ������Χ֮��	
		if(zValue > 65535)			zValue = 65535;
		if(zValue < 0)			zValue = 0;
		
		g_DA_z = zValue;
		write_DA( DA_Z, g_DA_z);
		udelay(delay); //waiting for a short time
		
		error = -(read_AD(channel) - setPoint);// �õ����ֵ
		loop++;
	}
	return -error;    
}

short getError(int n)
{
	return (read_AD_N(PID_channel, n) - PID_setPoint);
}
