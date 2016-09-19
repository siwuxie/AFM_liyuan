/*
 * closeloop.h closeloop.c ����������ջ��㷨
 * Originated by ��Ԩ on 2007��07��11��
 * Version 0.0.1
 * Copyright (C) 2007, by LI Yuan <liyuan@ss.buaa.edu.cn>.
 * Licensed under GPLv2 or later, see file LICENSE for details.
*/

/*
 *  ����ֻ������򵥵� PID �㷨�������㷨������
*/

#ifndef ALGORITHM_H
#define ALGORITHM_H
#include <pthread.h>

extern short (*pid_func)();

//unsigned short simplePID();//ֻ������������
short PID_function00(void);
short PID_function00_N(void);
short PID_function01(void);
short PID_function01_N(void);
short PID_function02(void);
short PID_function03(void);
short PID_function01_debug(short new_setPoint, short record_dots);

void setPIDPara(short Kp, short Ki, short Kd, short loopCircles);
void setPIDParaOther(short eps, short times, short delay, short errorThresh);
void setFeedBackMode(short mode);
void setPIDChannel(unsigned char channel);
void setPIDSetPoint(short setPoint);
short getError(int n);
unsigned char getPIDChannel(void);
// ��������ջ���������ĸ�ȫ�ֱ���
extern pthread_mutex_t g_PID_mutex;

//typedef struct 
//{
	//int Kp;
	//int Ki;
	//int Kd;
	//int eps;
	//int max_loop_circles;
	//int error_thresh;
//} PID_PARA;

//PID_PARA g_pid_para;

extern int g_DA_z; //���� Z ��� DA ���ֵ
#endif

