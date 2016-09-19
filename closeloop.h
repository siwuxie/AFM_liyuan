/*
 * closeloop.h closeloop.c 包含各种软闭环算法
 * Originated by 李渊 on 2007年07月11日
 * Version 0.0.1
 * Copyright (C) 2007, by LI Yuan <liyuan@ss.buaa.edu.cn>.
 * Licensed under GPLv2 or later, see file LICENSE for details.
*/

/*
 *  现在只包括最简单的 PID 算法，其余算法待补充
*/

#ifndef ALGORITHM_H
#define ALGORITHM_H
#include <pthread.h>

extern short (*pid_func)();

//unsigned short simplePID();//只包括比例反馈
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
// 下面是软闭环控制所需的各全局变量
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

extern int g_DA_z; //保存 Z 向的 DA 输出值
#endif

