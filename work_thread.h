/*
 * PC104 控制板上控制程序
 * work_thread.h work_thread.c 包含各工作线程
 * Originated by 李渊 on 2007年08月15日
 * Version 0.0.2
 * Copyright (C) 2007, by LI Yuan <liyuan@ss.buaa.edu.cn>.
 * Licensed under GPLv2 or later, see file LICENSE for details.
 */
/*
 * Last Modified on 2007.0824 by LI Yuan
 */
#ifndef WORKTHREAD_H
#define WORKTHREAD_H

#include <pthread.h>
#include <sys/socket.h>
#include "afm_comm.h"
#include "closeloop.h"

void readHugeData();

void dispatch_cmd(struct command *pCmd);
void setMode(struct command *pCmd);

/*in laserThread.c*/
void* laserThread(void* para);

void* get_error_thread(void* para);
void* pid_get_error_thread(void* para);

/*in motorThread.c*/
void motor_steps(int steps, int direction);
void* motor_autoforward_thread(void* para);
void* motor_autobackward_thread(void* para);
void* motor_fast_forward_thread(void* para);
void* motor_fast_backward_thread(void* para);


/*in scanThread.c*/
void setScanRange(int scanRangeX,	int scanRangeY, int scanOffsetX, int scanOffsetY, int scanAngle);
void setScanPixel(int XDots, int YDots);
void setScanDelay(int delay);
void setScanSamplingTimes(int times);
void setHysteresisPara(int m, int h);
void* lineScanThread(void* para);
void* fastScanThread(void* para);
void* normalScanThread(void* para);

/* in FreqScanThread.h*/
void setFreqRange(unsigned int startFreq, unsigned int endFreq);
void* feedback_thread(void* para);// only for test purpose
void* freqScanThread(void* para);

/*in forceCurveThread.c*/
void *forceCurveThread(void *para);

//extern int g_scanPixels;
//extern int g_scanDelays;
//
//extern int g_samplingNum;

//extern int g_PID_P;
//extern int g_PID_I;
//extern int g_PID_D;
//extern int g_PID_eps;
//extern int g_PID_error_thresh;
//extern int g_PID_maxLoopCircles;


extern pthread_mutex_t g_current_task_mutex;
extern int g_current_task;

extern int g_DA_z; //保存 Z 向的 DA 输出值
extern pthread_mutex_t g_PID_mutex;

extern int connect_socket_fd; // 与上位机通讯所用 socket 
#endif

