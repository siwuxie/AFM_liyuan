 /*
 * PC104 ���ư��Ͽ��Ƴ���
 * main.c ��������Ҫ���ͨѶ����
 * Originated by ��Ԩ on 2007��07��11��
 * Version 0.0.3
 * Copyright (C) 2007, by LI Yuan <liyuan@ss.buaa.edu.cn>.
 * Licensed under GPLv2 or later, see file LICENSE for details.

 * modified on 2007��07��15�� ������ 21ʱ55��15��
 * Last Modified on 2008.0103 By LI Yuan
 */
#include <sys/types.h>
#include <netinet/in.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>


#include "afm_comm.h"
#include "closeloop.h"
#include "work_thread.h"
#include "hardware.h"
#include "restart.h"
#include "serial_port.h"


void catcher_SIGPIPE();
void init_para(void);
extern pthread_t scanTid, motorTid, laserTid, freqScanTid;
int connect_socket_fd;
int main(int argc, char* argv[])
{
	int listen_socket_fd, len;
	struct sockaddr_in server_addr;
	struct command new_cmd;


	/*����ǰ��������ȼ���ߵ�ʵʱ�������ȼ�*/
//	struct sched_param param; 
//	param.sched_priority = 10;
//	sched_setscheduler(0, SCHED_FIFO, &param);	

	static struct sigaction sig_act;
	//ע��SIGPIPE �źŴ�������
	//��һ��������ͼ��һ���Ѿ��Ͽ����ӵ��׽��� write �� send ʱ�����յ� SIGPIPE �ź�
	sig_act.sa_handler = catcher_SIGPIPE;
	sigfillset( &(sig_act.sa_mask) );
	sigaction(SIGPIPE, &sig_act, NULL);

	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(SERVER_PORT);
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	bzero( &(server_addr.sin_zero), 8 );

	//����һ������ socket�������˿� 7000���ȴ���λ������
	if((listen_socket_fd = socket(PF_INET, SOCK_STREAM, 0)) == -1)
	{
		perror("socket call failed");
		exit(1);
	}
	DEBUG0("socket create success!");
	if((bind(listen_socket_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in))) == -1)
	{
		perror("bind call failed");
		exit(2);
	}
	DEBUG0("socket bind success");
	if((listen(listen_socket_fd, 1))== -1)
	{
		perror("listen call failed");
		exit(3);
	}
	DEBUG0("socket listening success");

	init_Hardware(); // declared in hardware.h
	DEBUG0("init_Hardware()");
	lcdBeep(200);
	init_para();//declared below
	DEBUG0("init_para()");
	DEBUG0("20080318");
	while(1)
	{
		DEBUG0("Waiting for a new connection.");
		// ����һ��������������λ����������
		if((connect_socket_fd = accept(listen_socket_fd, NULL, NULL)) == -1)
    {
			perror("accept call failed");
			exit(4);
    }
		DEBUG0("A new connection set up sucessful.");
		init_para();
		while( (len = r_read(connect_socket_fd, (char *)&new_cmd, 12)) > 0 )
		{
			//����������
			//������߳�
			DEBUG1("receive cmd 0x%x\n", new_cmd.cmd);
			dispatch_cmd(&new_cmd);
		}
		pthread_cancel(scanTid);
		pthread_cancel(motorTid);
		pthread_cancel(laserTid);
		pthread_cancel(freqScanTid);
		r_close(connect_socket_fd);
		DEBUG0("Connection closed");
		g_current_task = STOP;
		// �����жϣ��ȴ����й����߳̽�����ǿ�ƽ������й����߳�
		// �������ƺ����������ͷŸ�����Ӧ��Դ
		// ׼���ȴ���һ���µ�����
	}
}
void init_para(void)
{
	//scanRangeX, scanRangeY, scanOffsetX, scanOffsetY, scanAngle
	setScanRange(30000,	30000, 0,	0, 0);
	setScanPixel(512, 512);
	setScanDelay(0);

	g_DA_z = 0;
//void setPIDPara(short Kp, short Ki, short Kd, short loopCircles);
//void setPIDParaOther(short eps, short times, short delay, short errorThresh);	
	setPIDPara(60, 60, 0, 30);
	setPIDParaOther(0, 0, 0, 1);
	setPIDChannel(10);
	setPIDSetPoint(0);	
	pid_func = PID_function01;
	
	g_current_task = STOP;
}
/*
void init_para(void)
{
	FILE* fp;
	if(fp = fopen("./.server_para", "r"))
	{
		fscanf(fp, "Scan Range X : %d\n", &g_scanRangeX);
		fscanf(fp, "Scan Range Y : %d\n", &g_scanRangeY);
		fscanf(fp, "Scan Offset X : %d\n", &g_scanOffsetX);
		fscanf(fp, "Scan Offset Y : %d\n", &g_scanOffsetY);
		fscanf(fp, "Scan Angle : %d\n", &g_scanAngle);
		fscanf(fp, "Scan Pixels : %d\n", &g_scanPixels);
		fscanf(fp, "Scan Delays : %d\n", &g_scanDelays);
		
		fscanf(fp, "Kp of PID : %d\n", &g_PID_P);
		fscanf(fp, "Ki of PID : %d\n", &g_PID_I);
		fscanf(fp, "Kd of PID : %d\n", &g_PID_D);
		fscanf(fp, "Eps of PID : %d\n", &g_PID_eps);
		fscanf(fp, "Max Loop Circles of PID : %d\n", &g_PID_maxLoopCircles);
		
		fclose(fp);
	}
	else
	{	
		g_scanRangeX = 30000;
		g_scanRangeY = 30000;
		g_scanOffsetX = 0;
		g_scanOffsetY = 0;
		g_scanAngle = 0;
		g_scanPixels = 512;
		g_scanDelays = 0;
	
		g_samplingNum = 5;
	
		g_PID_P = 60;
		g_PID_I = 60;
		g_PID_D = 0;
		g_PID_eps = 100;
		g_PID_maxLoopCircles = 30;
		
		if(fp = fopen("./.server_para", "w"))
		{
			fprintf(fp, "Scan Range X : %d\n", g_scanRangeX);
			fprintf(fp, "Scan Range Y : %d\n", g_scanRangeY);
			fprintf(fp, "Scan Offset X : %d\n", g_scanOffsetX);
			fprintf(fp, "Scan Offset Y : %d\n", g_scanOffsetY);
			fprintf(fp, "Scan Angle : %d\n", g_scanAngle);
			fprintf(fp, "Scan Pixels : %d\n", g_scanPixels);
			fprintf(fp, "Scan Delays : %d\n", g_scanDelays);
			
			fprintf(fp, "Kp of PID : %d\n", g_PID_P);
			fprintf(fp, "Ki of PID : %d\n", g_PID_I);
			fprintf(fp, "Kd of PID : %d\n", g_PID_D);
			fprintf(fp, "Eps of PID : %d\n", g_PID_eps);
			fprintf(fp, "Max Loop Circles of PID : %d\n", g_PID_maxLoopCircles);	
			fclose(fp);		
		}
	}
	g_DA_z = 0;
	setPIDChannel(10);
	setPIDSetPoint(0);
//	PID_LIST[0] = PID_function00;
//	PID_LIST[1] = PID_function01;
//	PID_LIST[2] = PID_function02;
//	PID_LIST[3] = PID_function03;
//	PID_LIST[4] = PID_function04;
}
*/
void catcher_SIGPIPE()
{
	// ���������ж�
	// �������ƺ�������
	// �˺�����ϵͳ���õĻ���ǳ���С
	r_close(connect_socket_fd);
//	exit(0);
}


