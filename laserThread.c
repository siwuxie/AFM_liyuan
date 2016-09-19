#include <unistd.h>

#include "work_thread.h"
#include "hardware.h"

unsigned short X_data_trace[512 * 512];
unsigned short X_data_retrace[512 * 512];
unsigned short Y_data_trace[512 * 512];
void readHugeData(void)
{
	int len;
	DEBUG0("IN readHugeData");
	len = recv(connect_socket_fd, (char *)X_data_trace, 512 * 512 * 2, MSG_WAITALL);
	len = recv(connect_socket_fd, (char *)X_data_retrace, 512 * 512 * 2, MSG_WAITALL);
	len = recv(connect_socket_fd, (char *)Y_data_trace, 512 * 512 * 2, MSG_WAITALL);
	DEBUG0("ALL DATA RECEIVED");
}

unsigned short sendBuf[8];
void* laserThread(void* para)
{
	int i, laserA, laserB, laserC, laserD;
	int currentTask;
	pthread_detach(pthread_self());//成为自由线程
	
	bzero(sendBuf, 8 * sizeof(unsigned short));
	
	sendBuf[0] = GET_LASER_POS;
	sendBuf[1] = 4;
	DEBUG0("in laser thread");
	
	while(1)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex); 
		if(currentTask == STOP) break;
		pid_func();
		for (i = 0; i < 50; i++)
		{
			pid_func();
		}
		
		laserA = 0;
		laserB = 0;
		laserC = 0;
		laserD = 0;
		for (i = 0; i < 50; i++)
		{
			laserA += read_AD(AD_LASER_A);
			laserB += read_AD(AD_LASER_B);
			laserC += read_AD(AD_LASER_C);
			laserD += read_AD(AD_LASER_D);
		}
		sendBuf[2] = laserA / 50;
		sendBuf[3] = laserB / 50;
		sendBuf[4] = laserC / 50;
		sendBuf[5] = laserD / 50;	
			
//		sendBuf[2] = read_AD_N(AD_LASER_A, 30);
//		sendBuf[3] = read_AD_N(AD_LASER_B, 30);
//		sendBuf[4] = read_AD_N(AD_LASER_C, 30);
//		sendBuf[5] = read_AD_N(AD_LASER_D, 30);
//		sendBuf[7] = read_AD_N(AD_ERROR, 30);
		sendBuf[7] = pid_func();
		sendBuf[6] = (short)g_DA_z;
		send(connect_socket_fd, (char*)sendBuf, 16, 0);
		//DEBUG0("Send laser pos...");
		//printf("a:%d, b:%d, c:%d, d:%d \n", laserA / 100, laserB / 100, laserC / 100, laserD / 100);
		usleep(10000);
	}
 
	DEBUG0("leave laser thread");
	return 0;
}
