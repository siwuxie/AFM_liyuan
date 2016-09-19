#include "work_thread.h"
#include "hardware.h"

/*
力曲线
上位机向下位机发送命令:

cmd = CMD_FORCE_CURVE	: 0x2A
para1 = z 下限
para2 = z 上限
para3 = 点数
para4 = delay
para5 = channel

下位机向上位机传回数据：（每点传回一次数据）

cmd = CMD_FORCE_CURVE	: 0x2A
para1 = 0
para2 = 当前点 （回程时为负数）1,2,3,4... , -1, -2, -3, ...
para3 = Z
para4 = 通道号
para5 = 数据
*/
void *forceCurveThread(void *para)
{
	int zStart, zEnd, totalSteps, delay, j, i;
	unsigned short channel, zOld;
	float stride;//每一步的步距
	short adValue;
	int adValueTotal;
	int zCurrent;
	int currentTask;
	int currentStep = 1;

	COMMAND *pCmd = (COMMAND *)para;
	RESPONSE resp;
	
	pthread_detach(pthread_self());//成为自由线程
	
	zOld = g_DA_z;
	zStart = g_DA_z + (unsigned short) (pCmd->para1);
	zEnd = g_DA_z - (unsigned short) (pCmd->para2);
	if(zStart > 65535) zStart = 65535;
	if(zEnd < 0) zEnd = 0;
	
	totalSteps = pCmd->para3;
	if(totalSteps < 2) totalSteps = 2;
		
	delay = (unsigned short) (pCmd->para4);
	channel = (unsigned short) (pCmd->para5);
	
	DEBUG1("z start: %d\n", zStart);
	DEBUG1("z end: %d\n", zEnd);
	DEBUG1("total strides: %d\n", totalSteps);
	DEBUG1("delay: %d\n", delay);
	DEBUG1("channel: %d\n", channel);
	stride = (float)(zStart - zEnd) / totalSteps;
	
	resp.cmd = CMD_FORCE_CURVE;
	resp.para1 = 0;
	//正程
	for(currentStep = 1; currentStep <= totalSteps; currentStep++)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex); 
		if(currentTask == STOP)
		{
			goto GET_OUT; 
		}
		resp.para2 = currentStep;// 当前的点数
		zCurrent = zStart - (currentStep - 1) * stride;
		g_DA_z = zCurrent; 
		write_DA( DA_Z, g_DA_z);
		memcpy(&(resp.para3), &zCurrent, 2);// 当前的Ｚ向高度

		udelay(delay);//delay
		
		for(i = 0; i < 16; i++)
		{
			if(channel & (1 << i))
			{
				adValueTotal = 0;
				for(j = 0; j < 10; j++)
				{
					adValueTotal += read_AD(i);
				}
				adValue = adValueTotal / 10;
				resp.para4 = i; // 当前的通道号
				resp.para5 = adValue; //采集的数据
				DEBUG2("z = %d, error = %d\n", (unsigned short)resp.para3, resp.para5);
				send(connect_socket_fd, (char*)&resp, 12, 0);
			}
		}
	}
	DEBUG0("force_curve_thread, forward scan finish");
	// 返程
	for(currentStep = totalSteps; currentStep >= 1; currentStep--)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex); 
		if(currentTask == STOP)
		{
			goto GET_OUT; 
		}
		resp.para2 = currentStep - totalSteps - 1;// 当前的点数
		zCurrent = zStart - (currentStep - 1) * stride;
		g_DA_z = zCurrent; 
		write_DA( DA_Z, g_DA_z);
		memcpy(&(resp.para3), &zCurrent, 2);// 当前的Ｚ向高度
		
		udelay(delay);//delay
		
		for(i = 0; i < 16; i++)
		{
			if(channel & (1 << i))
			{
				adValueTotal = 0;
				for(j = 0; j < 10; j++)
				{
					adValueTotal += read_AD(i);
				}
				adValue = adValueTotal / 10;
				resp.para4 = i; // 当前的通道号
				resp.para5 = adValue; //采集的数据
				DEBUG2("z = %d, error = %d\n", (unsigned short)resp.para3, resp.para5);
				send(connect_socket_fd, (char*)&resp, 12, 0);
			}
		}
	}
	DEBUG0("force_curve_thread, back scan finish");	
	
GET_OUT:
	g_DA_z = zOld;
	write_DA( DA_Z, g_DA_z);
	pthread_mutex_lock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	DEBUG0("leave force_curve_thread");	
	return (void*) 0;	
}

