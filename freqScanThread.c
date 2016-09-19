#include "work_thread.h"
#include "hardware.h"
#include <string.h>


//----ɨƵ����------
static unsigned int freqScanStartFreq;
static unsigned int freqScanEndFreq;

static short freqScanAmpBuf[518];
static short freqScanPhaseBuf[518];

/**
*@brief  ����ɨƵƵ�ʷ�Χ
*@param  startFreq(unsigned int) ��ʼƵ��*100
*@param  endFreq(unsigned int) ��ֹƵ��*100
*@return  void
*/
void setFreqRange(unsigned int startFreq, unsigned int endFreq)
{
	freqScanStartFreq = startFreq;
	freqScanEndFreq = endFreq;
}


/**
*@brief  ɨƵ�߳�
*@param  para(struct command *) ͨ��ѡ���
*@return  void*
*/
void* freqScanThread(void* para)
{
	int i;
	unsigned char hasAmpChannel, hasPhaseChannel;
	struct command *pCmd = (struct command *) para;

	hasAmpChannel = ((pCmd->para1 & 0x01) == 0x01);
	hasPhaseChannel = ((pCmd->para1 & 0x02) == 0x02);
	short delay = ((pCmd->para2) & 0xffff);
	
	if(hasAmpChannel) DEBUG0("hasAmpChannel");
	if(hasPhaseChannel) DEBUG0("hasPhaseChannel");
	pthread_detach(pthread_self());//��Ϊ�����߳�
	
	double startFreq = (double) freqScanStartFreq / 100.0;
	double endFreq = (double) freqScanEndFreq / 100.0;
	
	int currentTask;
	double currentFreq;
/************************************************************************/
	DEBUG0("in freq scan thread");
	DEBUG1("start freq = %f\n", startFreq);
	DEBUG1("end freq = %f\n", endFreq);
	DEBUG1("delay = %d\n", (int)delay);
	DEBUG0("STARTING FREQENCY SCANNING ..................................");
/************************************************************************/
	
//	memcpy(freqScanAmpBuf, pCmd, sizeof(struct command));
//	memcpy(freqScanPhaseBuf, pCmd, sizeof(struct command));
	freqScanAmpBuf[0] = CMD_FREQ_SCAN;
	freqScanPhaseBuf[0] = CMD_FREQ_SCAN;
	freqScanAmpBuf[1] = 1024;
	freqScanPhaseBuf[1] = 1024;
	freqScanAmpBuf[2] = 512;
	freqScanPhaseBuf[2] = 512;
	freqScanAmpBuf[3] = 0x01;
	freqScanPhaseBuf[3] = 0x02;

	for(i = 0; i < 512; i++)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex);
		if(currentTask == STOP) break;
		currentFreq = startFreq + i * (endFreq - startFreq) / 512.0;
		dds_Out(currentFreq);
		udelay(delay);
		if(hasAmpChannel) freqScanAmpBuf[6 + i] = read_AD_N( AD_OSAP, 30 );//�Ժ������Ҫ����
		if(hasPhaseChannel) freqScanPhaseBuf[6 + i] = read_AD_N( AD_PHASE, 30 );
	}
	
	if(hasAmpChannel)
	{
		send(connect_socket_fd, (char*)freqScanAmpBuf, 1036, 0);
		DEBUG0("send(connect_socket_fd, (char*)&freqScanAmpBuf, 1036, 0);");
	}
	if(hasPhaseChannel)
	{
		send(connect_socket_fd, (char*)freqScanPhaseBuf, 1036, 0);
		DEBUG0("send(connect_socket_fd, (char*)&freqScanPhaseBuf, 1036, 0);");
	}
	pthread_mutex_lock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	DEBUG0("End of freq scan thread ......");
	return (void*)0;
}

