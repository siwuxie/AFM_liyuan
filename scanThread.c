#include <math.h>

#include "work_thread.h"
#include "hardware.h"
#include "closeloop.h"

#define XY_DELAY 20

//----扫描数据存储区---
static unsigned short topoBuf[518];
static unsigned short errorBuf[518];
static unsigned short phaseBuf[518];
static unsigned short fictionBuf[518];
//static unsigned short psdA_Buf[518];
//static unsigned short psdB_Buf[518];
//static unsigned short psdC_Buf[518];
//static unsigned short psdD_Buf[518];
//unsigned short topoBuf_retrace[518];
//unsigned short errorBuf_retrace[518];
//unsigned short phaseBuf_retrace[518];

//----扫描参数---
static int scanRangeX, scanRangeY, scanOffsetX, scanOffsetY;
static int scanAngle;
static int scanPixels;
static int scanDelays;
static int scanSamplingTimes;

//----通道选择---
static int hasInv = 0;
static int hasError = 0, hasPhase = 0, hasOther = 0, hasFiction = 0;
//static int hasPsdA = 0, hasPsdB = 0, hasPsdC = 0, hasPsdD = 0;

static int hysteresisH = 0;

static double corr_x[512];
static double corr_y[512];
void setHysteresisPara(int a, int b)
{
//	hysteresisM = m;
	hysteresisH = h;
	DEBUG1("hysteresisH is %d\n", hysteresisH);
//	if(m == 0)
//	{
//		hysteresisM = 1;
//	}
}

/**
*@brief  初始化扫描数据存储区
*@param  cmd(unsigned short) 扫描命令
*@param  size(unsigned short) 附加数据大小（字节）
*@return  void
*/
void initScanBuf(unsigned short cmd, unsigned short size)
{
	topoBuf[0] = cmd;
	topoBuf[1] = size;
	topoBuf[3] = 0; //通道号
	
	errorBuf[0] = cmd;
	errorBuf[1] = size;
	errorBuf[3] = 1;//通道号
	
	phaseBuf[0] = cmd;
	phaseBuf[1] = size;
	phaseBuf[3] = 2;//通道号
	
	fictionBuf[0] = cmd;
	fictionBuf[1] = size;
	fictionBuf[3] = 4;//通道号
	
//	psdA_Buf[0] = cmd;
//	psdA_Buf[1] = size;	
//	psdA_Buf[3] = 5;//通道号
	
//	psdB_Buf[0] = cmd;
//	psdB_Buf[1] = size;
//	psdB_Buf[3] = 6;//通道号

//	psdC_Buf[0] = cmd;
//	psdC_Buf[1] = size;
//	psdC_Buf[3] = 7;//通道号
	
//	psdD_Buf[0] = cmd;
//	psdD_Buf[1] = size;
//	psdD_Buf[3] = 8;//通道号
//	errorBuf[3] = 1;	
}


/**
*@brief  设置 AD 采样次数（不包括反馈控制中的 AD）
*@param  times(int) AD 采样次数
*@return  void
*/
void setScanSamplingTimes(int times)
{
	scanSamplingTimes = times;
	DEBUG1("scanSamplingTimes is %d\n", scanSamplingTimes);
}


/**
*@brief  设置 扫描延时（不包括反馈控制中的 延时）
*@param  delay(int) 延时（us）
*@return  void
*/
void setScanDelay(int delay)
{
	scanDelays = delay;
	DEBUG1("scanDelays is %d\n", scanDelays);
}


/**
*@brief  设置 扫描点数
*@param  XDots(int) X 方向扫描点数
*@param  YDots(int) Y 方向扫描点数
*@return  void
*/
void setScanPixel(int XDots, int YDots)
{
	scanPixels = (XDots > YDots) ? XDots : YDots;
	DEBUG1("scanPixels is %d\n", scanPixels);
}


/**
*@brief  设置 扫描范围等参数
*@param  rangeX(int) X 方向扫描范围
*@param  rangeY(int) Y 方向扫描范围
*@param  offsetX(int) X 方向偏移
*@param  offsetY(int) Y 方向偏移
*@param  angle(int) 旋转角度
*@return  void
*/
void setScanRange(int rangeX, int rangeY, int offsetX, int offsetY, int angle)
{
	scanRangeX = rangeX;
	scanRangeY = rangeY;
	scanOffsetX = offsetX;
	scanOffsetY = offsetY;
	scanAngle = angle;
	DEBUG0("scanRanges have been changed");
	DEBUG1("scanRangeX is %d\n", scanRangeX);
	DEBUG1("scanRangeY is %d\n", scanRangeY);
	DEBUG1("scanOffsetX is %d\n", scanOffsetX);
	DEBUG1("scanOffsetY is %d\n", scanOffsetY);
	DEBUG1("scanAngle is %d\n", scanAngle);
}


/**
*@brief  更新行号
*@param  line(short) 行号，从 1 开始
*@return  void*
*/
void updateLineNum(int line)
{
		topoBuf[2] = line;
		phaseBuf[2] = line;
		errorBuf[2] = line;
//		psdA_Buf[2] = line;
//		psdB_Buf[2] = line;
//		psdC_Buf[2] = line;
//		psdD_Buf[2] = line;	
}


/**
*@brief  检查开启的通道
*@param  para(COMMAND *) 通道选择
*@return  void*
*/
void updateChannels(COMMAND * para)
{
	unsigned short channels; //通道
	hasInv = para->para1;
	channels = para->para2;
	
	hasError = 0;
	hasPhase = 0;
	hasPhase = 0;
	hasFiction = 0;
	
	if( channels & (1 << 1) ) {hasError = 1;DEBUG0("hasError");}
	if( channels & (1 << 2) ) {hasPhase= 1;DEBUG0("hasPhase");}
	if( channels & (1 << 3) ) {hasOther = 1;DEBUG0("hasOther");}
	if( channels & (1 << 4) ) {hasFiction = 1;DEBUG0("hasFiction");}
//	if( channels & (1 << 5) ) {hasPsdA = 1;DEBUG0("hasPsdA");}
//	if( channels & (1 << 6) ) {hasPsdB = 1;DEBUG0("hasPsdB");}
//	if( channels & (1 << 7) ) {hasPsdC = 1;DEBUG0("hasPsdC");}
//	if( channels & (1 << 8) ) {hasPsdD = 1;	DEBUG0("hasPsdD");}
}

void sendData(void)
{
	send(connect_socket_fd, (char*)topoBuf, 1036, 0);DEBUG0("send topoBuf");
	if(hasError) {send(connect_socket_fd, (char*)errorBuf, 1036, 0);DEBUG0("send errorBuf");}
	if(hasFiction) {send(connect_socket_fd, (char*)fictionBuf, 1036, 0);DEBUG0("send fictionBuf");}
	if(hasPhase) {send(connect_socket_fd, (char*)phaseBuf, 1036, 0);DEBUG0("send phaseBuf");}
//		if(hasPsdA) send(connect_socket_fd, (char*)psdA_Buf, 1036, 0);
//		if(hasPsdB) send(connect_socket_fd, (char*)psdB_Buf, 1036, 0);
//		if(hasPsdC) send(connect_socket_fd, (char*)psdC_Buf, 1036, 0);
//		if(hasPsdD) send(connect_socket_fd, (char*)psdD_Buf, 1036, 0);
}
/**
*@brief  快速扫描线程（未完成）
*@param  para(void*) 通道选择
*@return  void*
*/
void* fastScanThread(void* para)
{
	pthread_detach(pthread_self());//成为自由线程

	//检查开启的通道
	updateChannels(para);
	//通道检查完成
	
	return (void*)0;
}

/**
*@brief  线扫描线程
*@param  para(void*) 通道选择
*@return  void*
*/
void* lineScanThread(void* para)
{
	int i, delay, line, h;
	int dots = 512;
	
	int currentTask;
	
	int rangeX, rangeY, offsetX, offsetY, samplingTimes;
	float angle, cosAngle, sinAngle;
	float stepX, stepY, tempX, tempY, ii;
	int posX, posY;

	DEBUG0("Line scan thread start.");
	pthread_detach(pthread_self());//成为自由线程
//	DEBUG0("Line scan thread start1.");
	//检查开启的通道
	updateChannels(para);
//	DEBUG0("Line scan thread start2.");
	line = ( (COMMAND *)para )->para4;//扫描第几行, 从 1 开始
	line --;//使其变成从零开始的行数
	
	//更新参数
	pthread_mutex_lock(&g_current_task_mutex);
	dots = scanPixels;// 扫描点数每次扫描都不变，因此只更新一次
	delay = scanDelays;
	rangeX = scanRangeX;
	rangeY = scanRangeY;
	offsetX = scanOffsetX;
	offsetY = scanOffsetY;
	samplingTimes = scanSamplingTimes;
	angle = PI * scanAngle / 180.0;
	h = hysteresisH;
	currentTask = g_current_task;
	pthread_mutex_unlock(&g_current_task_mutex); 

	initScanBuf(CMD_LINESCAN_START, dots * 2);
//DEBUG0("Line scan thread start3.");	
	if(currentTask == STOP)
	{
		goto lineScanExit; 
	}	
	//更新参数完成	

//Added in 2008.01.12 by liyuan
//将探针移动到扫描区域的起始点
	cosAngle = cos(angle);
	sinAngle = sin(angle);
	stepX = (float)rangeX / (dots - 1);
	stepY = (float)rangeY / (dots - 1);	
	tempX = (stepX * 0 + offsetX - rangeX / 2);
	tempY = (stepY * line + offsetY - rangeY / 2);
	posX = tempX * cosAngle + tempY * sinAngle;
	posY = -tempX * sinAngle + tempY * cosAngle;
	
	for(i =0 ;i < 100; i++)
	{
			write_DA(DA_X, 32768 + i * posX / 100);
			write_DA(DA_Y, 32768 + i * posY / 100);
//			udelay(delay);
			pid_func();
	}
//	DEBUG0("Line scan thread start4.");
/////////////////////////////////		
//开始正式行扫描
	while(1)
	{
		for(i = 0; i < dots; i++)
		{
			ii = i + i * h / 128.0 - h / 256.0 / 255.0 * i * i;
			tempX = (stepX * ii + offsetX - rangeX / 2);
			tempY = (stepY * line + offsetY - rangeY / 2);
			posX = 32768 + tempX * cosAngle + tempY * sinAngle;
			posY = 32768 - tempX * sinAngle + tempY * cosAngle;
			
			write_DA(DA_X, posX);
			write_DA(DA_Y, posY);
//			udelay(delay);
			errorBuf[i + 6] = pid_func(); // PID 控制函数返回值就是控制误差
			topoBuf[i + 6] = g_DA_z;
			errorBuf[i + 6] =read_AD_N(AD_ERROR, samplingTimes);
			if(hasFiction) fictionBuf[i + 6] = read_AD_N(AD_FICTION, samplingTimes);
			if(hasPhase) phaseBuf[i + 6] = read_AD_N(AD_PHASE, samplingTimes);
//			if(hasPsdA) psdA_Buf[i + 6] = read_AD_N(AD_LASER_A, samplingTimes);
//			if(hasPsdB) psdB_Buf[i + 6] = read_AD_N(AD_LASER_B, samplingTimes);
//			if(hasPsdC) psdC_Buf[i + 6] = read_AD_N(AD_LASER_C, samplingTimes);
//			if(hasPsdD) psdD_Buf[i + 6] = read_AD_N(AD_LASER_D, samplingTimes);
		
		}
		updateLineNum(line + 1);
		sendData();
		DEBUG1("send one line data, line %d\n", line + 1);		
		
		//判断是否停止扫描
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex); 
		if(currentTask == STOP)
		{
			goto lineScanExit; 
		}
		
		//反扫循环
		for(i = dots - 1; i >= 0; i--)
		{
//			ii = i - i * h / 128 + h / 256 / 255 * i * i;
			ii = i - i * h / 128.0 + h / 256.0 / 255.0 * i * i;
			tempX = (stepX * ii + offsetX - rangeX / 2);
			tempY = (stepY * line + offsetY - rangeY / 2);
			posX = 32768 + tempX * cosAngle + tempY * sinAngle;
			posY = 32768 - tempX * sinAngle + tempY * cosAngle;

			write_DA(DA_X, posX);
			write_DA(DA_Y, posY);
//			udelay(delay);
			errorBuf[i + 6] = pid_func();
			topoBuf[i + 6] = g_DA_z;
			errorBuf[i + 6] =read_AD_N(AD_ERROR, samplingTimes);
			if(hasInv) 
			{
				if(hasFiction) fictionBuf[i + 6] = read_AD_N(AD_FICTION, samplingTimes);
				if(hasPhase) phaseBuf[i + 6] = read_AD_N(AD_PHASE, samplingTimes);
//				if(hasPsdA) psdA_Buf[i + 6] = read_AD_N(AD_LASER_A, samplingTimes);
//				if(hasPsdB) psdB_Buf[i + 6] = read_AD_N(AD_LASER_B, samplingTimes);
//				if(hasPsdC) psdC_Buf[i + 6] = read_AD_N(AD_LASER_C, samplingTimes);
//				if(hasPsdD) psdD_Buf[i + 6] = read_AD_N(AD_LASER_D, samplingTimes);	
			}
		}
		updateLineNum( -line - 1);
		if(hasInv) 
		{
			sendData();
		}
		DEBUG1("Send one line data, line %d\n", -line - 1);		

		//更新参数，准备下一行扫描
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		delay = scanDelays;
		rangeX = scanRangeX;
		rangeY = scanRangeY;
		offsetX = scanOffsetX;
		offsetY = scanOffsetY;
		samplingTimes = scanSamplingTimes;
		angle = PI * scanAngle / 180.0;
		h = hysteresisH;
		pthread_mutex_unlock(&g_current_task_mutex); 	
			
		if(currentTask == STOP)
		{
			goto lineScanExit; 
		}	
		//根据更新参数计算步距，正反扫周期只计算一次
		cosAngle = cos(angle);
		sinAngle = sin(angle);
		stepX = (float)rangeX / (dots - 1);
		stepY = (float)rangeY / (dots - 1);	
	}
lineScanExit:
	
	//将探针归位于中心位置	
	for(i =100 ;i >= 0; i--)
	{
			write_DA(DA_X, 32768 + i * (posX - 32768) / 100);
			write_DA(DA_Y, 32768 + i * (posY - 32768) / 100);
//			udelay(delay);
			pid_func();
	}	
	g_DA_z = 65535;
	pid_func();
	pthread_mutex_lock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	DEBUG0("line scan stoped");
	return (void*) 0;
}


void* normalScanThread(void* para)
{
	int i, row, delay;
	int dots;
	int currentTask;
	int rangeX, rangeY, offsetX, offsetY, samplingTimes;
	float angle, cosAngle, sinAngle;
	float stepX, stepY, tempX, tempY, ii, h;
	int posX, posY;
	
	DEBUG0("Normal scan thread start.");
	pthread_detach(pthread_self());//成为自由线程

	//检查开启的通道
	updateChannels(para);

	//更新参数
	pthread_mutex_lock(&g_current_task_mutex);
		dots = scanPixels;// 扫描点数每次扫描都不变，因此只更新一次
		delay = scanDelays;
		rangeX = scanRangeX;
		rangeY = scanRangeY;
		offsetX = scanOffsetX;
		offsetY = scanOffsetY;
		samplingTimes = scanSamplingTimes;
		angle = PI * scanAngle / 180.0;
		h = hysteresisH;
		currentTask = g_current_task;
	pthread_mutex_unlock(&g_current_task_mutex); 

	initScanBuf(CMD_SCAN_WHOLE, dots * 2);
	if(currentTask == STOP)
	{
		goto normalScanExit; 
	}	
	//更新参数完成	
	

//Added in 2008.01.12 by liyuan
//将探针移动到扫描区域的起始点
	cosAngle = cos(angle);
	sinAngle = sin(angle);
	stepX = (float)rangeX / (dots - 1);
	stepY = (float)rangeY / (dots - 1);	
	tempX = (stepX * 0 + offsetX - rangeX / 2);
	tempY = (stepY * 0 + offsetY - rangeY / 2);
	posX = tempX * cosAngle + tempY * sinAngle;
	posY = -tempX * sinAngle + tempY * cosAngle;
	
//	write_DA(DA_Z, 65535);
	for(i =0 ;i < 100; i++)
	{
			write_DA(DA_X, 32768 + i * posX / 100);
			write_DA(DA_Y, 32768 + i * posY / 100);
			udelay(delay);
			pid_func();
	}
/////////////////////////////////	
//开始正式扫图
	for (row = 0; row < dots; row++) // 当前行
	{
		//正扫循环
		for(i = 0; i < dots; i++)
		{
			ii = i + i * h / 128.0 - h / 256.0 / 255.0 * i * i;
			tempX = (stepX * ii + offsetX - rangeX / 2);
			tempY = (stepY * row + offsetY - rangeY / 2);
			posX = 32768 + tempX * cosAngle + tempY * sinAngle;
			posY = 32768 - tempX * sinAngle + tempY * cosAngle;
			
			write_DA(DA_X, posX);
			write_DA(DA_Y, posY);
	//		udelay(delay);
			errorBuf[i + 6] = pid_func(); // PID 控制函数
			topoBuf[i + 6] = g_DA_z;
			errorBuf[i + 6] =read_AD_N(AD_ERROR, samplingTimes);
			if(hasFiction) fictionBuf[i + 6] = read_AD_N(AD_FICTION, samplingTimes);
			if(hasPhase) phaseBuf[i + 6] = read_AD_N(AD_PHASE, samplingTimes);
//			if(hasPsdA) psdA_Buf[i + 6] = read_AD_N(AD_LASER_A, samplingTimes);
//			if(hasPsdB) psdB_Buf[i + 6] = read_AD_N(AD_LASER_B, samplingTimes);
//			if(hasPsdC) psdC_Buf[i + 6] = read_AD_N(AD_LASER_C, samplingTimes);
//			if(hasPsdD) psdD_Buf[i + 6] = read_AD_N(AD_LASER_D, samplingTimes);
		
		}
		updateLineNum(row + 1);
		
		sendData();
		DEBUG1("send one line data, line %d\n", row+1);		
		
		//判断是否停止扫描
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex); 
		if(currentTask == STOP)
		{
			goto normalScanExit; 
		}
		
		//反扫循环
		for(i = dots - 1; i >= 0; i--)
		{
			ii = i - i * h / 128.0 + h / 256.0 / 255.0 * i * i;
			tempX = (stepX * ii + offsetX - rangeX / 2);
			tempY = (stepY * row + offsetY - rangeY / 2);
			posX = 32768 + tempX * cosAngle + tempY * sinAngle;
			posY = 32768 - tempX * sinAngle + tempY * cosAngle;

			write_DA(DA_X, posX);
			write_DA(DA_Y, posY);
//			udelay(delay);
			errorBuf[i + 6] = pid_func();
			topoBuf[i + 6] = g_DA_z;
			if(hasInv) 
			{
				errorBuf[i + 6] =read_AD_N(AD_ERROR, samplingTimes);
				if(hasFiction) fictionBuf[i + 6] = read_AD_N(AD_FICTION, samplingTimes);
				if(hasPhase) phaseBuf[i + 6] = read_AD_N(AD_PHASE, samplingTimes);
//				if(hasPsdA) psdA_Buf[i + 6] = read_AD_N(AD_LASER_A, samplingTimes);
//				if(hasPsdB) psdB_Buf[i + 6] = read_AD_N(AD_LASER_B, samplingTimes);
//				if(hasPsdC) psdC_Buf[i + 6] = read_AD_N(AD_LASER_C, samplingTimes);
//				if(hasPsdD) psdD_Buf[i + 6] = read_AD_N(AD_LASER_D, samplingTimes);	
			}
		}
		updateLineNum( -row - 1);
		
		if(hasInv) 
		{
			sendData();
		}
		
		DEBUG1("Send one line data, line %d\n", -row - 1);		

		//更新参数，准备下一行扫描
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		delay = scanDelays;
		rangeX = scanRangeX;
		rangeY = scanRangeY;
		offsetX = scanOffsetX;
		offsetY = scanOffsetY;
		samplingTimes = scanSamplingTimes;
		h = hysteresisH;
		angle = PI * scanAngle / 180.0;
		pthread_mutex_unlock(&g_current_task_mutex); 	
			
		if(currentTask == STOP)
		{
			goto normalScanExit; 
		}	
		//根据更新参数计算步距，正反扫周期只计算一次
		cosAngle = cos(angle);
		sinAngle = sin(angle);
		stepX = (float)rangeX / (dots - 1);
		stepY = (float)rangeY / (dots - 1);	
	}

normalScanExit:		
	//将探针归位于中心位置	
	for(i =100 ;i >= 0; i--)
	{
			write_DA(DA_X, 32768 + i * (posX - 32768) / 100);
			write_DA(DA_Y, 32768 + i * (posY - 32768) / 100);
//			udelay(delay);
			pid_func();
	}	
	pthread_mutex_lock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	DEBUG0("Leave normal scan thread");		
	return (void*) 0;
}



