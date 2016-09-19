#include <math.h>

#include "work_thread.h"
#include "hardware.h"
#include "closeloop.h"

#define XY_DELAY 20

//----扫描数据存储区---
static short topoBuf[518];
static short errorBuf[518];
static short phaseBuf[518];
static short fictionBuf[518];
static short psdA_Buf[518];
static short psdB_Buf[518];
static short psdC_Buf[518];
static short psdD_Buf[518];
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
static int hasPsdA = 0, hasPsdB = 0, hasPsdC = 0, hasPsdD = 0;

/**
*@brief  初始化扫描数据存储区
*@param  cmd(unsigned short) 扫描命令
*@param  size(unsigned short) 附加数据大小（字节）
*@return  void
*/
void initScanBuf ( unsigned short cmd, unsigned short size )
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

    psdA_Buf[0] = cmd;
    psdA_Buf[1] = size;
    psdA_Buf[3] = 5;//通道号

    psdB_Buf[0] = cmd;
    psdB_Buf[1] = size;
    psdB_Buf[3] = 6;//通道号

    psdC_Buf[0] = cmd;
    psdC_Buf[1] = size;
    psdC_Buf[3] = 7;//通道号

    psdD_Buf[0] = cmd;
    psdD_Buf[1] = size;
    psdD_Buf[3] = 8;//通道号
//	errorBuf[3] = 1;
}


/**
*@brief  设置 AD 采样次数（不包括反馈控制中的 AD）
*@param  times(int) AD 采样次数
*@return  void
*/
void setScanSamplingTimes ( int times )
{
    scanSamplingTimes = times;
    DEBUG1 ( "scanSamplingTimes is %d\n", scanSamplingTimes );
}


/**
*@brief  设置 扫描延时（不包括反馈控制中的 延时）
*@param  delay(int) 延时（us）
*@return  void
*/
void setScanDelay ( int delay )
{
    scanDelays = delay;
    DEBUG1 ( "scanDelays is %d\n", scanDelays );
}


/**
*@brief  设置 扫描点数
*@param  XDots(int) X 方向扫描点数
*@param  YDots(int) Y 方向扫描点数
*@return  void
*/
void setScanPixel ( int XDots, int YDots )
{
    scanPixels = ( XDots > YDots ) ? XDots : YDots;
    DEBUG1 ( "scanPixels is %d\n", scanPixels );
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
void setScanRange ( int rangeX, int rangeY, int offsetX, int offsetY, int angle )
{
    scanRangeX = rangeX;
    scanRangeY = rangeY;
    scanOffsetX = offsetX;
    scanOffsetY = offsetY;
    scanAngle = angle;
    DEBUG0 ( "scanRanges have been changed" );
    DEBUG1 ( "scanRangeX is %d\n", scanRangeX );
    DEBUG1 ( "scanRangeY is %d\n", scanRangeY );
    DEBUG1 ( "scanOffsetX is %d\n", scanOffsetX );
    DEBUG1 ( "scanOffsetY is %d\n", scanOffsetY );
    DEBUG1 ( "scanAngle is %d\n", scanAngle );
}


/**
*@brief  更新行号
*@param  line(short) 行号，从 1 开始
*@return  void*
*/
void updateLineNum ( int line )
{
    topoBuf[2] = line;
    phaseBuf[2] = line;
    errorBuf[2] = line;
    fictionBuf[2] = line;
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
void updateChannels ( COMMAND * para )
{
    unsigned short channels; //通道
    hasInv = para->para1;
    if ( hasInv ) DEBUG0 ( "hasInv" );
    channels = para->para2;

    hasError = 0;
    hasPhase = 0;
    hasPhase = 0;
    hasFiction = 0;

    if ( channels & ( 1 << 1 ) )
    {
        hasError = 1;
        DEBUG0 ( "hasError" );
    }
    if ( channels & ( 1 << 2 ) )
    {
        hasPhase = 1;
        DEBUG0 ( "hasPhase" );
    }
    if ( channels & ( 1 << 3 ) )
    {
        hasOther = 1;
        DEBUG0 ( "hasOther" );
    }
    if ( channels & ( 1 << 4 ) )
    {
        hasFiction = 1;
        DEBUG0 ( "hasFiction" );
    }
//	if( channels & (1 << 5) ) {hasPsdA = 1;DEBUG0("hasPsdA");}
//	if( channels & (1 << 6) ) {hasPsdB = 1;DEBUG0("hasPsdB");}
//	if( channels & (1 << 7) ) {hasPsdC = 1;DEBUG0("hasPsdC");}
//	if( channels & (1 << 8) ) {hasPsdD = 1;	DEBUG0("hasPsdD");}
}

void sendData ( void )
{
    send ( connect_socket_fd, ( char* ) topoBuf, 1036, 0 );
    DEBUG0 ( "send topoBuf" );
    if ( hasError )
    {
        send ( connect_socket_fd, ( char* ) errorBuf, 1036, 0 );
        DEBUG0 ( "send errorBuf" );
    }
    if ( hasFiction )
    {
        send ( connect_socket_fd, ( char* ) fictionBuf, 1036, 0 );
        DEBUG0 ( "send fictionBuf" );
    }
    if ( hasPhase )
    {
        send ( connect_socket_fd, ( char* ) phaseBuf, 1036, 0 );
        DEBUG0 ( "send phaseBuf" );
    }
//		if(hasPsdA) {send(connect_socket_fd, (char*)psdA_Buf, 1036, 0);DEBUG0("send psdA_Buf");}
//		if(hasPsdB) {send(connect_socket_fd, (char*)psdB_Buf, 1036, 0);DEBUG0("send psdB_Buf");}
//		if(hasPsdC) {send(connect_socket_fd, (char*)psdC_Buf, 1036, 0);DEBUG0("send psdC_Buf");}
//		if(hasPsdD) {send(connect_socket_fd, (char*)psdD_Buf, 1036, 0);DEBUG0("send psdD_Buf");}
}
/**
*@brief  快速扫描线程（未完成）
*@param  para(void*) 通道选择
*@return  void*
*/
void* fastScanThread ( void* para )
{
    pthread_detach ( pthread_self() );//成为自由线程

    //检查开启的通道
    updateChannels ( para );
    //通道检查完成

    return ( void* ) 0;
}
int hcMethod = 0;
double hctc_xA = 0.0;//正扫x方向校正系数 A, -1 +1
double hctc_xB = 0.0;//正扫x方向校正系数 B, -1 +1
double hcrc_xA = 0.0;//反扫x方向校正系数 A, -1 +1
double hcrc_xB = 0.0;//反扫x方向校正系数 B, -1 +1
double hctc_yA = 0.0;//正扫y方向校正系数 A, -1 +1
double hctc_yB = 0.0;//正扫y方向校正系数 B, -1 +1
double hctc_x[512]; //hysteresis correction trace curve, x dircection
double hcrc_x[512];//hysteresis correction retrace curve, x dircection
double hctc_y[512];////hysteresis correction trace curve, y dircection

//用二次函数的反函数做非线性校正
double corr2 ( double y, double a ) //只对正扫描校正
{
    //非线性方程为 y = a x^2 - a + x
    double sq;
    if ( a == 0 ) return y;
    sq = sqrt ( 1 + 4 * a * ( a + y ) );
    if ( a > 0 ) //对应的是对正扫曲线的矫正，取大的根
        return 0.5 * ( -1 + sq ) / a;
    else //对应的是对反扫曲线的矫正，取小的根
        return 0.5 * ( -1 - sq ) / a;
}
//用三次函数的反函数做非线性校正
double corr3 ( double y, double a, double b )
{
    double x, f, ff;
    double i;
    if ( y > 1 ) y = 1;
    if ( y < -1 ) y = -1;

    //非线性方程为 y = a x^3 + b x^2 - a x - b + x
    x = y; //set the initial value of iteration
    for ( i = 0; i < 5; i++ ) //利用牛顿迭代法求方程的根
    {
        f = a * x * x * x + b * x * x - a * x - b + x - y;
        ff = 3 * a * x * x + 2 * b * x - a + 1;
        x = x - f / ff; // x = x - f / f'
    }
    return x;
}
void updateHCCurve ( void ) //更新非线性校正曲线
{
    int i;
    double y;
    if ( hcMethod == 0 ) // 不做非线性校正
    {
        for ( i = 0; i < 512; i++ )
        {
            y = 2.0 * i / 511.0 - 1;
            hctc_x[i] = hcrc_x[i] = hctc_y[i] = y;
        }
    }
    else if ( hcMethod == 1 ) //利用 2 次函数的反函数校正
    {
        for ( i = 0; i < 512; i++ )
        {
            y = 2.0 * i / 511.0 - 1;
            hctc_x[i] = corr2 ( y, hctc_xB );
            hcrc_x[i] = corr2 ( y, hcrc_xB );
            hctc_y[i] = corr2 ( y, hctc_yB );

        }
    }
    else if ( hcMethod == 2 ) // 利用三次函数的反函数校正
    {
        for ( i = 0; i < 512; i++ )
        {
            y = 2.0 * i / 511.0 - 1;
            hctc_x[i] = corr3 ( y, hctc_xA, hctc_xB );
            hcrc_x[i] = corr3 ( y, hcrc_xA, hcrc_xB );
            hctc_y[i] = corr3 ( y, hctc_yA, hctc_yB );
        }
    }
}
//
//int linearInterpolation(int startX, int startY, int endX, int endY, int *posX, int *posY, int i, int dots)
//{
//	int j = dots - i - 1;
//	*posX = (startX * j + endX * i) / 511;
//	*posY = (startY * j + endY * i) / 511;
//	return 0;
//}
//
//void setHysteresisPara(int m, int h)
//{
//	hysteresisM = m;
//	hysteresisH = h;
//	if(m == 0)
//	{
//		hysteresisM = 1;
//	}
//}
//
//int hysteresisCorrection01(int rangeX, int rangeY, int *posX, int *posY, int i, int direction)
//{
//	int h_x, h_y;
//	if(direction == 0)
//	{
//		h_x = hysteresisH * rangeX / 100;
//		h_y = hysteresisH * rangeY / 100;
//		i = 511 - i;
//	}
//	else
//	{
//
//		h_x = -hysteresisH * rangeX / 100;
//		h_y = -hysteresisH * rangeY / 100;
//	}
//
//	*posX += (h_x * i / 511);
//	*posY += (h_y * i / 511);
//
//	return 0;
//}
//int hysteresisCorrection02(int rangeX, int rangeY, int *posX, int *posY, int i, int direction)
//{
//	int m, h_x, h_y;
//	if(direction == 0)
//	{
//		m = hysteresisM;
//		h_x = hysteresisH * rangeX / 100;
//		h_y = hysteresisH * rangeY / 100;
//	}
//	else
//	{
//		m = 511 - hysteresisM;
//		h_x = -hysteresisH * rangeX / 100;
//		h_y = -hysteresisH * rangeY / 100;
//	}
//	if( i <= m)
//	{
//		*posX += (h_x * i / m);
//		*posY += (h_y * i / m);
//	}
//	else
//	{
//		*posX += (h_x * (511 - i) / (511 - m));
//		*posY += (h_y * (511 - i) / (511 - m));
//	}
//	return 0;
//}


/**
*@brief  线扫描线程
*@param  para(void*) 通道选择
*@return  void*
*/
void* lineScanThread ( void* para )
{
    int i, delay, line;
    int dots = 512;

    int currentTask;

    int rangeX, rangeY, offsetX, offsetY, samplingTimes;
    float angle, cosAngle, sinAngle;
    float  tempX, tempY;
    int posX, posY;

    DEBUG0 ( "Line scan thread start." );
    pthread_detach ( pthread_self() );//成为自由线程
//	DEBUG0("Line scan thread start1.");
    //检查开启的通道
    updateChannels ( para );
//	DEBUG0("Line scan thread start2.");
    line = ( ( COMMAND * ) para )->para4;//扫描第几行, 从 1 开始
    line --;//使其变成从零开始的行数

    //更新参数
    pthread_mutex_lock ( &g_current_task_mutex );
    dots = scanPixels;// 扫描点数每次扫描都不变，因此只更新一次
    delay = scanDelays;
    rangeX = scanRangeX;
    rangeY = scanRangeY;
    offsetX = scanOffsetX;
    offsetY = scanOffsetY;
    samplingTimes = scanSamplingTimes;
    angle = PI * scanAngle / 180.0;
    currentTask = g_current_task;
    pthread_mutex_unlock ( &g_current_task_mutex );

    initScanBuf ( CMD_LINESCAN_START, dots * 2 );

    if ( currentTask == STOP )
    {
        goto lineScanExit;
    }
    //更新参数完成

//Added in 2008.01.12 by liyuan
//将探针移动到扫描区域的起始点
    cosAngle = cos ( angle );
    sinAngle = sin ( angle );

    tempX = hctc_x[0] * rangeX / 2.0 + offsetX;
    tempY = hctc_y[line] * rangeY / 2.0 + offsetY;
    posX = tempX * cosAngle + tempY * sinAngle;
    posY = -tempX * sinAngle + tempY * cosAngle;

    for ( i = 0 ;i < 100; i++ )
    {
        write_DA ( DA_X, 32768 + i * posX / 100 );
        write_DA ( DA_Y, 32768 + i * posY / 100 );
//			udelay(delay);
        pid_func();
    }
/////////////////////////////////
//开始正式行扫描
    while ( 1 )
    {
        tempY = hctc_y[line] * rangeY / 2.0 + offsetY;//每行只需计算一次即可
        for ( i = 0; i < dots; i++ )
        {
            tempX = hctc_x[i] * rangeX / 2.0 + offsetX;
            posX = 32767 + tempX * cosAngle + tempY * sinAngle;
            posY = 32767 - tempX * sinAngle + tempY * cosAngle;
            write_DA ( DA_X, posX );
            write_DA ( DA_Y, posY );
            errorBuf[i + 6] = pid_func(); // PID 控制函数返回值就是控制误差
            topoBuf[i + 6] = ( short ) g_DA_z;
            errorBuf[i + 6] = read_AD_N ( AD_ERROR, samplingTimes );
            if ( hasFiction ) fictionBuf[i + 6] = read_AD_N ( AD_FICTION, samplingTimes );
            if ( hasPhase ) phaseBuf[i + 6] = read_AD_N ( AD_PHASE, samplingTimes );
        }
        updateLineNum ( line + 1 );
        sendData();
        DEBUG1 ( "send one line data, line %d\n", line + 1 );

        //判断是否停止扫描
        pthread_mutex_lock ( &g_current_task_mutex );
        currentTask = g_current_task;
        pthread_mutex_unlock ( &g_current_task_mutex );
        if ( currentTask == STOP )
        {
            goto lineScanExit;
        }

        //反扫循环
        for ( i = dots - 1; i >= 0; i-- )
        {
            tempX = hcrc_x[i] * rangeX / 2.0 + offsetX;
            posX = 32767 + tempX * cosAngle + tempY * sinAngle;
            posY = 32767 - tempX * sinAngle + tempY * cosAngle;
            write_DA ( DA_X, posX );
            write_DA ( DA_Y, posY );
//			udelay(delay);
            errorBuf[i + 6] = pid_func();
            topoBuf[i + 6] = ( short ) g_DA_z;
            errorBuf[i + 6] = read_AD_N ( AD_ERROR, samplingTimes );
            if ( hasInv )
            {
                if ( hasFiction ) fictionBuf[i + 6] = read_AD_N ( AD_FICTION, samplingTimes );
                if ( hasPhase ) phaseBuf[i + 6] = read_AD_N ( AD_PHASE, samplingTimes );
            }
        }
        updateLineNum ( -line - 1 );
        if ( hasInv )
        {
            sendData();
        }
        DEBUG1 ( "Send one line data, line %d\n", -line - 1 );

        //更新参数，准备下一行扫描
        pthread_mutex_lock ( &g_current_task_mutex );
        currentTask = g_current_task;
        delay = scanDelays;
        rangeX = scanRangeX;
        rangeY = scanRangeY;
        offsetX = scanOffsetX;
        offsetY = scanOffsetY;
        samplingTimes = scanSamplingTimes;
        angle = PI * scanAngle / 180.0;
        pthread_mutex_unlock ( &g_current_task_mutex );

        if ( currentTask == STOP )
        {
            goto lineScanExit;
        }
        //根据更新参数计算步距，正反扫周期只计算一次
        cosAngle = cos ( angle );
        sinAngle = sin ( angle );
    }
lineScanExit:


    write_DA ( DA_Z, 65535 );
    write_DA ( DA_X, 32768 );
    write_DA ( DA_Y, 32768 );
    g_DA_z = 65535;
    pid_func();
    pthread_mutex_lock ( &g_current_task_mutex );
    g_current_task = STOP;
    pthread_mutex_unlock ( &g_current_task_mutex );
    DEBUG0 ( "line scan stoped" );
    return ( void* ) 0;
}


void* normalScanThread ( void* para )
{
    int i, row, delay;
    int dots;
    int currentTask;
    int rangeX, rangeY, offsetX, offsetY, samplingTimes;
    float angle, cosAngle, sinAngle;
    float tempX, tempY;
    int posX, posY;

    DEBUG0 ( "Normal scan thread start." );
    pthread_detach ( pthread_self() );//成为自由线程

    //检查开启的通道
    updateChannels ( para );

    //更新参数
    pthread_mutex_lock ( &g_current_task_mutex );
    dots = scanPixels;// 扫描点数每次扫描都不变，因此只更新一次
    delay = scanDelays;
    rangeX = scanRangeX;
    rangeY = scanRangeY;
    offsetX = scanOffsetX;
    offsetY = scanOffsetY;
    samplingTimes = scanSamplingTimes;
    angle = PI * scanAngle / 180.0;
    currentTask = g_current_task;
    pthread_mutex_unlock ( &g_current_task_mutex );

    initScanBuf ( CMD_SCAN_WHOLE, dots * 2 );
    if ( currentTask == STOP )
    {
        goto normalScanExit;
    }
    //更新参数完成


//Added in 2008.01.12 by liyuan
//将探针移动到扫描区域的起始点
    cosAngle = cos ( angle );
    sinAngle = sin ( angle );
    tempX = hctc_x[0] * rangeX / 2.0 + offsetX;
    tempY = hctc_y[0] * rangeY / 2.0 + offsetY;
    posX = tempX * cosAngle + tempY * sinAngle;
    posY = -tempX * sinAngle + tempY * cosAngle;

//	write_DA(DA_Z, 65535);
    for ( i = 0 ;i < 100; i++ )
    {
        write_DA ( DA_X, 32767 + i * posX / 100 );
        write_DA ( DA_Y, 32767 + i * posY / 100 );
        udelay ( delay );
        pid_func();
    }

//预扫 3 行，然后开始正式扫描
		tempY = hctc_y[0] * rangeY / 2.0 + offsetY;
		for ( row = 0; row < 3; row++ ) // 当前行
		    for ( i = 0; i < dots; i++ )
		    {
		        tempX = hctc_x[i] * rangeX / 2.0 + offsetX;
		        posX = 32767 + tempX * cosAngle + tempY * sinAngle;
		        posY = 32767 - tempX * sinAngle + tempY * cosAngle;
		
		        write_DA ( DA_X, posX );
		        write_DA ( DA_Y, posY );
		        errorBuf[i + 6] = pid_func(); // PID 控制函数
		        topoBuf[i + 6] = ( short ) g_DA_z;
		    }
		    for ( i = dots - 1; i >= 0; i-- )
        {
            tempX = hcrc_x[i] * rangeX / 2.0 + offsetX;
            posX = 32767 + tempX * cosAngle + tempY * sinAngle;
            posY = 32767 - tempX * sinAngle + tempY * cosAngle;

            write_DA ( DA_X, posX );
            write_DA ( DA_Y, posY );
            errorBuf[i + 6] = pid_func();
            topoBuf[i + 6] = ( short ) g_DA_z;
        }
  	}
/////////////////////////////////
//开始正式扫图
    for ( row = 0; row < dots; row++ ) // 当前行
    {
        tempY = hctc_y[row] * rangeY / 2.0 + offsetY;
        //正扫循环
        for ( i = 0; i < dots; i++ )
        {
            tempX = hctc_x[i] * rangeX / 2.0 + offsetX;
            posX = 32767 + tempX * cosAngle + tempY * sinAngle;
            posY = 32767 - tempX * sinAngle + tempY * cosAngle;

            write_DA ( DA_X, posX );
            write_DA ( DA_Y, posY );
            //		udelay(delay);
            errorBuf[i + 6] = pid_func(); // PID 控制函数
            topoBuf[i + 6] = ( short ) g_DA_z;
            errorBuf[i + 6] = read_AD_N ( AD_ERROR, samplingTimes );
            if ( hasFiction ) fictionBuf[i + 6] = read_AD_N ( AD_FICTION, samplingTimes );
            if ( hasPhase ) phaseBuf[i + 6] = read_AD_N ( AD_PHASE, samplingTimes );
        }
        updateLineNum ( row + 1 );

        sendData();
        DEBUG1 ( "send one line data, line %d\n", row + 1 );

        //判断是否停止扫描
        pthread_mutex_lock ( &g_current_task_mutex );
        currentTask = g_current_task;
        pthread_mutex_unlock ( &g_current_task_mutex );
        if ( currentTask == STOP )
        {
            goto normalScanExit;
        }

        //反扫循环
        for ( i = dots - 1; i >= 0; i-- )
        {
            tempX = hcrc_x[i] * rangeX / 2.0 + offsetX;
            posX = 32767 + tempX * cosAngle + tempY * sinAngle;
            posY = 32767 - tempX * sinAngle + tempY * cosAngle;

            write_DA ( DA_X, posX );
            write_DA ( DA_Y, posY );
            errorBuf[i + 6] = pid_func();
            topoBuf[i + 6] = ( short ) g_DA_z;
            if ( hasInv )
            {
                errorBuf[i + 6] = read_AD_N ( AD_ERROR, samplingTimes );
                if ( hasFiction ) fictionBuf[i + 6] = read_AD_N ( AD_FICTION, samplingTimes );
                if ( hasPhase ) phaseBuf[i + 6] = read_AD_N ( AD_PHASE, samplingTimes );
            }
        }
        updateLineNum ( -row - 1 );

        if ( hasInv )
        {
            sendData();
        }
        DEBUG1 ( "Send one line data, line %d\n", -row - 1 );

        //更新参数，准备下一行扫描
        pthread_mutex_lock ( &g_current_task_mutex );
        currentTask = g_current_task;
        delay = scanDelays;
        rangeX = scanRangeX;
        rangeY = scanRangeY;
        offsetX = scanOffsetX;
        offsetY = scanOffsetY;
        samplingTimes = scanSamplingTimes;
        angle = PI * scanAngle / 180.0;
        pthread_mutex_unlock ( &g_current_task_mutex );

        if ( currentTask == STOP )
        {
            goto normalScanExit;
        }
        //根据更新参数计算步距，正反扫周期只计算一次
        cosAngle = cos ( angle );
        sinAngle = sin ( angle );
    }

normalScanExit:
    //将探针归位于中心位置
    for ( i = 100 ;i >= 0; i-- )
    {
        write_DA ( DA_X, 32768 + i * posX / 100 );
        write_DA ( DA_Y, 32768 + i * posY / 100 );
//			udelay(delay);
        pid_func();
    }
    pthread_mutex_lock ( &g_current_task_mutex );
    g_current_task = STOP;
    pthread_mutex_unlock ( &g_current_task_mutex );
    DEBUG0 ( "Leave normal scan thread" );
    return ( void* ) 0;
}



