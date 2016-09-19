#include <math.h>

#include "work_thread.h"
#include "hardware.h"
#include "closeloop.h"

#define XY_DELAY 20

//----ɨ�����ݴ洢��---
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

//----ɨ�����---
static int scanRangeX, scanRangeY, scanOffsetX, scanOffsetY;
static int scanAngle;
static int scanPixels;
static int scanDelays;
static int scanSamplingTimes;

//----ͨ��ѡ��---
static int hasInv = 0;
static int hasError = 0, hasPhase = 0, hasOther = 0, hasFiction = 0;
static int hasPsdA = 0, hasPsdB = 0, hasPsdC = 0, hasPsdD = 0;

/**
*@brief  ��ʼ��ɨ�����ݴ洢��
*@param  cmd(unsigned short) ɨ������
*@param  size(unsigned short) �������ݴ�С���ֽڣ�
*@return  void
*/
void initScanBuf ( unsigned short cmd, unsigned short size )
{
    topoBuf[0] = cmd;
    topoBuf[1] = size;
    topoBuf[3] = 0; //ͨ����

    errorBuf[0] = cmd;
    errorBuf[1] = size;
    errorBuf[3] = 1;//ͨ����

    phaseBuf[0] = cmd;
    phaseBuf[1] = size;
    phaseBuf[3] = 2;//ͨ����

    fictionBuf[0] = cmd;
    fictionBuf[1] = size;
    fictionBuf[3] = 4;//ͨ����

    psdA_Buf[0] = cmd;
    psdA_Buf[1] = size;
    psdA_Buf[3] = 5;//ͨ����

    psdB_Buf[0] = cmd;
    psdB_Buf[1] = size;
    psdB_Buf[3] = 6;//ͨ����

    psdC_Buf[0] = cmd;
    psdC_Buf[1] = size;
    psdC_Buf[3] = 7;//ͨ����

    psdD_Buf[0] = cmd;
    psdD_Buf[1] = size;
    psdD_Buf[3] = 8;//ͨ����
//	errorBuf[3] = 1;
}


/**
*@brief  ���� AD �������������������������е� AD��
*@param  times(int) AD ��������
*@return  void
*/
void setScanSamplingTimes ( int times )
{
    scanSamplingTimes = times;
    DEBUG1 ( "scanSamplingTimes is %d\n", scanSamplingTimes );
}


/**
*@brief  ���� ɨ����ʱ�����������������е� ��ʱ��
*@param  delay(int) ��ʱ��us��
*@return  void
*/
void setScanDelay ( int delay )
{
    scanDelays = delay;
    DEBUG1 ( "scanDelays is %d\n", scanDelays );
}


/**
*@brief  ���� ɨ�����
*@param  XDots(int) X ����ɨ�����
*@param  YDots(int) Y ����ɨ�����
*@return  void
*/
void setScanPixel ( int XDots, int YDots )
{
    scanPixels = ( XDots > YDots ) ? XDots : YDots;
    DEBUG1 ( "scanPixels is %d\n", scanPixels );
}


/**
*@brief  ���� ɨ�跶Χ�Ȳ���
*@param  rangeX(int) X ����ɨ�跶Χ
*@param  rangeY(int) Y ����ɨ�跶Χ
*@param  offsetX(int) X ����ƫ��
*@param  offsetY(int) Y ����ƫ��
*@param  angle(int) ��ת�Ƕ�
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
*@brief  �����к�
*@param  line(short) �кţ��� 1 ��ʼ
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
*@brief  ��鿪����ͨ��
*@param  para(COMMAND *) ͨ��ѡ��
*@return  void*
*/
void updateChannels ( COMMAND * para )
{
    unsigned short channels; //ͨ��
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
*@brief  ����ɨ���̣߳�δ��ɣ�
*@param  para(void*) ͨ��ѡ��
*@return  void*
*/
void* fastScanThread ( void* para )
{
    pthread_detach ( pthread_self() );//��Ϊ�����߳�

    //��鿪����ͨ��
    updateChannels ( para );
    //ͨ��������

    return ( void* ) 0;
}
int hcMethod = 0;
double hctc_xA = 0.0;//��ɨx����У��ϵ�� A, -1 +1
double hctc_xB = 0.0;//��ɨx����У��ϵ�� B, -1 +1
double hcrc_xA = 0.0;//��ɨx����У��ϵ�� A, -1 +1
double hcrc_xB = 0.0;//��ɨx����У��ϵ�� B, -1 +1
double hctc_yA = 0.0;//��ɨy����У��ϵ�� A, -1 +1
double hctc_yB = 0.0;//��ɨy����У��ϵ�� B, -1 +1
double hctc_x[512]; //hysteresis correction trace curve, x dircection
double hcrc_x[512];//hysteresis correction retrace curve, x dircection
double hctc_y[512];////hysteresis correction trace curve, y dircection

//�ö��κ����ķ�������������У��
double corr2 ( double y, double a ) //ֻ����ɨ��У��
{
    //�����Է���Ϊ y = a x^2 - a + x
    double sq;
    if ( a == 0 ) return y;
    sq = sqrt ( 1 + 4 * a * ( a + y ) );
    if ( a > 0 ) //��Ӧ���Ƕ���ɨ���ߵĽ�����ȡ��ĸ�
        return 0.5 * ( -1 + sq ) / a;
    else //��Ӧ���ǶԷ�ɨ���ߵĽ�����ȡС�ĸ�
        return 0.5 * ( -1 - sq ) / a;
}
//�����κ����ķ�������������У��
double corr3 ( double y, double a, double b )
{
    double x, f, ff;
    double i;
    if ( y > 1 ) y = 1;
    if ( y < -1 ) y = -1;

    //�����Է���Ϊ y = a x^3 + b x^2 - a x - b + x
    x = y; //set the initial value of iteration
    for ( i = 0; i < 5; i++ ) //����ţ�ٵ������󷽳̵ĸ�
    {
        f = a * x * x * x + b * x * x - a * x - b + x - y;
        ff = 3 * a * x * x + 2 * b * x - a + 1;
        x = x - f / ff; // x = x - f / f'
    }
    return x;
}
void updateHCCurve ( void ) //���·�����У������
{
    int i;
    double y;
    if ( hcMethod == 0 ) // ����������У��
    {
        for ( i = 0; i < 512; i++ )
        {
            y = 2.0 * i / 511.0 - 1;
            hctc_x[i] = hcrc_x[i] = hctc_y[i] = y;
        }
    }
    else if ( hcMethod == 1 ) //���� 2 �κ����ķ�����У��
    {
        for ( i = 0; i < 512; i++ )
        {
            y = 2.0 * i / 511.0 - 1;
            hctc_x[i] = corr2 ( y, hctc_xB );
            hcrc_x[i] = corr2 ( y, hcrc_xB );
            hctc_y[i] = corr2 ( y, hctc_yB );

        }
    }
    else if ( hcMethod == 2 ) // �������κ����ķ�����У��
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
*@brief  ��ɨ���߳�
*@param  para(void*) ͨ��ѡ��
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
    pthread_detach ( pthread_self() );//��Ϊ�����߳�
//	DEBUG0("Line scan thread start1.");
    //��鿪����ͨ��
    updateChannels ( para );
//	DEBUG0("Line scan thread start2.");
    line = ( ( COMMAND * ) para )->para4;//ɨ��ڼ���, �� 1 ��ʼ
    line --;//ʹ���ɴ��㿪ʼ������

    //���²���
    pthread_mutex_lock ( &g_current_task_mutex );
    dots = scanPixels;// ɨ�����ÿ��ɨ�趼���䣬���ֻ����һ��
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
    //���²������

//Added in 2008.01.12 by liyuan
//��̽���ƶ���ɨ���������ʼ��
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
//��ʼ��ʽ��ɨ��
    while ( 1 )
    {
        tempY = hctc_y[line] * rangeY / 2.0 + offsetY;//ÿ��ֻ�����һ�μ���
        for ( i = 0; i < dots; i++ )
        {
            tempX = hctc_x[i] * rangeX / 2.0 + offsetX;
            posX = 32767 + tempX * cosAngle + tempY * sinAngle;
            posY = 32767 - tempX * sinAngle + tempY * cosAngle;
            write_DA ( DA_X, posX );
            write_DA ( DA_Y, posY );
            errorBuf[i + 6] = pid_func(); // PID ���ƺ�������ֵ���ǿ������
            topoBuf[i + 6] = ( short ) g_DA_z;
            errorBuf[i + 6] = read_AD_N ( AD_ERROR, samplingTimes );
            if ( hasFiction ) fictionBuf[i + 6] = read_AD_N ( AD_FICTION, samplingTimes );
            if ( hasPhase ) phaseBuf[i + 6] = read_AD_N ( AD_PHASE, samplingTimes );
        }
        updateLineNum ( line + 1 );
        sendData();
        DEBUG1 ( "send one line data, line %d\n", line + 1 );

        //�ж��Ƿ�ֹͣɨ��
        pthread_mutex_lock ( &g_current_task_mutex );
        currentTask = g_current_task;
        pthread_mutex_unlock ( &g_current_task_mutex );
        if ( currentTask == STOP )
        {
            goto lineScanExit;
        }

        //��ɨѭ��
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

        //���²�����׼����һ��ɨ��
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
        //���ݸ��²������㲽�࣬����ɨ����ֻ����һ��
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
    pthread_detach ( pthread_self() );//��Ϊ�����߳�

    //��鿪����ͨ��
    updateChannels ( para );

    //���²���
    pthread_mutex_lock ( &g_current_task_mutex );
    dots = scanPixels;// ɨ�����ÿ��ɨ�趼���䣬���ֻ����һ��
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
    //���²������


//Added in 2008.01.12 by liyuan
//��̽���ƶ���ɨ���������ʼ��
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

//Ԥɨ 3 �У�Ȼ��ʼ��ʽɨ��
		tempY = hctc_y[0] * rangeY / 2.0 + offsetY;
		for ( row = 0; row < 3; row++ ) // ��ǰ��
		    for ( i = 0; i < dots; i++ )
		    {
		        tempX = hctc_x[i] * rangeX / 2.0 + offsetX;
		        posX = 32767 + tempX * cosAngle + tempY * sinAngle;
		        posY = 32767 - tempX * sinAngle + tempY * cosAngle;
		
		        write_DA ( DA_X, posX );
		        write_DA ( DA_Y, posY );
		        errorBuf[i + 6] = pid_func(); // PID ���ƺ���
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
//��ʼ��ʽɨͼ
    for ( row = 0; row < dots; row++ ) // ��ǰ��
    {
        tempY = hctc_y[row] * rangeY / 2.0 + offsetY;
        //��ɨѭ��
        for ( i = 0; i < dots; i++ )
        {
            tempX = hctc_x[i] * rangeX / 2.0 + offsetX;
            posX = 32767 + tempX * cosAngle + tempY * sinAngle;
            posY = 32767 - tempX * sinAngle + tempY * cosAngle;

            write_DA ( DA_X, posX );
            write_DA ( DA_Y, posY );
            //		udelay(delay);
            errorBuf[i + 6] = pid_func(); // PID ���ƺ���
            topoBuf[i + 6] = ( short ) g_DA_z;
            errorBuf[i + 6] = read_AD_N ( AD_ERROR, samplingTimes );
            if ( hasFiction ) fictionBuf[i + 6] = read_AD_N ( AD_FICTION, samplingTimes );
            if ( hasPhase ) phaseBuf[i + 6] = read_AD_N ( AD_PHASE, samplingTimes );
        }
        updateLineNum ( row + 1 );

        sendData();
        DEBUG1 ( "send one line data, line %d\n", row + 1 );

        //�ж��Ƿ�ֹͣɨ��
        pthread_mutex_lock ( &g_current_task_mutex );
        currentTask = g_current_task;
        pthread_mutex_unlock ( &g_current_task_mutex );
        if ( currentTask == STOP )
        {
            goto normalScanExit;
        }

        //��ɨѭ��
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

        //���²�����׼����һ��ɨ��
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
        //���ݸ��²������㲽�࣬����ɨ����ֻ����һ��
        cosAngle = cos ( angle );
        sinAngle = sin ( angle );
    }

normalScanExit:
    //��̽���λ������λ��
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



