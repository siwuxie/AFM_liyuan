#include <stdio.h>      /*标准输入输出定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include "afm_comm.h"

/**
*@brief				Open serial port. return the file
*@param		dev		类型 char *, usually we use "/dev/ttyS0" as argument
*@return	int		descripter on sucess or -1  on error
**/
int openPort(char * dev);


/**
*@brief  设置串口通信速率
*@param  fd     类型 int  打开串口的文件句柄
*@param  speed  类型 int  串口速度
*@return  void
**/
void setSpeed(int fd, int speed);
/**
*@brief  设置串口数据位，停止位和效验位
*@param  fd         类型 int 打开的串口文件句柄
*@param  databits   类型 int 数据位 取值为 7 或者 8
*@param  stopbits   类型 int 停止位 取值为 1 或者 2
*@param  parity     类型 int 效验类型 取值为 N, E, O, S
**/
int setParity(int fd, int databits, int stopbits, int parity);
/**
*@brief  串口发数，波特率9600， N 8 1
*@param  int port   串口号（0/1）
*@param  char *buf  发送内容
*@param  int len 		发送长度
**/
int serialWrite(int port, char *buf, int len);

int lcdBeep(int micro_second);
int lcdSetPicNum(int micro_second);

