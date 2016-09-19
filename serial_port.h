#include <stdio.h>      /*��׼�����������*/
#include <unistd.h>     /*Unix ��׼��������*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>      /*�ļ����ƶ���*/
#include <termios.h>    /*PPSIX �ն˿��ƶ���*/
#include <errno.h>      /*����Ŷ���*/
#include "afm_comm.h"

/**
*@brief				Open serial port. return the file
*@param		dev		���� char *, usually we use "/dev/ttyS0" as argument
*@return	int		descripter on sucess or -1  on error
**/
int openPort(char * dev);


/**
*@brief  ���ô���ͨ������
*@param  fd     ���� int  �򿪴��ڵ��ļ����
*@param  speed  ���� int  �����ٶ�
*@return  void
**/
void setSpeed(int fd, int speed);
/**
*@brief  ���ô�������λ��ֹͣλ��Ч��λ
*@param  fd         ���� int �򿪵Ĵ����ļ����
*@param  databits   ���� int ����λ ȡֵΪ 7 ���� 8
*@param  stopbits   ���� int ֹͣλ ȡֵΪ 1 ���� 2
*@param  parity     ���� int Ч������ ȡֵΪ N, E, O, S
**/
int setParity(int fd, int databits, int stopbits, int parity);
/**
*@brief  ���ڷ�����������9600�� N 8 1
*@param  int port   ���ںţ�0/1��
*@param  char *buf  ��������
*@param  int len 		���ͳ���
**/
int serialWrite(int port, char *buf, int len);

int lcdBeep(int micro_second);
int lcdSetPicNum(int micro_second);

