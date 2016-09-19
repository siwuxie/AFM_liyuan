#include "serial_port.h"
/**
*@brief				Open serial port. return the file
*@param		dev		���� char *, usually we use "/dev/ttyS0" as argument
*@return	int		descripter on sucess or -1  on error
**/
int openPort(char * dev)
{
    int fd;
	/* here O_NOCTTY is must be,or program CTL your term */
    fd = open(dev, O_RDWR | O_NDELAY | O_NOCTTY); 
    if (fd != -1)
	{
		/* set file descriptor flags be async */
        fcntl(fd, F_SETFL, FASYNC); 
	}
    return (fd);
}

/**
*@brief  ���ô���ͨ������
*@param  fd     ���� int  �򿪴��ڵ��ļ����
*@param  speed  ���� int  �����ٶ�
*@return  void
**/
int speed_arr[] = {B38400, B19200, B9600, B4800, B2400, B1200, B300};
int name_arr[] = {38400, 19200, 9600, 4800, 2400, 1200, 300};
void setSpeed(int fd, int speed)
{
    int i;
    int status;
    struct termios Opt;
    tcgetattr(fd, &Opt);
    for (i = 0; i < sizeof(speed_arr)/sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);
            if (status != 0)
            {
                perror("tcsetattr fd1");
                return;
            }
            tcflush(fd,TCIOFLUSH);
        }
    }
}

/**
*@brief  ���ô�������λ��ֹͣλ��Ч��λ
*@param  fd         ���� int �򿪵Ĵ����ļ����
*@param  databits   ���� int ����λ ȡֵΪ 7 ���� 8
*@param  stopbits   ���� int ֹͣλ ȡֵΪ 1 ���� 2
*@param  parity     ���� int Ч������ ȡֵΪ N, E, O, S
**/
int setParity(int fd, int databits, int stopbits, int parity)
{
    struct termios options;
    if (tcgetattr(fd, &options) != 0)
    {
        perror("Setup serial port\n");
        return(-1);
    }
    options.c_cflag &= ~CSIZE;

    /*��������λ*/
    switch (databits)
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "Unsupported data size\n");
        return (-1);
    }

    /*����Ч��λ*/
    switch (parity)
    {
    case 'n':
    case 'N':
        options.c_cflag &= ~PARENB;   /* Clear parity enable */
        options.c_iflag &= ~INPCK;     /* Enable parity checking */
        break;
    case 'o':
    case 'O':
        options.c_cflag |= (PARODD | PARENB); /* ����Ϊ��Ч��*/
        options.c_iflag |= INPCK;             /* Disnable parity checking */
        break;
    case 'e':
    case 'E':
        options.c_cflag |= PARENB;     /* Enable parity */
        options.c_cflag &= ~PARODD;   /* ת��ΪżЧ��*/
        options.c_iflag |= INPCK;       /* Disnable parity checking */
        break;
    case 'S':
    case 's':  /*as no parity*/
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported parity\n");
        return (-1);
    }

    /* ����ֹͣλ*/
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported stop bits\n");
        return (-1);
    }

    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;
    tcflush(fd, TCIFLUSH);
    options.c_cc[VTIME] = 150; /* ���ó�ʱ 15 seconds*/
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("Setup Serial");
        return (-1);
    }
    return (0);
}

int serialWrite(int port, char *buf, int len)
{
		char portName[2][11]={"/dev/ttyS0", "/dev/ttyS1"};
		int fd = openPort(portName[port]);
		if(fd == -1)
		{
			perror("open port /dev/ttyS*");
			return -1;
		}
    setSpeed(fd, 9600);
    setParity(fd, 8, 1, 'N');

		write(fd, buf, len);
		close(fd);
		return 0;
}

int lcdBeep(int micro_second)
{
		char buf[6] = {0xf0, 0x5a, 0x35, 0x01, 0xa5, 0xf0};
		DEBUG1("Beep %d micro second\n", micro_second);
		serialWrite(1, buf, 6);
		usleep(1000 * micro_second);
		buf[3] = 0;
		serialWrite(1, buf, 6);
		return 0;
}

int lcdSetPicNum(int n)
{
		char buf[11] = {0xf0, 0x5a, 0x91, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0xa5, 0xf0};
		n %= 255;
		buf[4] = n % 100;
		buf[3] = n / 100;
		DEBUG1("Change LCD pic to No. %d \n", n);
		serialWrite(1, buf, 11);
		return 0;
}
