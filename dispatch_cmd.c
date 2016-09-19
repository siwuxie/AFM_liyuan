#include <math.h>
#include <stdio.h>
#include <sys/socket.h>
#include <pthread.h>
#include <string.h>


#include "work_thread.h"
#include "closeloop.h"
#include "hardware.h"
#include "serial_port.h"

pthread_mutex_t g_current_task_mutex = PTHREAD_MUTEX_INITIALIZER;
int g_current_task;




pthread_mutex_t g_thread_mutex = PTHREAD_MUTEX_INITIALIZER;
// motor 相关线程的互斥
pthread_mutex_t g_motor_mutex = PTHREAD_MUTEX_INITIALIZER;
int g_motor_task; 


int g_DA_z; //保存 Z 向的 DA 输出值

pthread_t scanTid, motorTid, laserTid, freqScanTid;

char rs232_buf[100];
void dispatch_cmd(struct command *pCmd)
{ 
	//分析命令字
	//激活工作线程
	int error;
	RESPONSE res;
	double dTemp;
	int nTemp;
	unsigned int unTemp1, unTemp2;
	/* 将扫描线程的优先级提高为实时优先级 */
	pthread_attr_t scan_thread_attr;
	struct sched_param scan_thread_param; 
	
	pthread_attr_init(&scan_thread_attr);
	pthread_attr_setdetachstate(&scan_thread_attr, PTHREAD_CREATE_DETACHED);
	pthread_attr_setscope(&scan_thread_attr, PTHREAD_SCOPE_PROCESS);
	pthread_attr_setinheritsched(&scan_thread_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_getschedparam(&scan_thread_attr, &scan_thread_param);
	scan_thread_param.sched_priority = 10;
	pthread_attr_setschedparam(&scan_thread_attr, &scan_thread_param);
	error = pthread_attr_setschedpolicy(&scan_thread_attr, SCHED_FIFO);
	if(error)
	{
		fprintf(stderr, 
			"Failed to set the scheduling policy to SCHED_FIFO : %s\n", 
			strerror(error));
	}		

	switch(pCmd->cmd)
	{
		case MOTOR_STEP_FORWARD:
			motor_steps(pCmd->para1, MOTOR_STEP_FORWARD);
			break;
		case MOTOR_STEP_BACKWARD:
			motor_steps(pCmd->para1, MOTOR_STEP_BACKWARD);
			break;
		case MOTOR_AUTO_FORWARD:
			DEBUG0("MOTOR_AUTO_FORWARD");
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("ERROR: another scan thread is working now!");
			}
			else
			{
				g_current_task = MOTOR_AUTO_FORWARD;
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("pthread_create(&motorTid, NULL, motor_autoforward_thread, NULL);");
				pthread_create(&motorTid, NULL, motor_autoforward_thread, NULL);
			};
		break;
		case MOTOR_GET_STEPS:
			DEBUG0("MOTOR_GET_STEPS");
			res.cmd = MOTOR_GET_STEPS;
			res.para1 = motor_get_steps();
			DEBUG1("Motor current step is %d", (int) res.para1);
			break;
		case MOTOR_AUTO_BACKWARD:
			DEBUG0("MOTOR_AUTO_BACKWARD");
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("ERROR: another scan thread is working now!");
			}
			else
			{
				g_current_task = MOTOR_AUTO_FORWARD;
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("pthread_create(&motorTid, NULL, motor_autobackward_thread, NULL);");				
				pthread_create(&motorTid, NULL, motor_autobackward_thread, &(pCmd->para1));
			};
		break;
		case MOTOR_FAST_FORWARD:
			DEBUG0("MOTOR_FAST_FORWARD");
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("ERROR: another scan thread is working now!");
			}
			else
			{
				g_current_task = MOTOR_FAST_FORWARD;
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("pthread_create(&motorTid, NULL, motor_fast_forward_thread, NULL);");
				pthread_create(&motorTid, NULL, motor_fast_forward_thread, NULL);
			};
			break;
		case MOTOR_FAST_BACKWARD:
			DEBUG0("MOTOR_FAST_BACKWARD");
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("ERROR: another scan thread is working now!");
			}
			else
			{
				g_current_task = MOTOR_FAST_BACKWARD;
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("pthread_create(&motorTid, NULL, motor_fast_forward_thread, NULL);");
				pthread_create(&motorTid, NULL, motor_fast_backward_thread, NULL);
			};			
			break;
		case MOTOR_STOP:
//			pthread_cancel(motorTid);
			g_current_task = STOP;	
			DEBUG0("MOTOR_STOP");
			break;

		case LASER_ON:
			setLaserOn();
			DEBUG0("LASER_ON");
			break;
		case LASER_OFF:
			setLaserOff();
			DEBUG0("LASER_OFF");
			break;
		case SET_HV_ON:
			setHighVoltageOn();
			break;
		case SET_HV_OFF:
			setHighVoltageOff();
			break;
		case GET_LASER_POS:
			DEBUG0("GET_LASER_POS");
			if(pCmd->para1 == 0)
			{
//				DEBUG0("GET_LASER_POS");
				pthread_mutex_lock(&g_current_task_mutex);
				if(g_current_task != STOP)
				{
				//还有扫描任务没有完成，不能继续，向上位机报告错误
					pthread_mutex_unlock(&g_current_task_mutex);
					DEBUG0("ERROR: another scan thread is working now!");
				}
				else
				{
					g_current_task = GET_LASER_POS;
					pthread_mutex_unlock(&g_current_task_mutex);
					DEBUG0("pthread_create(&laserTid, NULL, laser_thread, NULL);");
					pthread_create(&laserTid, NULL, laserThread, NULL);
				};
			}
			else
			{
				res.cmd = GET_LASER_POS;
				res.para1 = read_AD(AD_LASER_A);
				res.para2 = read_AD(AD_LASER_B);
				res.para3 = read_AD(AD_LASER_C);
				res.para4 = read_AD(AD_LASER_D);
				res.para5 = g_DA_z;
				send(connect_socket_fd, (char*)&res, sizeof(RESPONSE), 0);
			}
			break;

		case SET_MODE:
			DEBUG0("SET_MODE");
			setMode(pCmd);
			break;
		case SET_VOLTAGE:
			DEBUG0("SET_VOLTAGE");
			break;
		case CMD_FREQ_SCAN:
			DEBUG0("CMD_FREQ_SCAN");
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误write_DA( DA_Z, g_DA_z);
				pthread_mutex_unlock(&g_current_task_mutex);
				fprintf(stderr, "ERROR: another scan thread is working now\n");
			}
			else
			{
				g_current_task = CMD_FREQ_SCAN;
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("create freq scan thread");
				pthread_create(&scanTid, NULL, freqScanThread, pCmd);
			};
			break;
		case SET_FREQ_RANGE:
			DEBUG0("SET_FREQ_RANGE");
			memcpy(&unTemp1, &(pCmd->para1), sizeof(int));
			memcpy(&unTemp2, &(pCmd->para3), sizeof(int)); 
			setFreqRange(unTemp1, unTemp2);
			break;
		case SET_FREQ_AMPLITUDE:
			DEBUG0("SET_FREQ_AMPLITUDE");
			setWaveAmplitude( (unsigned char) pCmd->para1);
			break;
		case SET_WORKING_FREQ:
			DEBUG0("SET_WORKING_FREQ");
			memcpy(&nTemp, &(pCmd->para1), sizeof(int));
			dTemp = (double)((unsigned int) nTemp) / 100.0;
			dds_Out(dTemp);
			break;
		case CMD_FREQ_STOP:
			dds_Stop();
			break;
		case CMD_LINESCAN_START:
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误
				pthread_mutex_unlock(&g_current_task_mutex);
				fprintf(stderr, "ERROR: another scan thread is working now\n");
			}
			else
			{
				g_current_task = CMD_LINESCAN_START;
				pthread_mutex_unlock(&g_current_task_mutex);
				pthread_create(&scanTid, NULL, lineScanThread, pCmd);
			};
			break;
		case CMD_FAST_SCAN:
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误
				pthread_mutex_unlock(&g_current_task_mutex);
				fprintf(stderr, "ERROR: another scan thread is working now\n");
			}
			else
			{
				g_current_task = FAST_SCAN;
				pthread_mutex_unlock(&g_current_task_mutex);
				pthread_create(&scanTid, NULL, fastScanThread, NULL);
			};
			break;
		case CMD_SCAN_WHOLE:
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误
				pthread_mutex_unlock(&g_current_task_mutex);
			}
			else
			{
				g_current_task = NOMAL_SCAN;
				pthread_mutex_unlock(&g_current_task_mutex);
				pthread_create(&scanTid, &scan_thread_attr, normalScanThread, pCmd);
			};
			break;
		case CMD_SCAN_STOP:
			pthread_mutex_lock(&g_current_task_mutex);
			g_current_task = STOP;
			pthread_mutex_unlock(&g_current_task_mutex);
			DEBUG0("stop");
			break;
			
			case CMD_FORCE_CURVE:
			DEBUG0("CMD_FORCE_CURVE");
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("ERROR: another scan thread is working now!");
			}
			else
			{
				g_current_task = CMD_FORCE_CURVE;
				pthread_mutex_unlock(&g_current_task_mutex);
				DEBUG0("pthread_create(&motorTid, NULL, force_curve_thread, pCmd);");				
				pthread_create(&motorTid, NULL, forceCurveThread, pCmd);
			};			
			break;
			
		case SET_SCAN_RANGE:
			pthread_mutex_lock(&g_current_task_mutex);
			setScanRange(pCmd->para1, pCmd->para2, pCmd->para3, pCmd->para4, pCmd->para5);
			pthread_mutex_unlock(&g_current_task_mutex);

			break;
		case SET_SCAN_PIXEL:
				setScanPixel(pCmd->para1, pCmd->para2);
				setScanSamplingTimes(pCmd->para3);
				break;
		case SET_FEEDBACK_MODE:
				setFeedBackMode(pCmd->para1);
			break;
		case SET_PID_PARA:
			pthread_mutex_lock(&g_PID_mutex);
//////setPIDPara(short Kp, short Ki, short Kd， short loopCircles)
			setPIDPara(pCmd->para1, pCmd->para2, pCmd->para3, pCmd->para4);
			pthread_mutex_unlock(&g_PID_mutex);

			break;
		case SET_WORKING_POINT:
			setPIDSetPoint((short)pCmd->para1);
			break;
		case SET_PID_OTHER:
			pthread_mutex_lock(&g_PID_mutex);
//////setPIDParaOther(short eps, short times, short delay, short errorThresh)
			setPIDParaOther(pCmd->para2, pCmd->para1, pCmd->para3, pCmd->para4);
			pthread_mutex_unlock(&g_PID_mutex);
			break;
		case SET_LCD_PIC:
			lcdSetPicNum(pCmd->para1);
			break;			
		case SET_LCD_BEEP:
			lcdBeep(pCmd->para1);
			break;			
		case SET_CORRECTION_PARA:
			setHysteresisPara(pCmd->para1, pCmd->para2);
			break;	
			
/************************************************************************/
		case EXPERT_MODE_AD:
			res.cmd = pCmd->cmd;
			res.para1 = read_AD(pCmd->para1);
			send(connect_socket_fd, (char*)&res, sizeof(RESPONSE), 0);
			DEBUG1("ad10 = %d\n", res.para1);
			break;
		case EXPERT_MODE_DA:
			write_DA(pCmd->para1, pCmd->para2);
			break;
		case EXPERT_MODE_DDS_FREQ:
			memcpy(&nTemp, &(pCmd->para1), sizeof(int));
			dds_Out((double)((unsigned int) nTemp) / 100.0);
			break;
		case EXPERT_MODE_DDS_AMPL:
			setWaveAmplitude(pCmd->para1 & 0xff);
			break;
		case EXPERT_MODE_DDS_STOP:
			dds_Stop();
			break;	
		case EXPERT_MODE_LASER_POS:
			break;
		case EXPERT_MODE_IO_OUT1:
			IO_Out(pCmd->para1, pCmd->para2);
			break;
		case EXPERT_MODE_IO_OUT8:
			IO_Out8(pCmd->para1, pCmd->para2);
			break;
		case EXPERT_MODE_IO_OUT16:
			IO_Out16(pCmd->para1);
			break;
		case EXPERT_MODE_PID:
			pid_func();
			break;
		case EXPERT_MODE_LED_ON:
			setLedOn();
			break;
		case EXPERT_MODE_LED_OFF:
			setLedOff();
			break;
		case EXPERT_MODE_DDS_PHASE:
			DEBUG0("EXPERT_MODE_DDS_PHASE");
			setPhase(pCmd->para1);
			break;
/************************************************************************/
		case EXPERT_MODE_TEST:
			DEBUG0("EXPERT_MODE_TEST");
			readHugeData();
			DEBUG0("end of EXPERT_MODE_TEST");
			break;
			
		case EXPERT_MODE_GET_ERR:
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误
				pthread_mutex_unlock(&g_current_task_mutex);
			}
			else
			{
				g_current_task = NOMAL_SCAN;
				pthread_mutex_unlock(&g_current_task_mutex);
				pthread_create(&scanTid, NULL, get_error_thread, pCmd);
			};
			break;
			
			case EXPERT_MODE_PID_ERR:
			pthread_mutex_lock(&g_current_task_mutex);
			if(g_current_task != STOP)
			{
			//还有扫描任务没有完成，不能继续，向上位机报告错误
				pthread_mutex_unlock(&g_current_task_mutex);
			}
			else
			{
				g_current_task = NOMAL_SCAN;
				pthread_mutex_unlock(&g_current_task_mutex);
				pthread_create(&scanTid, NULL, pid_get_error_thread, pCmd);
			};
			break;
			case EXPERT_MODE_RS232_OUT:
			puts("EXPERT_MODE_RS232_OUT");
			nTemp = recv(connect_socket_fd, rs232_buf, pCmd->para2, MSG_WAITALL);
			serialWrite(pCmd->para1, rs232_buf, nTemp);
			break;
	};
}
void* pid_get_error_thread(void* para)
{
//	short new_set_point = ((short *)para)[1];
//	short record_dots = ((short *)para)[2];
//	pthread_detach(pthread_self());
//	DEBUG0("in get error thread");
//	PID_function01_debug(new_set_point, record_dots);
	
//	int Kp, Ki, loop_circles, channel, set_point;
//	static int integral = 0;
//	int loop = 0, z_value, error;
//	int err[200];
//	int zzz[200];
//	int integ[200];
//	int i;
//	FILE *fp;
//	
//	
///* Initialization of PID parameters */	
//	Kp = g_PID_P;
//	Ki = g_PID_I;
//	set_point = 0;
//	channel = AD_ERROR;
//	loop_circles = g_PID_maxLoopCircles;
//	write_DA( DA_Z, 0);
//	udelay(50);
//	error = -(read_AD(channel) - set_point);// 得到误差值
//	
//	while(loop < 200)
//	{
//		integral += error; 
//		if(integral > 200000) integral = 200000;
//		if(integral < 0) integral = 0;
//
//		/* PID 反馈计算结果 */
//		z_value = (Kp * error + Ki * integral) / 1000.0;
//		// 限制 z_value 在 DA 工作范围之内	
//		if(z_value > 65535)		z_value = 65535;
//		if(z_value < 0)			z_value = 0;
//			
//		err[loop] = error;		
//		integ[loop] = integral;	
//		zzz[loop]=z_value;
//		g_DA_z = z_value;
//		write_DA( DA_Z, g_DA_z);
//		udelay(5); //waiting for a short time 
//		
//		error = -(read_AD(channel) - set_point);// 得到误差值
//		loop++;
//	}	
//	fp = fopen("pid_error.dat","w");
//	fprintf(fp, "error, integral, z_value\n");
//	for(i = 0; i<200; i++)
//	{
//		fprintf(fp, "%d, %d, %d\n", err[i], integ[i], zzz[i]);
//	}
//	fclose(fp);
//	
//	g_current_task = STOP;
//	DEBUG0("leave get error thread");	
	return (void*)0;
}

void* get_error_thread(void* para)
{
//	struct response resp;
	FILE *fp;
	short error[400];
	int i;
	pthread_detach(pthread_self());
	DEBUG0("in get error thread");
	for(i=0;i<400;i++)
	{
		error[i] = getError(10);
//		udelay(20);
	}
	fp = fopen("error.dat","w");
	for(i = 0; i<400; i++)
	{
		fprintf(fp, "%d\n", error[i]);
	}
	fclose(fp);
	g_current_task = STOP;
	DEBUG0("leave get error thread");
	return (void*)0;
}








void* feedback_thread(void* para)
{
	int current_task;
	while(1)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		current_task = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex); 
		if(current_task == STOP)
		{
			break; 
		}
		pid_func();

	}
	return (void*)0;
}





/**
*@brief  设置工作模式
*@param  para(struct command*) 模式
*@return  void
*/
void setMode(struct command *pCmd)
{
	switch (pCmd->para1)
	{
	case MODE_STM:
//		setPIDChannel(AD_STM);
		setPIDChannel(AD_STM);//-------------------------
		DEBUG0("set to STM mode");
		break;
	case MODE_AFM_CONTACT:
//		setPIDChannel(AD_ERROR);
		setPIDChannel(AD_ERROR);//-------------------------
		setPIDSetPoint(0);
		DEBUG0("set to AFM contact mode");
		break;
	case MODE_AFM_TAPPING:
//		setPIDChannel(AD_OSAP);
		setPIDChannel(AD_OSAP);//-------------------------
		DEBUG0("set to AFM tapping mode");
		break;
	case MODE_SHEAR_FORCE:
		setPIDChannel(15);
		DEBUG0("set to shear force mode");
		break;
	default:
		setPIDChannel(AD_ERROR);
		setPIDSetPoint(0);
		DEBUG0("set to Default mode");
		DEBUG0("set to AFM contact mode");
	}
}

