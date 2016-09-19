#include <unistd.h>
#include "hardware.h"
#include "work_thread.h"

//----步进电机参数--
// 需要特别处理，应该每次关闭前把这个值存到文件中
// 暂时这个参数还没有用到
int motorSteps; 

// 步进电机每步的延时在400us - 1500us 之间时运转平稳
const int stepMotorDelay = 800;
const int fasMotorDelay = 800;
const int delayInThread = 2000;
const int maxSteps = 4000; //单次进退针的最大步数


/**
*@brief  步进电机进退
*@param  steps(int) 进退的步数
*@param  direction(int) 方向 MOTOR_STEP_FORWARD / MOTOR_STEP_BACKWARD
*@return  void
*/
void motor_steps(int steps, int direction)
{
	int i;
	if(direction == MOTOR_STEP_FORWARD)
	{
		DEBUG0("MOTOR_STEP_FORWARD");
		if(steps <= maxSteps)
		{
			for(i = 0; i < steps; i++)
			{
				motor_forward_one_step();
				udelay(stepMotorDelay);
			}
		}
		else
		{
			motor_forward_one_step();
			udelay(stepMotorDelay);
		}	
	}
	else
	{
		DEBUG0("MOTOR_STEP_BACKWARD");
		if(steps <= maxSteps)
		{
			for(i = 0; i < steps; i++)
			{
				motor_backward_one_step();
				udelay(stepMotorDelay);
			}
		}
		else
		{
			motor_backward_one_step();
			udelay(stepMotorDelay);
		}
	}
	motor_stop();
}
/*void* motor_autoforward_thread(void* para)
{
	int i;
	struct response cmd;
	cmd.cmd = MOTOR_AUTO_FORWARD;
	DEBUG0("motor auto forward");
	while(1)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		if(g_current_task == STOP) break;
		pthread_mutex_unlock(&g_current_task_mutex);
		g_DA_z = 65535; 
		write_DA( DA_Z, g_DA_z);
		motor_forward_one_step();
		for(g_DA_z = 65500; g_DA_z >= 30000; g_DA_z -= 100)
		{
			write_DA( DA_Z, g_DA_z);
			for(i = 0; i < 1000; i++);
			if(getError() < 0) 
			{
				for(i = 0; i < 10; i++)
				{
					simplePID();
				}
				break;
			}
		}
	}
	motor_stop();

	send(connect_socket_fd, (char*)&cmd, 12, 0);
	g_current_task = STOP;
	DEBUG0("motor auto forward stoped");
	return (void*)0;
}*/


/**
*@brief  电机自动进针线程
*@param  para(void*) 没有用到
*@return  void*
*/
void* motor_autoforward_thread(void* para)
{
	int i, error;
	int currentTask;
	struct response resp;
	
	pthread_detach(pthread_self());//成为自由线程
	
	bzero(&resp, sizeof(struct response));
	resp.cmd = MOTOR_AUTO_FORWARD;
	resp.para1 = 0;//无附加字节
	resp.para2 = 0;//开始自动进针
	send(connect_socket_fd, (char*)&resp, 12, 0);
	DEBUG0("motor auto forward");
	g_DA_z = 10000; 
	write_DA( DA_Z, g_DA_z);
	while(1)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex);

		if(currentTask == STOP) 
		{
				goto GETOUT;
		}

		motor_forward_one_step();	
		usleep(delayInThread);
//		motor_stop();
//		udelay(500);
		error = getError(100);
		if(error < 0) break;
//		for(i = 0; i < 50; i++)
//		{
//			pid_func();
//		}
//		if( g_DA_z > 10000 ) break; 
	}
	pid_func();
	motor_stop();	
	resp.para1 = 0;//无附加字节
	resp.para2 = 1;//自动进针结束
	send(connect_socket_fd, (char*)&resp, 12, 0);

GETOUT:
	motor_stop();	
	pthread_mutex_lock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	
	DEBUG0("motor auto forward stoped");
	return (void*)0;
}


/**
*@brief  电机自动退针线程
*@param  para(void*) 退针步数
*@return  void*
*/
void* motor_autobackward_thread(void* para)
{
	int currentTask, i;
	short steps = *((short*)para);
	struct response resp;
	bzero(&resp, sizeof(struct response));
	resp.cmd = MOTOR_AUTO_BACKWARD;
	pthread_detach(pthread_self());//成为自由线程
	if(steps < 0 && steps > 4000)
	{
		steps = 800;
	}

	DEBUG1("motor auto backward %d steps..", (int)steps);
	for(i = 0; i < steps; i++)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex);
		if(currentTask == STOP) break;
		motor_backward_one_step();
		usleep(delayInThread);
	}
	motor_stop();
	send(connect_socket_fd, (char*)&resp, 12, 0);
	DEBUG0("motor auto backward stoped");
	
	pthread_mutex_lock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);

	return (void*)0;
}
/**
*@brief  电机快速进针线程
*@param  para(void*) 没有用到
*@return  void*
*/
void* motor_fast_forward_thread(void* para)
{
	int currentTask;
	pthread_detach(pthread_self());
	DEBUG0("motor fast forward..");
	while(1)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex);
		if(currentTask == STOP) break;
		motor_forward_one_step();
		usleep(delayInThread);
	}
	
	motor_stop();
	pthread_mutex_unlock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	DEBUG0("motor fast forward stoped");
	return (void*)0;
}
/**
*@brief  电机快速退针线程
*@param  para(void*) 没有用到
*@return  void*
*/
void* motor_fast_backward_thread(void* para)
{
	int currentTask;
	pthread_detach(pthread_self());
	DEBUG0("motor fast backward..");
	while(1)
	{
		pthread_mutex_lock(&g_current_task_mutex);
		currentTask = g_current_task;
		pthread_mutex_unlock(&g_current_task_mutex);
		if(currentTask == STOP) break;
		motor_backward_one_step();
		usleep(delayInThread);
	}
	motor_stop();
	pthread_mutex_unlock(&g_current_task_mutex);
	g_current_task = STOP;
	pthread_mutex_unlock(&g_current_task_mutex);
	DEBUG0("motor fast backward stoped");
	return (void*)0;
}

