#ifndef __PIDTASK_H__
#define __PIDTASK_H__

#include "MyFunc.h"

typedef struct pid_Para{
	float kp;
	float	ki;
	float kd;
	uint8_t i_flag;
	uint8_t d_flag;
	float ki_limit;
	float outlimit;
	uint8_t flag;
}_pid_Para;

typedef struct pid_Out{
	float target;
	float feedback;
	float p_Out;
	float error;
	float last_error;
	float i_interval;
	float	i_Out;
	float d_Out;
	float Out;
}_pid_Out;

typedef struct pid{
	_pid_Out pid;
	_pid_Para  k_para;
}_pid;

typedef	struct pidDouble{
	_pid	shell;
	_pid	core;	
}_pidDouble;

int16_t pidGet(_pid_Para* ,
							_pid_Out* ,
							float ,
							int16_t);
#endif 
