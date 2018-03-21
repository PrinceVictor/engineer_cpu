#ifndef __HOLDER_H__
#define __HOLDER_H__

#include "MyFunc.h"
#include "PidTask.h"
#define yawID 1
#define pitchID 0

typedef struct{
	_pid pid;
	int16_t fb_postion;
	float target;
}_holder;

typedef struct{
	_holder yaw;
	_holder pitch;	
	int16_t out[2];
}_motor;

extern _motor motor;
int8_t holder_Init(void);
int8_t holder_Control(int8_t flag);
#endif 
