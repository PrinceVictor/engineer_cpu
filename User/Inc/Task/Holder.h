#ifndef __HOLDER_H__
#define __HOLDER_H__

#include "MyFunc.h"
#include "PidTask.h"

#define yawID 0
#define pitchID 1
void holder_control(uint8_t, uint8_t);
void holder_init(void);
#if 0
typedef struct{
	_pid pid_angle;
	_pid pid_torque;
	int16_t torque_feedback;
	int16_t postion;
	float last_pos;
	float target;
	float target_temp;
	float angle;
	float angle_temp;
}_holder;

typedef struct{
	_holder yaw;
	_holder pitch;	
	int16_t out[2];
}_motor;

extern _motor motor;
int8_t holder_Init(void);
int16_t holder_Control(int8_t flag);
#endif

#endif 
