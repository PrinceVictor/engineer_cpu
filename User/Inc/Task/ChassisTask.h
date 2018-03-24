#ifndef __CHASSISTASK_H__
#define __CHASSISTASK_H__

#include "MyFunc.h"
#include "PidTask.h"
#include "Referee.h"

typedef struct{
	float target;
	uint8_t target_changeMode;
	float last_target;
	float temp;
	float angle;
	float angle_speed;
}_chassisYaw;

typedef struct{
	float Fb;
	float Lr;
	float Rt;
	float x,y;
	_chassisYaw yaw;
	float observe;
	_pidDouble pid;
	uint8_t pid_flag;
}_chassis;

typedef struct{
	int16_t Speed[4];
	int16_t Postion[4];
}_feedback;

typedef struct wheelPara{
	_pid_Para kpid;
	_pid_Out pid[4];
	_feedback feedback;
	uint8_t	pid_flag;
	float speedLimit;
	float K_speed;
	float speed[4];
	float targetSpeed[4];
	float direction[4];
	int16_t out[4];
}_wheelPara;

extern _wheelPara wheelInfo;
extern _chassis chassisPara;

int8_t Lidar_Func(const uint8_t , const _lidar_message* , int8_t );
int8_t chassisControl(uint8_t);
int8_t wheelSolute(_wheelPara*, _chassis*);
int8_t allParaInit(void);

#endif



