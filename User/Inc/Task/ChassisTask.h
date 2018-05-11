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
	float angle_last;
}_chassisYaw;

typedef struct{
	uint8_t flag;
	float target;
	uint8_t direction;
}_rotate_move;

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
	float target_vx;
	float target_vy;
	float x;
	float y;
	float Vx;
	float Vy;
	float Wz;
	float theta;
}_wheel_solve;

typedef struct{
	int16_t Speed[4];
	int16_t Postion[4];
}_feedback;

typedef struct wheelPara{
	_pid_Out pid[4];
	_feedback feedback;
	uint8_t	pid_flag;
	float speedLimit;
	float K_speed;
	_wheel_solve info;
	float targetSpeed[4];
	float direction[4];
	int16_t out[4];
}_wheelPara;

extern _wheelPara wheelInfo;
extern _chassis chassisPara;
extern _rotate_move rotate_para;

extern _pid_Para wheelpid;

void rotate_control(void);
int8_t Lidar_Func(const uint8_t ,  _lidar_message* , const uint8_t );
int8_t chassisControl(uint8_t);
int8_t wheelSolute(_wheelPara*, _chassis*, uint8_t);
int8_t allParaInit(void);
int8_t rotate_move(uint8_t , float , uint8_t );
#endif



