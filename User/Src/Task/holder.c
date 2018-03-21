#include "Holder.h"
_motor motor;

_pid_Para yaw_init = {
	60,	// kp
	0.75f,	// ki
	0,	// kd
	1,	// i flag
	0,	// d flag
	300,	// i limit
	2000,	// out limit, limit range from -29000 ~ 29000
	2	//mode flag,  0 for disable, 3 for interval isolate
};

_pid_Para pitch_init = {
	60.0f,	// kp
	0.75f,	// ki
	5.5f,	// kd
	0,	// i flag
	1,	// d flag
	300,	// i limit
	2000,	// out limit
	2	//mode flag,  0 for disable
};

int8_t holder_Init(){
	motor.pitch.pid.k_para = pitch_init;
	motor.yaw.pid.k_para = yaw_init;
	return 1;
}
int8_t holder_Control(int8_t flag){
	if(!flag) return 0;
	motor.out[pitchID]=
			pidGet(&motor.pitch.pid.k_para,
						 &motor.pitch.pid.pid,
						 motor.pitch.target,
						 motor.pitch.fb_postion);
	motor.out[yawID]=
			pidGet(&motor.pitch.pid.k_para,
						 &motor.pitch.pid.pid,
						 motor.yaw.target,
						 motor.pitch.fb_postion);
	return 1;
}