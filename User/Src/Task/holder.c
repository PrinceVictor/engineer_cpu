#include "Holder.h"
int pos[2] = {0};

void holder_init(){
	pos[yawID] = 20;
	pos[pitchID] = 14;
}
void holder_control(uint8_t flag, uint8_t mode){
	if(!flag){
		return;
	}
	switch(mode){
		case 0x01:{
		break;	
		}
		case 0x02:{
			break;	
		}
		case 0x03:{
			break;	
		}
		
	}
	TIM_SetCompare3(TIM3,pos[yawID]);  //5 - 26
	TIM_SetCompare4(TIM3,pos[pitchID]);
}
#if 0
_motor motor;
const int16_t mid_postion[2] = {
	5763, 6094
};

//yaw out left +
//pitch out down +
_pid_Para yaw_init = {
	50,	// kp
	1.0f,	// ki
	1.5,	// kd
	1,	// i flag
	1,	// d flag
	200,	// i limit
	1000,	// out limit, limit range from -29000 ~ 29000
	2	//mode flag,  0 for disable, 3 for interval isolate
};
_pid_Para yaw_torque_init = {
	50,	// kp
	0.0f,	// ki
	0,	// kd
	0,	// i flag
	0,	// d flag
	800,	// i limit
	5000,	// out limit, limit range from -29000 ~ 29000
	6	//mode flag,  0 for disable, 3 for interval isolate
};

_pid_Para pitch_init = {
	30.0f,	// kp
	1.75f,	// ki
	0.0f,	// kd
	0,	// i flag
	0,	// d flag
	200,	// i limit
	800,	// out limit
	2	//mode flag,  0 for disable
};

_pid_Para pitch_torque_init = {
	10,	// kp
	0.0f,	// ki
	0,	// kd
	0,	// i flag
	0,	// d flag
	800,	// i limit
	2000,	// out limit, limit range from -29000 ~ 29000
	6	//mode flag,  0 for disable, 3 for interval isolate
};

int8_t holder_Init(){
	motor.pitch.pid_angle.k_para = pitch_init;
	motor.yaw.pid_angle.k_para = yaw_init;
	motor.pitch.pid_torque.k_para = pitch_torque_init;
	motor.yaw.pid_torque.k_para = yaw_torque_init;
	motor.pitch.target = 0;
	motor.yaw.target = 0;
	motor.yaw.angle = ((float)motor.yaw.postion - mid_postion[0])/8191.0f*359.0f;
	motor.pitch.angle = -((float)motor.pitch.postion - mid_postion[1])/8191.0f*359.0f;
	motor.yaw.last_pos = motor.yaw.postion;
	motor.pitch.last_pos = motor.pitch.postion;
	return 1;
}
int16_t holder_Control(int8_t flag){
	
	if(!flag) return 0;
	
			pidGet(&motor.yaw.pid_angle.k_para,
						 &motor.yaw.pid_angle.pid,
						 motor.yaw.target,
						 motor.yaw.angle);
			motor.out[yawID]=pidGet(&motor.yaw.pid_torque.k_para,
						 &motor.yaw.pid_torque.pid,
						 motor.yaw.pid_angle.pid.Out,
						 (float)motor.yaw.torque_feedback);
			pidGet(&motor.pitch.pid_angle.k_para,
						 &motor.pitch.pid_angle.pid,
						 motor.pitch.target,
						 motor.pitch.angle);
		motor.out[pitchID]=-pidGet(&motor.pitch.pid_torque.k_para,
						 &motor.pitch.pid_torque.pid,
						 motor.pitch.pid_angle.pid.Out,
						 (float)motor.pitch.torque_feedback);

	return 1;
}
#endif