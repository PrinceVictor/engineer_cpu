#include "ChassisTask.h"
#include "ComunicateTask.h"
#include "main.h"

_wheelPara wheelInfo = {0};
_chassis chassisPara = {0};

_pid_Para wheelpid = {
	60,	// kp
	0.75f,	// ki
	0,	// kd
	1,	// i flag
	0,	// d flag
	1000,	// i limit
	5000,	// out limit, limit range from -32768 ~ 32768
	3	//mode flag,  0 for disable, 3 for interval isolate
};

_pid_Para chassispid_core = {
	18.0f,	// kp
	0,	// ki
	2.5f,	// kd
	0,	// i flag
	1,	// d flag
	0,	// i limit
	400,	// out limit
	4	//mode flag,  0 for disable
};

_pid_Para chassispid_shell = {
	15.0f, //180,	// kp 6.5
	0.5f,	// ki
	1.0f,	// kd
	0,	// i flag
	1,	// d flag
	0,	// i limit
	1000,	// out limit
	5	//mode flag,  0 for disable
};

int8_t allParaInit(void)
{
	wheelInfo.kpid = wheelpid;
	wheelInfo.K_speed = 1.0f;
	wheelInfo.speedLimit = 400;
	chassisPara.x = 1.5f;
	chassisPara.y = 0.5f;
	chassisPara.pid.shell.k_para = chassispid_shell;
	chassisPara.pid.core.k_para = chassispid_core;
	remote.rc.ch0 = 1024;
	remote.rc.ch1 = 1024;
	remote.rc.ch2 = 1024;
	remote.rc.ch3 = 1024;
	return 1;
} 



int8_t chassisControl(uint8_t flag)
{	
	int i;
	if(!flag){
		return 0;
}
	else{
	//pid for angle
//		chassisPara.Rt = 
			pidGet(&chassisPara.pid.shell.k_para,
														&chassisPara.pid.shell.pid,		
														chassisPara.yaw.target,
														chassisPara.yaw.angle);
	//pid for angle_speed
//		chassisPara.pid.shell.pid.Out =0;
			chassisPara.Rt = \
					pidGet(&chassisPara.pid.core.k_para,
														&chassisPara.pid.core.pid,		
														chassisPara.pid.shell.pid.Out,
														chassisPara.yaw.angle_speed);
		
		chassisPara.yaw.last_target = chassisPara.yaw.target;
		
		Send_data[0] = (float)chassisPara.yaw.angle ; 
		Send_data[1] = (float)chassisPara.yaw.target;
//		Send_data[2] = (float)wheelInfo.pid[1].Out;
//		Send_data[3] = (float)wheelInfo.feedback.Speed[1];

		Send_data[2] = (float)chassisPara.pid.core.pid.feedback; 
		Send_data[3] = (float)chassisPara.pid.shell.pid.Out;
//		Send_data[4] = (float)chassisPara.pid.core.pid.Out;
		Send_data[4] = (float)chassisPara.pid.core.pid.Out;
		
//		chassisPara.Rt = 0;
	//wheel solute
		wheelSolute(&wheelInfo, &chassisPara);
	//pid for wheels
		for(i=0; i<4; i++){
			wheelInfo.out[i] = pidGet(&wheelInfo.kpid,
																&wheelInfo.pid[i],
																wheelInfo.targetSpeed[i],
																wheelInfo.feedback.Speed[i]);
		}
	return 1;
}
}

int8_t wheelSolute(_wheelPara* para, _chassis* chassis){
	int i;
	
	para->direction[0] = \
		chassis->x*(-chassis->Fb +chassis->Lr) + chassis->y* chassis->Rt;
	para->direction[1] = \
		chassis->x*(chassis->Fb +chassis->Lr) + chassis->y* chassis->Rt;
	para->direction[2] = \
		chassis->x*(chassis->Fb -chassis->Lr) + chassis->y* chassis->Rt;
	para->direction[3] = \
		chassis->x*(-chassis->Fb -chassis->Lr) + chassis->y* chassis->Rt;
	
	for(i=0; i<4; i++ ){
		para->targetSpeed[i] = \
			amplitudeLimiting(1, para->direction[i]*para->K_speed, para->speedLimit);
		para->feedback.Speed[i] = para->feedback.Speed[i]/19;
}
	return 1;
}


