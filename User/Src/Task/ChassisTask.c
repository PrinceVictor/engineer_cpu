#include "ChassisTask.h"
#include "ComunicateTask.h"
#include "main.h"

_wheelPara wheelInfo = {0};
_chassis chassisPara = {0};
int16_t set = 0;
int8_t lidar_flag;
int const postion[8][2] = {
	{25,360+ 310*4},{25,360+ 310*3},{25,360+ 310*2},{25,360+ 310*1},{25,360+ 310*0},{275+130,25},{275+130*1,25},{275+130*2,25}
};
_pid_Para wheelpid = {
	55,	// kp
	0.75f,	// ki	
	0,	// kd
	1,	// i flag
	0,	// d flag
	800,	// i limit
	3000,	// out limit, limit range from -32768 ~ 32768
	3	//mode flag,  0 for disable, 3 for interval isolate
};

_pid_Para chassispid_core = {
	13.5f,	// kp
	0,	// ki
	1.0f,	// kd
	0,	// i flag
	0,	// d flag
	0,	// i limit
	400,	// out limit
	4	//mode flag,  0 for disable
};

_pid_Para chassispid_shell = {
	8.5f, //180,	// kp 6.5
	0.5f,	// ki
	0.5f,	// kd
	0,	// i flag
	0,	// d flag
	0,	// i limit
	1000,	// out limit
	5	//mode flag,  0 for disable
};

int8_t allParaInit(void)
{
	wheelInfo.kpid = wheelpid;
	wheelInfo.K_speed = 1.0f;
	wheelInfo.speedLimit = 400;
	chassisPara.x = 2.0f;
	chassisPara.y = 1.2f;
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

		Send_data[2] = (float)chassisPara.pid.core.pid.feedback; 
		Send_data[3] = (float)chassisPara.pid.shell.pid.Out;
		
//		Send_data[4] = (float)chassisPara.pid.core.pid.Out;
		
//		chassisPara.Rt = 0;
	//wheel solute
	
		wheelSolute(&wheelInfo, &chassisPara);
	//pid for wheels
		for(i=0; i<4; i++){
			wheelInfo.out[i] = pidGet(&wheelInfo.kpid,
																&wheelInfo.pid[i],
																wheelInfo.targetSpeed[i],
																(float)(wheelInfo.feedback.Speed[i]/19.0f));
		}		
		Send_data[4] = (float)wheelInfo.pid[0].feedback;
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
			-amplitudeLimiting(1, para->direction[i]*para->K_speed, para->speedLimit);
}
	return 1;
}

int8_t Lidar_Func(const uint8_t control_flag, const _lidar_message* lidar1, int8_t pos){
	static uint8_t lidar_flag;
	float d1,d2,angle;
	if(control_flag != 1){
		if(lidar_flag){
			lidar_flag = 0;
			chassisPara.Rt = 0;
			chassisPara.yaw.target = 0;
			chassisPara.yaw.angle = 0;
}
		return 0;
}
	else{
		if(lidar1->flag){
		d1 = lidar1->d1;
		d2 = lidar1->d2;
		if(abs(lidar1->angle)<0.5f) angle =0;
		else angle = lidar1->angle/160;
		chassisPara.yaw.target = chassisPara.yaw.target + angle;
		chassisPara.Fb = amplitudeLimiting(1, (d1 - postion[pos][0])/10.0f, 200);
		chassisPara.Lr = amplitudeLimiting(1, (d2 - postion[pos][1])/10.0f, 150);
		lidar_flag = 1;
		return 1;
}
else{
		chassisPara.Rt = 0;
		chassisPara.yaw.target = 0;
		chassisPara.yaw.angle = 0;
		lidar_flag = 1;
	return -1;
}
	
}
}
