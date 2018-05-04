#include "ChassisTask.h"
#include "ComunicateTask.h"
#include "main.h"
static float KalmanFilter_speed(const float ,float ,float );
float Q_speed = 0.02f , R_speed = 7.5f;
_wheelPara wheelInfo = {0};
_chassis chassisPara = {0};
int16_t set = 0;
int8_t lidar_flag;

int const postion[8][2] = {
	{25,360+ 310*4},{25,360+ 310*3},{25,360+ 310*2},{25,360+ 310*1},{25,360+ 310*0},{275+130,25},{275+130*1,25},{275+130*2,25}
};

_pid_Para wheelpid = {
	20,	// kp
	0.50f,	// ki	
	0,	// kd
	1,	// i flag
	0,	// d flag
	800,	// i limit
	6000,	// out limit, limit range from -32768 ~ 32768
	3	//mode flag,  0 for disable, 3 for interval isolate
};

_pid_Para super_wheelpid = {
	27,	// kp
	1.00f,	// ki	
	0,	// kd
	1,	// i flag
	0,	// d flag
	1000,	// i limit
	5500,	// out limit, limit range from -32768 ~ 32768
	3	//mode flag,  0 for disable, 3 for interval isolate
};

_pid_Para chassispid_core = {
	9.0f,	// kp
	0,	// ki
	2.0f,	// kd
	0,	// i flag
	0,	// d flag
	0,	// i limit
	300,	// out limit
	4	//mode flag,  0 for disable
};

_pid_Para chassispid_shell = {
	12.0f, //180,	// kp 6.5
	0.0f,	// ki
	1.0f,	// kd
	0,	// i flag
	0,	// d flag
	0,	// i limit
	1000,	// out limit
	5	//mode flag,  0 for disable
};

int8_t allParaInit(void)
{
	wheelInfo.kpid[0] = wheelpid;
	wheelInfo.kpid[1] = wheelpid;
	wheelInfo.kpid[2] = wheelpid;
	wheelInfo.kpid[3] = wheelpid;
	
	wheelInfo.K_speed = 19.0f;
	wheelInfo.speedLimit = 7000;
	chassisPara.x = 1.8f;
	chassisPara.y = 1.2f;
	chassisPara.pid.shell.k_para = chassispid_shell;
	chassisPara.pid.core.k_para = chassispid_core;
	remote.rc.ch0 = 1024;
	remote.rc.ch1 = 1024;
	remote.rc.ch2 = 1024;
	remote.rc.ch3 = 1024;
	return 1;
} 

void para_update(){
	if(sys1.super_runOr_normal){
		chassisPara.x = 1.8f;
		chassisPara.y = 1.2f;
		wheelInfo.kpid[0] = super_wheelpid;
		wheelInfo.kpid[1] = super_wheelpid;
		wheelInfo.kpid[2] = super_wheelpid;
		wheelInfo.kpid[3] = super_wheelpid;
	}
	else{
		chassisPara.x = 1.5f;
		chassisPara.y = 1.0f;
		wheelInfo.kpid[0] = wheelpid;
		wheelInfo.kpid[1] = wheelpid;
		wheelInfo.kpid[2] = wheelpid;
		wheelInfo.kpid[3] = wheelpid;
	}
}

void rotate_control(){
	//if(relay_flag.status_flag == 0x00){
	pidGet(&chassisPara.pid.shell.k_para,
														&chassisPara.pid.shell.pid,		
														chassisPara.yaw.target,
														chassisPara.yaw.angle);
	//pid for angle_speed
//		chassisPara.pid.shell.pid.Out =0;
			chassisPara.Rt = \
					pidGet(&chassisPara.pid.core.k_para,
														&chassisPara.pid.core.pid,		
//														KalmanFilter_speed(chassisPara.pid.shell.pid.Out,Q_speed,R_speed),
														chassisPara.pid.shell.pid.Out,
														chassisPara.yaw.angle_speed);
//	}

		chassisPara.Rt = chassisPara.Rt ;
		chassisPara.yaw.last_target = chassisPara.yaw.target;
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
		rotate_control();

//		chassisPara.Rt = 0;
	//wheel solute
	
		wheelSolute(&wheelInfo, &chassisPara);
	//pid for wheels
		for(i=0; i<4; i++){
			wheelInfo.out[i] = pidGet(&wheelInfo.kpid[i],
																&wheelInfo.pid[i],
																wheelInfo.targetSpeed[i],	
																(float)(wheelInfo.feedback.Speed[i]));
		}
	return 1;
}
}

int8_t wheelSolute(_wheelPara* para, _chassis* chassis){
	int i;
//	if( abs(chassis->Rt) > 120.0f){
//			chassisPara.x = 1.0f;
//	chassisPara.y = 1.4f;
//	}
//	if(abs(chassis->Fb) > 120.0f || abs(chassis->Lr) > 120.0f){
//		chassisPara.x = 1.8f;
//		chassisPara.y = 0.6f;
//}
//else{
//		chassisPara.x = 1.8f;
//	chassisPara.y = 1.2f;
//}
	para->direction[0] = \
		chassis->x*(-chassis->Fb +chassis->Lr*1.2f) + chassis->y* chassis->Rt;
	para->direction[1] = \
		chassis->x*(chassis->Fb +chassis->Lr*1.2f) + chassis->y* chassis->Rt;
	para->direction[2] = \
		chassis->x*(chassis->Fb -chassis->Lr*1.2f) + chassis->y* chassis->Rt;
	para->direction[3] = \
		chassis->x*(-chassis->Fb -chassis->Lr*1.2f) + chassis->y* chassis->Rt;
	
	for(i=0; i<4; i++ ){
		para->targetSpeed[i] = \
			-amplitudeLimiting(1, para->direction[i]*para->K_speed, para->speedLimit);
	}
	return 1;
}



int8_t Lidar_Func(const uint8_t control_flag,  _lidar_message* lidar1, int8_t pos){
	static uint8_t lidar_flag;
	float d1,d2,angle;
	if(control_flag != 1){
		if(lidar_flag){
			lidar_flag = 0;
}
		return 0;
}
	else{
		if(lidar1->flag){
		d1 = lidar1->d1;
		d2 = lidar1->d2;
		if(abs(lidar1->angle)<1.0f) angle =0;
		else angle = lidar1->angle/1000.0f;
		chassisPara.yaw.target = chassisPara.yaw.target + angle;
		chassisPara.Fb = amplitudeLimiting(1, (d1 )/10.0f, 200);
		chassisPara.Lr = amplitudeLimiting(1, (d2 )/10.0f, 150);
		lidar_flag = 1;
	//	lidar1->flag = 0;
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

static float KalmanFilter_speed(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }

