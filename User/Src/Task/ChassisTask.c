#include "ChassisTask.h"
#include "ComunicateTask.h"
#include "main.h"
static float KalmanFilter_speed(const float ,float ,float );
float Q_speed = 0.02f , R_speed = 7.5f;
_wheelPara wheelInfo = {0};
_chassis chassisPara = {0};
int16_t set = 0;
int8_t lidar_flag;
_rotate_move rotate_para = {0};

int const postion[5][2] = {
	{220,360+ 310*0},{220,360+ 310*1+15},{220,360+ 310*2+35},{220,360+ 310*3+45},{220,360+ 310*4+70} //,{275+130,25},{275+130*1,25},{275+130*2,25}
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

_pid_Para rescue_pid = {
	20,	// kp
	0.50f,	// ki	
	0,	// kd
	1,	// i flag
	0,	// d flag
	500,	// i limit
	2000,	// out limit, limit range from -32768 ~ 32768
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
	}
	else{
		chassisPara.x = 1.5f;
		chassisPara.y = 1.0f;
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
	if(flag == 0x03){
		chassisPara.y = 1.6f;
	}
	else {
		chassisPara.y = 1.2f;
	}
		wheelSolute(&wheelInfo, &chassisPara, flag);
	//pid for wheels
	if(flag == 0x01 || flag == 0x03){
		for(i=0; i<4; i++){
			wheelInfo.out[i] = pidGet(&wheelpid,
																&wheelInfo.pid[i],
																wheelInfo.targetSpeed[i],	
																(float)(wheelInfo.feedback.Speed[i]));
		}
	}
	else if(flag == 0x02){
		for(i=0; i<4; i++){
			wheelInfo.out[i] = pidGet(&rescue_pid,
																&wheelInfo.pid[i],
																wheelInfo.targetSpeed[i],	
																(float)(wheelInfo.feedback.Speed[i]));
		}
	}
	return 1;
}
}

int8_t wheelSolute(_wheelPara* para, _chassis* chassis, uint8_t flag){
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



int8_t Lidar_Func(const uint8_t control_flag,  _lidar_message* lidar1,const uint8_t lidar_flag){
	float d1,d2,angle;
	static uint16_t countflag = 0;
	static uint8_t last_position=0;
	static uint8_t rotate =0;
	static uint8_t count1=0,count2=0,count3=0;
	static uint8_t flag_fb=0,flag_lr=0;
	static uint8_t come_in_rotate =0;
	if(control_flag == 0x00){
		return 0;
}
	else if(lidar_flag == 0x01){
		if(rotate_move(lidar1->flag.turn_flag ,  lidar1->angle , lidar1->flag.direction) == 2 ){
			relay_flag.up = 1;
			relay_flag.can1_flag = 0x01;
			lidar.flag.turn_flag = 0;
		}
	}
	else if(lidar_flag == 0x02){
		come_in_rotate =0;
		count3 =0;
		countflag = 0;
		if(rotate_move(lidar1->flag.turn_flag ,  lidar1->angle , lidar1->flag.direction) == 2 ){
			if(lidar.flag.direction){
				relay_flag.bullet_position = 1;
			}
			else if(lidar.flag.direction == 0x01){
			 relay_flag.bullet_position = 1;
			}
			relay_flag.status_flag = 0x03;
			lidar.flag.turn_flag = 0;
			
		}
		
	}
	else if(lidar_flag == 0x03){

//		if(rotate_move(lidar1->flag.turn_flag ,  lidar1->angle , lidar1->flag.direction) == 2 ){
//			relay_flag.status_flag = 0x03;
//		}
		if(relay_flag.bullet_position == 0x00){
			return 0;
		}
		else {
		
		if(!come_in_rotate ){
			chassisPara.Fb = 0;
			chassisPara.Lr = 0;
				countflag ++;	
			}
		if(!come_in_rotate && countflag> 300){
			
			rotate_move(lidar1->flag.turn_flag ,  lidar1->angle , lidar1->flag.direction);
			if(abs(lidar.angle) < 1.0f){
				if(count3 == 0x03){
					come_in_rotate = 1;
				}
				count3++;
			}
			else{
				count3 = 0;
			}
		}
		else if(come_in_rotate){
		if(last_position !=relay_flag.bullet_position ){
			last_position = relay_flag.bullet_position;
			flag_fb = 1;
			flag_lr =1;
//			come_in_rotate = 1;
		}
//		if(!flag_lr && !flag_fb&& !rotate && come_in_rotate){
//			rotate = 1;

//		}

		
		d1 = lidar.d1- postion[relay_flag.bullet_position-1][0];
		if(lidar.d2 == 0){
			flag_lr =1;
			count2 = 0;
			d2 = 0;
		}
		else{
			d2 = lidar.d2- postion[relay_flag.bullet_position-1][1];
		}
		if(abs(d1)<15){
			d1 = 0;
			count1++;
			if(count1 == 4){
				flag_fb =0;
			}
		}
		else{
			count1 = 0;
		}
		if(abs(d2)<10){
			d2 = 0;
			count2++;
			if(count2 == 4){
				flag_lr =0;
			}
		}
		else{
			count2 = 0;
		}
		if(flag_lr|| flag_fb){

		chassisPara.Lr = amplitudeLimiting(1, (d2 )/12.0f, 120);
		chassisPara.Fb = -amplitudeLimiting(1, (d1 )/12.0f, 120);   //amplitudeLimiting(1, (d1 )/10.0f, 200);
	}
	else{
		chassisPara.Lr  = 0;
		chassisPara.Fb = 0;
	}
		
}
	}
	}
	else if(lidar_flag == 0x00){
		chassisPara.yaw.target = 0;
		chassisPara.yaw.angle = 0;
		chassisPara.Fb = 0;   //amplitudeLimiting(1, (d1 )/10.0f, 200);
		chassisPara.Lr = 0;
	}
		return 1;
}

//0 turn left, 1 turn right, target for turn digrees
int8_t rotate_move(uint8_t flag, float target, uint8_t direction){
	static float angle_temp = 0;
  static uint32_t count = 0;
	static uint8_t count_flag = 0;
	static uint8_t last_direction = 0;
	static float last_target =0;
	float target_count, target_input = 0;
	
	if(!flag){
		return 0;
	}
	
	if(target != last_target){
		count = 0;
		last_direction = direction;
		last_target = target ;
		angle_temp = chassisPara.yaw.target;
		count_flag = 1;
	}
	if(last_direction != direction){
		angle_temp = chassisPara.yaw.target;
		last_direction = direction;
	}
	if(count_flag){
		count++;
	}
	target_count = count/1000.0f*3.141592f;
	
	if(target_count>1.5707f){
		target_input = last_target;
		count_flag = 0;
	}
	else target_input = last_target*sin(target_count);
	
	if(direction == 0){
		chassisPara.yaw.target = angle_temp +  target_input;
		if(abs(angle_temp +target -chassisPara.yaw.angle)<2.0f){
		last_target = 0;
		count_flag = 0;
		rotate_para.flag =0;
		return 2;
	}
	}
	else if(direction == 1){
		chassisPara.yaw.target = angle_temp - target_input;
		if(abs(angle_temp -target -chassisPara.yaw.angle)<2.0f){
		last_target = 0;
		count_flag = 0;
		rotate_para.flag =0;
		return 2;
	}
	}
		return 1;
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

