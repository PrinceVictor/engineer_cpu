#include "PidTask.h"
#include "ChassisTask.h"
float Scale[5] = { 0 , 10 , 20 , 30 , 40  };
float k[5] = { 1.0f , 0.8f , 0.65f , 0.5f , 0.45f };

int16_t pidGet(_pid_Para* pidPara,
							_pid_Out* pidOut,
							float target,
							int16_t feedback)
{
	float last_target = pidOut->target;
	pidOut->target = target;
	pidOut->feedback = (float)feedback;
	pidOut->error = target - (float)feedback;
	if(!pidPara->flag){
		return 0;
}
	else{
		
		switch(pidPara->flag){
		case 1:  break;
		case 2:  break;
		case 3: {
			pidOut->error = pidOut->error * 0.5f;
			if(pidOut->target == 0){
				if(abs(pidOut->error)<20.0f) pidOut->error = 0;
			}
			if((abs(pidOut->error) < 400) && (pidOut->target != 0)) pidPara ->i_flag = 1;
			else {
				pidPara ->i_flag = 0;
				pidOut -> i_interval = 0;
				pidOut ->i_Out = 0;
			}
			break;
}		
		case 4:  
		{
			pidOut->error = pidOut->error * 0.1f;
			if(abs(pidOut->target)<10.0f && abs(pidOut->error)<0.5f  && !chassisPara.yaw.target_changeMode ) {
				pidOut->error = 0;
			}
//			if((abs(pidOut->error) > 30 ) && chassisPara.yaw.target_changeMode) pidPara ->i_flag = 1;
//			else {
//				pidPara ->i_flag = 0;
//				pidOut -> i_interval = 0;
//			}
//			if((abs(pidOut->error) < 3) &&(chassisPara.yaw.last_target == chassisPara.yaw.target)) pidOut->error = 0;
//			else{
//				if(abs(pidOut->error) > 120) pidOut->error =pidOut->error* 0.9f;
//				else if(abs(pidOut->error)>70)  pidOut->error = pidOut->error* 0.75f;
//				else if(abs(pidOut->error)>20)  pidOut->error = pidOut->error* 0.60f;
//				else if(abs(pidOut->error)>7)  pidOut->error = pidOut->error* 0.45f;
//				else  pidOut->error = 0;
//				break;
//			}
			break;
}
		case 5: {
			if((abs(pidOut->error) < 0.5f) &&(!chassisPara.yaw.target_changeMode)) pidOut->error = 0;
			else {
				pidOut->error = amplitudeLimiting(1, pidOut->error, 40);
				Subsection_PID(1, pidOut->error, 5, k,  Scale);
				break;
			}
			break;
}
	
		default : break;
}
}
	pidOut->p_Out = pidPara->kp * pidOut->error;
	pidOut->Out = pidOut->p_Out;

	if(pidPara->i_flag){
		pidOut->i_interval += pidOut->error;
		pidOut->i_Out = pidOut->i_interval * pidPara->ki;
		pidOut->i_Out = amplitudeLimiting(pidPara->i_flag,
											pidOut->i_Out,
											pidPara->ki_limit);
		pidOut->Out = pidOut->Out + pidOut->i_Out;
	}
	if(pidPara->d_flag){
		pidOut->d_Out = (pidOut->error - pidOut->last_error)* pidPara->kd;
		pidOut->last_error = pidOut->error;
		pidOut->Out = pidOut->p_Out + pidOut->d_Out;
	}
	pidOut->Out = amplitudeLimiting(pidPara->flag,
										pidOut->Out,
										pidPara->outlimit);
	return (int16_t)pidOut->Out;
}
