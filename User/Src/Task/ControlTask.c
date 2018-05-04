#include "ControlTask.h"
#define pi 3.1415926f
_sysState sys1 = {1};

int16_t output[4] = {
	100, 100, 100, 100
};

int flag = 0;

float target;
int8_t runControl(){
	
	switch(sys1.state){
		case init:{
			if(allParaInit())
			{
				holder_Init();
				sys1.state = run;
				flag = 1;
			}
			break;
		}
		case run:{
			sys1.state = commuiModeChange(&remote, 
																		&chassisPara);
			angle_update();
			
//			auto_move(target);
//			mode 1 to transferType,  mode 2 to read speed, 3 to read position
//			chassisControl(1);	
			//send_odm_msg2(&wheelInfo.info);			
			canTrans(chassisControl(0), 1, &canM, wheelInfo.out);
			
//			canTrans(holder_Control(0), 2, &canM, motor.out);
//			can1Trans(1);
			break;
		}
		case stop:{
			sys1.state = commuiModeChange(&remote, 
																		&chassisPara);
			canTrans(flag, 0, &canM, wheelInfo.out);
			break;
		}
		case auto_landing:{
			commuiModeChange(&remote, 
											 &chassisPara);
			                                                                   //relay_flag.autolanding = 0;
			if(!relay_flag.autolanding){
				canTrans(1, 3, &canM, Key_detect());
				angle_clear();
				sys1.state = 2;
			}
			angle_update();
			canTrans(1, 3, &canM, Key_detect());
			canTrans(chassisControl(0), 1, &canM, wheelInfo.out);
			
//			canTrans(chassisControl(1), 1, &canM, wheelInfo.out);
//			can1Trans(1);
			break;
		}
		case manual_landing:{

			commuiModeChange(&remote, 
											 &chassisPara);
			
			if(!relay_flag.manuallanding){
				canTrans(1, 3, &canM, Key_detect());
				sys1.state = 2;
				angle_clear();
			}
			else{
			angle_update();
			canTrans(1, 3, &canM, Key_detect());
			canTrans(chassisControl(0), 1, &canM, wheelInfo.out);
			}
			break;
		}
		case reset:{
			break;
		}
		default: break;
}		
	return 1;
}


void angle_update(void){
	
	wheelInfo.info.target_vy=\
	(wheelInfo.targetSpeed[0]+ wheelInfo.targetSpeed[1]- wheelInfo.targetSpeed[2]- wheelInfo.targetSpeed[3])/(19*4)*76.0f/60.0f*2*pi;
	wheelInfo.info.target_vx=\
	(wheelInfo.targetSpeed[0] -wheelInfo.targetSpeed[1]-wheelInfo.targetSpeed[2]+wheelInfo.targetSpeed[3])/(19*4)*76.0f/60.0f*2*pi;
	
	wheelInfo.info.Vy=\
	(wheelInfo.feedback.Speed[0]+ wheelInfo.feedback.Speed[1]- wheelInfo.feedback.Speed[2]- wheelInfo.feedback.Speed[3])/(19*4)*76.0f/60.0f*2*pi;
	wheelInfo.info.Vx=\
	(wheelInfo.feedback.Speed[0] -wheelInfo.feedback.Speed[1]-wheelInfo.feedback.Speed[2]+wheelInfo.feedback.Speed[3])/(19*4)*76.0f/60.0f*2*pi;
	wheelInfo.info.Wz=\
	(-wheelInfo.feedback.Speed[0]-wheelInfo.feedback.Speed[1]-wheelInfo.feedback.Speed[2]-wheelInfo.feedback.Speed[3])/(19*4)*76.0f/(280+227)/60.0f*2*180.0f;
	wheelInfo.info.theta += wheelInfo.info.Wz/1000;
		chassisPara.yaw.angle_speed =  sensor.gyro.radian.z * 57.3f;
	
	wheelInfo.info.x += wheelInfo.info.Vx/1000.0f;
	wheelInfo.info.y += wheelInfo.info.Vy/1000.0f;
	
	if(abs(chassisPara.yaw.angle_speed) < 3.0f){
		chassisPara.yaw.angle_speed = 0;
	}
		chassisPara.yaw.angle += chassisPara.yaw.angle_speed / 1000.0f;
}

void angle_clear(void){
	chassisPara.yaw.angle = 0;
	chassisPara.yaw.target = 0;
	chassisPara.yaw.angle_speed = 0;
	wheelInfo.info.x = 0;
	wheelInfo.info.y = 0;
	wheelInfo.info.theta = 0;	
}

void angle_clear2(void){
	wheelInfo.info.x = 0;
	wheelInfo.info.y = 0;
	wheelInfo.info.theta = 0;	
}


int8_t Auto_mode(const _RC_Ctl* data){

	static uint8_t key_press_flag = 0;
	static uint8_t stick_press_flag = 0;
	if(data->rc.s1 == 1){
		if( data->rc.s2 == 3 && !stick_press_flag){
			relay_flag.autolanding = 1;
			relay_flag.can1_flag = 0x01;
			relay_flag.up = 1;
			stick_press_flag = 1;
			return 3;
		}
		else if(data->rc.s2 == 1){
			relay_flag.autolanding = 0;
			relay_flag.can1_flag = 0x00;
			stick_press_flag = 0;
			return 3;
		}
		else if(data->rc.s2 == 2 && !stick_press_flag){
			relay_flag.autolanding = 1;
			relay_flag.can1_flag = 0x17;
			relay_flag.down = 1;
			stick_press_flag = 1;
			return 2;
		}
		else{
			return 3;
		}
	}
	else if(data->rc.s1 == 2){
		if(!data->key.v){
			key_press_flag = 0;
			return 2;
		}
		else if(data->key.v == key_R){
			if(!key_press_flag){
						if(relay_flag.autolanding)  {
							relay_flag.autolanding = 0;
							relay_flag.can1_flag = 0x00;
							relay_flag.status_flag = 0x00;
								}
					else{
						relay_flag.autolanding = 1;
						relay_flag.status_flag = 0x01;   //calibration island 
//						relay_flag.can1_flag = 0x01;
//						relay_flag.up = 1;
						
						angle_clear();
					}
					key_press_flag = 1;
					return 3;
			}
		}
		else if(data->key.v == (key_Ctrl|key_R)){
			if(!key_press_flag){
						if(relay_flag.autolanding)  {
							relay_flag.autolanding = 0;
							relay_flag.can1_flag = 0x00;
								}
					else{
						relay_flag.autolanding = 1;
						relay_flag.can1_flag = 0x01;
						relay_flag.up = 1;
						
						angle_clear();
					}
					key_press_flag = 1;
					return 3;
			}
		}
		if(data->key.v & key_Shift){
			sys1.super_runOr_normal = 1;
			return 2;
		}
		else{
			sys1.super_runOr_normal = 0;
			return 2;
		}
	}
}
void send_odm_msg2(_wheel_solve * data)
{ 

  uint8_t uart2_send_buff[32]; 
	uart2_send_buff[0] = 0xA0;
	
	uart2_send_buff[1] = BYTE3(data->Vx);
	uart2_send_buff[2] = BYTE2(data->Vx);
	uart2_send_buff[3] = BYTE1(data->Vx);
	uart2_send_buff[4] = BYTE0(data->Vx);
	
	uart2_send_buff[1+4] = BYTE3(data->Vy);
	uart2_send_buff[2+4] = BYTE2(data->Vy);
	uart2_send_buff[3+4] = BYTE1(data->Vy);
	uart2_send_buff[4+4] = BYTE0(data->Vy);
	
	uart2_send_buff[1+8] = BYTE3(chassisPara.yaw.angle_speed);
	uart2_send_buff[2+8] = BYTE2(chassisPara.yaw.angle_speed);
	uart2_send_buff[3+8] = BYTE1(chassisPara.yaw.angle_speed);
	uart2_send_buff[4+8] = BYTE0(chassisPara.yaw.angle_speed);
	
//	uart2_send_buff[1+12] = BYTE3(nuc_tans_count);
//	uart2_send_buff[2+12] = BYTE2(nuc_tans_count);
//	uart2_send_buff[3+12] = BYTE1(nuc_tans_count);
//	uart2_send_buff[4+12] = BYTE0(nuc_tans_count);
	uart2_send_buff[13] = relay_flag.status_flag;
	uart2_send_buff[14] = 0xB0;
	New_Send_Data(uart2_send_buff,15);

}

void HardFault_Handler(void){
	while(1){
	}
}



