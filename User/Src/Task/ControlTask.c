#include "ControlTask.h"

_sysState sys = {1};

int16_t output[4] = {
	100, 100, 100, 100
};

int flag = 0;
int duoji_flag = 0;

int8_t runControl(_sysState* sys){
	
	switch(sys->state){
		case init:{
			if(allParaInit())
			{
				holder_Init();
				sys->state = run;
				flag = 1;
			}
			break;
		}
		case run:{
			#if 0
//			if(PAin(1)==0 && duoji_flag==0)
//				{
//					TIM_SetCompare4(TIM3,11);//11,22
//					duoji_flag=1;
//				}
//	
//			if(remote.rc.s2==1)
//				{
//					TIM_SetCompare4(TIM3,22);
//					duoji_flag=0;
//				}
			#endif
			sys->state = commuiModeChange(&sys->remoteOrkeyboard,
															&remote, 
															&chassisPara);
			angle_update();

//			Key_detect(&remote);
//			canTrans(1, 3, &canM, Key_detect(&remote));
//			mode 1 to transferType,  mode 2 to read speed, 3 to read position
//			chassisControl(1);		
				canTrans(chassisControl(1), 1, &canM, wheelInfo.out);
//			canTrans(holder_Control(0), 2, &canM, motor.out);
//			can1Trans(1);
			break;
		}
		case stop:{
			sys->state = commuiModeChange(&sys->remoteOrkeyboard,
															&remote, 
															&chassisPara);
			canTrans(flag, 0, &canM, wheelInfo.out);
			break;
		}
		case auto_landing:{
			commuiModeChange(&sys->remoteOrkeyboard,
															&remote, 
															&chassisPara);
			if(!relay_flag.autolanding){
				canTrans(1, 3, &canM, Key_detect(&remote));
				sys->state = 2;
			}
			angle_update();
//			Key_detect(&remote);
			canTrans(1, 3, &canM, Key_detect(&remote));
			canTrans(chassisControl(1), 1, &canM, wheelInfo.out);
//			canTrans(chassisControl(1), 1, &canM, wheelInfo.out);
			
//			can1Trans(1);
			break;
		}
		case manual_landing:{

			commuiModeChange(&sys->remoteOrkeyboard,
														&remote, 
														&chassisPara);
			
			if(!relay_flag.manuallanding){
				canTrans(1, 3, &canM, Key_detect(&remote));
				sys->state = 2;
}
			else{
			angle_update();
			canTrans(1, 3, &canM, Key_detect(&remote));
			canTrans(chassisControl(1), 1, &canM, wheelInfo.out);
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
	
		chassisPara.yaw.angle_speed =  sensor.gyro.radian.z * 57.3f  ;
if(abs(chassisPara.yaw.angle_speed) < 3.0f){
		chassisPara.yaw.angle_speed = 0;
}
		chassisPara.yaw.angle += chassisPara.yaw.angle_speed / 1000.0f;
}

int16_t* Key_detect(const _RC_Ctl* remote1){
	int16_t * flag;
	static uint8_t key_q_test =0;
	static uint8_t key_e_test =0;
	static uint8_t key_qe_test =0;
	static uint8_t key_press_flag = 0;
	static uint16_t key;
	static uint32_t count = 0;	
	static uint32_t relay_count = 0;
	static uint8_t sensor_12 = 0;
	static uint8_t sensor_34 = 0;
	static uint8_t sensor_56 = 0;
	static uint8_t sensor = 0;

	uint8_t press_flag  = 0;
	 

	if(key == remote1->key.v && key)
	{
		count++;
		if(count > 120) press_flag = 1;
}
	else{
		count = 0;
		press_flag = 0;
		key = remote1->key.v;
}

	if(remote1->key.v && !press_flag){
		key_press_flag = 0;
}

	if( !relay_flag.manuallanding && !relay_flag.can1_flag && !relay_flag.autolanding){
		*flag = relay_flag.can1_flag;
		key_qe_test = 0;
		key_q_test = 0;
		key_e_test = 0;
		return flag;
}
	else{
		if(relay_flag.manuallanding){
			if(!key_press_flag && press_flag){
			if((remote1->key.v&key_Q) && (remote1->key.v&key_E)) {
				if(!key_qe_test){
					relay_flag.can1_flag = 0x01;
					key_qe_test = 1;
					key_q_test = 0;
					key_e_test = 0;
				}
				else{
					relay_flag.can1_flag = 0x17;
					key_qe_test = 0;
					key_q_test = 1;
					key_e_test = 1;
				}
				key_press_flag = 1;
			}
			else if(remote1->key.v & key_Q) {
				if(!key_q_test){
					relay_flag.can1_flag = 0x02;
					key_q_test = 1;
				}
				else{
					relay_flag.can1_flag = 0x15;
					key_q_test = 0;
				}
				key_press_flag = 1;
			}
			else if(remote1->key.v & key_E) {
				if(!key_e_test){
					relay_flag.can1_flag = 0x03;
					key_e_test = 1;
				}
				else{
					relay_flag.can1_flag = 0x16;
					key_e_test = 0;
				}
				key_press_flag = 1;
			}
			}
			*flag = relay_flag.can1_flag ;
			*flag = *flag & 0x00ff;
			*flag = *flag | 0xb000;
		}
		else if(relay_flag.autolanding){
//			chassisPara.Fb = 70.0f;
//			if(sensor_12){
//				chassisPara.Fb = 100.0f;
//			}
			if(!key_press_flag && press_flag){
				if((remote1->key.v & key_Q) && relay_flag.take_bullet){
					if(!key_q_test){
						key_q_test = 1;
						relay_flag.take_bullet = 0;
						relay_flag.down = 1;
						relay_count = 0;
					}
					else{
						key_q_test = 0;
						relay_flag.take_bullet = 1;
						relay_flag.down = 0;
					}
				}
			}
			if(relay_flag.up){
				chassisPara.Fb = 20.0f;
				if(!sensor_56){
					if(!sensor && redlaser.flag == 0x00){
						sensor = 1;
					}
					if(sensor && redlaser.flag == 0x03){
						relay_flag.can1_flag = 0x02;
						sensor_12 = 1;
						sensor = 0;
					}
					else if(sensor_12 && redlaser.flag == 0x0f){
						relay_flag.can1_flag = 0x03;
						sensor_34 = 1;
						sensor_12 = 0;
					}
					else if(sensor_34 && redlaser.flag == 0x3f){
						relay_flag.can1_flag = 0x01;
						sensor_56 = 1;
						sensor_34 = 0;
					}
				}
				else if(sensor_56){
					if(redlaser.flag == 0x03){
						relay_flag.can1_flag = 0x02;
						sensor_12 = 1;
					}
					else if(sensor_12 && redlaser.flag == 0x0f){
						relay_flag.can1_flag = 0x03;
						sensor_34 = 1;
					}
					else if(sensor_34 && redlaser.flag == 0x3f){
						relay_flag.can1_flag = 0x01;
						relay_flag.up = 0;
						relay_flag.take_bullet = 1;
						chassisPara.Fb = 0.0f;
					}
				}
			}
			else if(relay_flag.down){
				chassisPara.Fb = -20.0f;
				if(sensor_12){
					if(redlaser.flag == 0x00){
						relay_count ++ ;
						chassisPara.Fb = 0.0f;
					}
					if(sensor_12 && sensor_56 && sensor_34 && redlaser.flag == 0x00 && (relay_count > 200)){
						relay_flag.can1_flag = 0x17;
						relay_count = 0;
					}
					if(redlaser.flag == 0x03 ){
						relay_flag.can1_flag = 0x16;
						sensor_56 = 0;
						sensor_34 = 0;
						relay_count = 0;
					}
					else if(!sensor_56 && !sensor_34 && redlaser.flag == 0x00){
						relay_flag.can1_flag = 0x15;
						sensor_56 = 1;
						sensor_34 = 1;
						sensor_12 = 0;
						relay_count = 0;
					}
				}
				else if(!sensor_12){
					if(redlaser.flag == 0x00){
						relay_count ++ ;
					}
					if(sensor_56 && sensor_34 && redlaser.flag == 0x00 && (relay_count > 1000)){
						relay_flag.can1_flag = 0x17;
						relay_count = 0;
					}
					else if(sensor_56&& sensor_34&& redlaser.flag == 0x03){
						relay_flag.can1_flag = 0x16;
						sensor_56 = 0;
						sensor_34 = 0;
						relay_count = 0;
					}
					else if(!sensor_56 && !sensor_34 && redlaser.flag == 0x00){
						relay_flag.can1_flag = 0x15;
						if(relay_count > 1000){
							relay_flag.down = 0;
							chassisPara.Fb = 0.0f;
						}
					}
				}
			}
			if(!relay_flag.up && !relay_flag.down && !relay_flag.take_bullet){
				relay_flag.autolanding = 0;
				relay_flag.can1_flag = 0x00;
			}
			*flag = relay_flag.can1_flag ;
			*flag = *flag & 0x00ff;
			*flag = *flag | 0xa000;
		}
}
//	key = remote1->key.v;
	return flag;
}

//uint8_t Flag_Inverse(){
//	
//}
