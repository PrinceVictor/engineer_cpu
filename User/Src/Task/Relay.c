#include "Relay.h"
#include "main.h"
#include "math.h"
_relay relay ={0};
_relay_flag relay_flag ={0};
_laser_sensor redlaser = {0}; 

_pid_Out move_base = {0};
_pid_Para move_base_pid = {
	10,	// kp
	0.0f,	// ki	
	0,	// kd
	0,	// i flag
	0,	// d flag
	0,	// i limit
	40,	// out limit, limit range from -32768 ~ 32768
	2	//mode flag,  0 for disable, 3 for interval isolate
};

float base_target = 0;
int16_t relay_trans = 0;

int16_t* Key_detect(){
	uint8_t press_flag  = 0;
	uint8_t mouse_flag = 0;
	static uint8_t  mouse_press_flag;
	
	static uint16_t key1;
	static uint8_t key_q_test =0;
	static uint8_t key_e_test =0;
	static uint8_t key_qe_test =0;
	static uint8_t key_press_flag = 0;
	static uint8_t stick_press_flag = 0;
	
	static uint16_t count = 0;
	static uint16_t mouse_count = 0;	
	static uint32_t relay_count = 0;
	
	static uint8_t sensor_12 = 0;
	static uint8_t sensor_34 = 0;
	static uint8_t sensor_56 = 0;
	static uint8_t sensor_ = 0;
	static uint8_t front_leg = 0;
	static uint8_t back_leg = 0;
	
	static uint8_t sensor_rescue = 0;
	static uint8_t sensor_rescue12 = 0;
	if(key1 == remote.key.v && key1)
{
		count++;
		if(count > 120) press_flag = 1;
}
	else{
		count = 0;
		press_flag = 0;
		key1 = remote.key.v;
}

	if(remote.mouse.press_r){
		mouse_count++;
		if(mouse_count > 100){
			mouse_flag =1;
		}
	}
	else{
		mouse_count = 0;
		mouse_flag = 0;
	}
	
	if(remote.key.v && !press_flag){
		key_press_flag = 0;
}
	if(remote.mouse.press_r && !mouse_flag){
		mouse_press_flag =0;
	}
	
	if(remote.rc.s1 == 1){
		if(remote.rc.s2 == 2 && !stick_press_flag){
			sensor_12 = 1;
			sensor_34 = 1;
			sensor_56 = 1;
			relay_count = 0;
			stick_press_flag = 1;
		}
		else if(remote.rc.s2 == 3 && !stick_press_flag){
			sensor_12 = 0;
			sensor_34 = 0;
			sensor_56 = 0;
			sensor_ = 0;
			relay_count = 0;
			stick_press_flag = 1;
		}
	}
	else if(remote.rc.s1 == 3 ){
		if( remote.rc.s2 == 2 && !stick_press_flag){
		relay_flag.rescue =1;
		stick_press_flag =1;
		sensor_rescue = 0;
		sensor_rescue12 = 0;
		relay_flag.can1_flag = 0x21;
	}
	else if(remote.rc.s2 != 2 && stick_press_flag){
		stick_press_flag = 0;
		relay_flag.rescue = 0;
		relay_flag.can1_flag = 0x21;
		sensor_rescue = 0;
		sensor_rescue12 = 0;
	}
}
	else {
		stick_press_flag = 0;
	}

	if( !relay_flag.manuallanding && !relay_flag.can1_flag && !relay_flag.autolanding&& !relay_flag.rescue){
		key_qe_test = 0;
		key_q_test = 0;
		key_e_test = 0;
		relay_flag.take_bullet = 0;
		relay_count = 0;
		relay_flag.up = 0;
		relay_flag.down = 0;
		sensor_12 = 0;
		sensor_34 = 0;
		sensor_56 = 0;
		relay_flag.status_flag = 0x00;
		relay_trans = relay_flag.can1_flag;
}
	else if(relay_flag.rescue == 1){
		if(((redlaser.flag>>6) == 3) && !sensor_rescue && !sensor_rescue12){
			relay_flag.can1_flag = 0x21;
			sensor_rescue = 1;
		}
		if(((redlaser.flag>>6)== 0) && sensor_rescue){
			relay_flag.can1_flag = 0x20;
			sensor_rescue = 0;
			sensor_rescue12 = 1;
		}
	}
	else{
		if(relay_flag.manuallanding){
			if(!key_press_flag && press_flag){
			if((remote.key.v&key_Q) && (remote.key.v&key_E)) {
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
			else if(remote.key.v & key_Q) {
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
			else if(remote.key.v & key_E) {
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
		}
		else if(relay_flag.autolanding){
			if(relay_flag.status_flag == 0x01){
				Lidar_Func(relay_flag.status_flag,&lidar,0);
			}
			if((relay_flag.status_flag==0x01) && !mouse_press_flag && mouse_flag){
				relay_flag.status_flag = 0x02;
				mouse_press_flag = 1;
			}
			if(!key_press_flag && press_flag){
				if((remote.key.v & key_E) ){
					if(!key_e_test && relay_flag.take_bullet){
						key_e_test = 1;
						relay_flag.take_bullet = 0;
						relay_flag.down = 1;
						relay_count = 0;
					}
					else if(key_e_test && !relay_flag.take_bullet){
						key_e_test = 0;
						relay_flag.take_bullet = 1;
						relay_flag.down = 0;
					}
				}
			}
			if(relay_flag.up){
				auto_move(base_target,front_leg,back_leg);
				if(!sensor_56){
					if(!sensor_ && redlaser.flag == 0x00){
						sensor_ = 1;
						base_target = 500;
						front_leg = 1;
						back_leg = 1;
						relay_count = 0;
					}
					if(sensor_ && redlaser.flag == 0x03){
						relay_flag.can1_flag = 0x02;
						sensor_12 = 1;
						sensor_ = 0;
						angle_clear2();
						base_target = 850;
						front_leg = 1;
						back_leg = 1;
						
						relay_count++;
						if(relay_count < 100){
							front_leg = 0;
						}
					}
					else if(sensor_12 && redlaser.flag == 0x0f){
						relay_flag.can1_flag = 0x03;
						if(sensor_12){
							relay_count = 0;
						}
						sensor_34 = 1;
						sensor_12 = 0;
						angle_clear2();
						base_target = 700;
						front_leg = 1;
						back_leg = 1;
						relay_count ++; 
						if(relay_count < 100){
							back_leg =0;
						}
					}
					else if(sensor_34 && redlaser.flag == 0x3f){
						relay_flag.can1_flag = 0x01;
						sensor_56 = 1;
						sensor_34 = 0;
						angle_clear2();
						base_target = 700;
						front_leg = 1;
						back_leg = 1;
					}
				}
				else if(sensor_56){
					if(redlaser.flag == 0x03){
						relay_flag.can1_flag = 0x02;
						if(!sensor_12){
							relay_count = 0;
						}
						sensor_12 = 1;
						angle_clear2();
						base_target = 700;
						front_leg = 1;
						back_leg = 1;
						
						relay_count++;
						if(relay_count < 100){
							front_leg = 0;
						}
					}
					else if(sensor_12 && redlaser.flag == 0x0f){
						relay_flag.can1_flag = 0x03;
						if(!sensor_34){
							relay_count = 0;
						}
						angle_clear2();
						base_target = 200;
						front_leg = 1;
						back_leg = 1;
						sensor_34 = 1;
						relay_count ++; 
						if(relay_count < 100){
							back_leg =0;
						}
					}
					else if(sensor_34 && redlaser.flag == 0x3f){
						relay_flag.can1_flag = 0x01;
						relay_flag.up = 0;
						relay_flag.take_bullet = 1;
						angle_clear2();
						base_target = 100;
					}
				}
			}
			else if(relay_flag.down){
				auto_move(base_target,front_leg,back_leg);
				if(sensor_12){
					if(redlaser.flag == 0x00){
						relay_count ++ ;

					}
					if(sensor_12 && sensor_56 && sensor_34 && redlaser.flag == 0x00 && (relay_count > 200)){
						relay_flag.can1_flag = 0x17;
						angle_clear2();
						base_target = -450;
						front_leg = 1;
						back_leg = 1;
						relay_count = 0;
					}
					if(redlaser.flag == 0x03 ){
						relay_flag.can1_flag = 0x16;
						sensor_56 = 0;
						sensor_34 = 0;
						angle_clear2();
						base_target = -450;
						front_leg = 1;
						back_leg = 1;
						relay_count = 0;
					}
					else if(!sensor_56 && !sensor_34 && redlaser.flag == 0x00){
						relay_flag.can1_flag = 0x15;
						sensor_56 = 1;
						sensor_34 = 1;
						sensor_12 = 0;
						angle_clear2();
						base_target = -450;
						front_leg = 1;
						back_leg = 1;
						relay_count = 0;
					}
				}
				else if(!sensor_12){
					if(redlaser.flag == 0x00){
						relay_count ++ ;
					}
					if(sensor_56 && sensor_34 && redlaser.flag == 0x00 && (relay_count > 500)){
						relay_flag.can1_flag = 0x17;
						angle_clear2();
						base_target = -520;
						front_leg = 1;
						back_leg = 1;
						relay_count = 0;
					}
					else if(sensor_56&& sensor_34&& redlaser.flag == 0x03){
						relay_flag.can1_flag = 0x16;
						sensor_56 = 0;
						sensor_34 = 0;
						angle_clear2();
						base_target = -60;
						front_leg = 1;
						back_leg = 1;
						relay_count = 0;
					}
					else if(!sensor_56 && !sensor_34 && redlaser.flag == 0x00){
						relay_flag.can1_flag = 0x15;
						angle_clear2();
						base_target = -200;
						front_leg = 1;
						back_leg = 1;
						if(relay_count > 600){
							relay_flag.down = 0;
							chassisPara.Fb = 0.0f;
						}
					}
				}
			}
			if(!relay_flag.up && !relay_flag.down && !relay_flag.take_bullet && !relay_flag.status_flag){
				relay_flag.autolanding = 0;
				relay_flag.can1_flag = 0x00;
			}
			
//			*flag = *flag & 0x00ff;
//			*flag = *flag | 0xa000;
		}
			
		
}
		relay_trans = relay_flag.can1_flag;
			relay_trans = relay_trans & 0x00ff;
			relay_trans = relay_trans | 0xa000;
return &relay_trans;
}


int8_t auto_move(float target, uint8_t front_leg, uint8_t back_leg, _lidar_message* lidar1){
  static uint32_t  count1 =0;
	static uint32_t  count2 =0;
	static uint8_t count_flag;
	static float last_target1 =0,last_target2 =0;
	float target1_count, target_input = 0;
	int i =0;
	
	if(target != last_target1){
		count1 = 0;
		last_target1 = target ;
		count_flag = 1;
	}
	if(count_flag){
		count1++;
	}
	
	target1_count = count1/2000.0f*3.141592f;
	
	if(target1_count>1.5707f){
		target_input = last_target1;
		count_flag = 0;
	}
	
	else target_input = last_target1*sin(target1_count);
	
	chassisPara.Fb = pidGet(&move_base_pid, &move_base,target_input, wheelInfo.info.x);
	
	rotate_control();
	if(front_leg && back_leg){
		wheelInfo.direction[0] = \
		chassisPara.x*(-chassisPara.Fb +chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[1] = \
		chassisPara.x*(chassisPara.Fb +chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[2] = \
		chassisPara.x*(chassisPara.Fb -chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[3] = \
		chassisPara.x*(-chassisPara.Fb -chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	}
	else if(front_leg){
	wheelInfo.direction[0] = \
		chassisPara.x*(-chassisPara.Fb +chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[1] = \
		chassisPara.x*(chassisPara.Fb +chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[2] = \
		0*(chassisPara.Fb -chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[3] = \
		0*(-chassisPara.Fb -chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	}
	else if(back_leg){
	wheelInfo.direction[0] = \
		0*(-chassisPara.Fb +chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[1] = \
		0*(chassisPara.Fb +chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[2] = \
		chassisPara.x*(chassisPara.Fb -chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[3] = \
		chassisPara.x*(-chassisPara.Fb -chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	}
	
	for(i=0; i<4; i++){
		
		wheelInfo.out[i] = pidGet(&wheelInfo.kpid[i],
															&wheelInfo.pid[i],
															wheelInfo.targetSpeed[i],
															(float)(wheelInfo.feedback.Speed[i]));}	
	canTrans(lidar.flag, 1, &canM, wheelInfo.out);
//	Send_data1[0] = wheelInfo.targetSpeed[0];
//	Send_data1[1] = wheelInfo.targetSpeed[1];		
//	Send_data1[2] = wheelInfo.info.x;
	return 1;
}

