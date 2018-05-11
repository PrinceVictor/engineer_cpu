#include "Relay.h"
#include "main.h"
_relay relay ={0};
_relay_flag relay_flag ={0};
_laser_sensor redlaser = {0}; 

_pid_Out move_base = {0};
_pid_Para move_base_pid = {
	150,	// kp
	0.0f,	// ki	
	0,	// kd
	0,	// i flag
	0,	// d flag
	0,	// i limit
	90,	// out limit, limit range from -32768 ~ 32768
	6	//mode flag,  0 for disable, 3 for interval isolate
};

float base_target = 0;
uint8_t front_leg = 0;
uint8_t back_leg = 0;
int16_t relay_trans = 0;

int16_t* Key_detect(){
	uint8_t press_flag  = 0;
	uint8_t mouse_flag_l = 0;
	uint8_t mouse_flag_r = 0;
	static uint8_t  mouse_press_flag_r;
	static uint8_t  mouse_press_flag_l;
	
	static uint16_t key1;
	static uint8_t key_q_test =0;
	static uint8_t key_e_test =0;
	static uint8_t key_qe_test =0;
	static uint8_t key_press_flag = 0;
	static uint8_t stick_press_flag = 0;
	static uint8_t	next_step = 0;
	
	static uint16_t count = 0;
	static uint16_t allocate_count = 0;
	static uint16_t mouse_count_l = 0;
	static uint16_t mouse_count_r = 0;	
	static uint32_t relay_count = 0;
	
	static uint8_t sensor_12 = 0;
	static uint8_t sensor_34 = 0;
	static uint8_t sensor_56 = 0;
	static uint8_t sensor_ = 0;

	static uint8_t allocate_up = 0;
	static uint8_t allocate_sketch = 0;
	static uint8_t allocate_up_floor = 0;
	static uint8_t allocate_down_floor = 0;
	
	static uint8_t sensor_rescue = 0;
	static uint8_t sensor_rescue12 = 0;
	static uint8_t sensor_next_step= 0;
	
	static uint8_t movement = 0;
	if(key1 == remote.key.v && key1)
	{
		count++;
		if(count > 100) press_flag = 1;
	}
	else{
		count = 0;
		press_flag = 0;
		key1 = remote.key.v;
}

	if(remote.mouse.press_r){
		mouse_count_r++;
		if(mouse_count_r > 50){
			mouse_flag_r =1;
		}
	}
	else{
		mouse_count_r = 0;
		mouse_flag_r = 0;
	}
	
	if(remote.mouse.press_l){
		mouse_count_l++;
		if(mouse_count_l > 50){
			mouse_flag_l =1;
		}
	}
	else{
		mouse_count_l = 0;
		mouse_flag_l = 0;
	}
	
	if(remote.key.v && !press_flag){
		key_press_flag = 0;
	}
	if(remote.mouse.press_l && !mouse_flag_l){
		mouse_press_flag_l =0;
	}
	if(remote.mouse.press_r && !mouse_flag_r){
		mouse_press_flag_r =0;
	}
	// ¾ÈÔ®

	if( !relay_flag.manuallanding && !relay_flag.can1_flag && !relay_flag.autolanding && !relay_flag.rescue && !relay_flag.allocate_flag){
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
		front_leg = 0;
		back_leg = 0;
		next_step = 0;
		sensor_rescue = 0;
		sensor_rescue12 = 0;
}

	else if(relay_flag.allocate_flag|| relay_flag.allocate_finished_flag){
		allocate_count++;
		if(relay_flag.allocate_flag){
			if(!mouse_press_flag_r && mouse_flag_r ){
				if(!relay_flag.allocate_hero_base){
					relay_flag.allocate_hero_base =1;
					relay_flag.can1_flag = 0x09;
				}
				else if(relay_flag.allocate_hero_base){
					relay_flag.allocate_hero_base =0;
					relay_flag.can1_flag = 0x10;
				}
				mouse_press_flag_r = 1;
			}
			else if(!mouse_press_flag_l && mouse_flag_l ){
				if(!relay_flag.allocate_infantry){
					relay_flag.allocate_infantry = 1;
					relay_flag.can1_flag = 0x13;
					allocate_up_floor = 1;
					allocate_count = 0;
				}
				else if(relay_flag.allocate_infantry){
					relay_flag.allocate_infantry = 0;
					relay_flag.can1_flag = 0x26;
					allocate_down_floor = 1;
					allocate_count = 0;
				}
				mouse_press_flag_l = 1;
			}
			if(allocate_down_floor && !relay_flag.allocate_infantry && allocate_up_floor && (allocate_count>300)){
				relay_flag.can1_flag = 0x26;
				allocate_down_floor = 0x00;
				allocate_up_floor = 0;
			}
			if(!allocate_down_floor && relay_flag.allocate_infantry && allocate_up_floor && (allocate_count>300)){
				relay_flag.can1_flag = 0x25;
				allocate_down_floor = 1;
			}
			if(relay_flag.allocate_start_flag && !allocate_up && !allocate_sketch){
				allocate_count = 0;
				allocate_up = 1;
				relay_flag.can1_flag = 0x01;
			}
			else if(relay_flag.allocate_start_flag && allocate_up && (allocate_count>500) && !allocate_sketch){
				allocate_count = 0;
//				relay_flag.allocate_start_flag = 0;
				allocate_up = 0;
				allocate_sketch = 1;
				relay_flag.can1_flag = 0x18;
			}
			else if(relay_flag.allocate_start_flag && !allocate_up && (allocate_count>500) && allocate_sketch){
				allocate_count = 0;
				allocate_up = 1;
				relay_flag.allocate_start_flag = 0;
				allocate_sketch = 1;
				relay_flag.can1_flag = 0x11;
			}
	}
	else if(relay_flag.allocate_finished_flag){
		if(allocate_sketch && allocate_up){
				allocate_sketch =0;
			allocate_count =0;
			relay_flag.can1_flag = 0x12;
		}
		else if(!allocate_sketch && allocate_up && (allocate_count > 500)){
			allocate_up =0;
			allocate_count =0;
			relay_flag.can1_flag = 0x00;
			relay_flag.allocate_finished_flag = 0;
		}
	}
		
	}
	else if(relay_flag.rescue == 1){	
	
				if(!mouse_press_flag_r && mouse_flag_r ){
			relay_flag.can1_flag = 0x21;
			sensor_rescue = 0;
			sensor_rescue12 = 1;
		}
		
	 else if(!relay_flag.rescue_catched){
			
		if(!mouse_press_flag_l && mouse_flag_l ){
			relay_flag.rescue_catched = 1;
			sensor_rescue = 0;
			sensor_rescue12 = 0;
			sensor_next_step = 0;
			mouse_press_flag_l = 1;
		}
		if((redlaser.flag2 == 0x03) && !sensor_rescue ){
			sensor_next_step = 1;
			sensor_rescue = 1;
		}
		else if((redlaser.flag2 == 0x00) && sensor_rescue ){
			relay_flag.can1_flag = 0x20;
//			sensor_rescue = 0;
		}
	}
	else if(relay_flag.rescue_catched){
		if(sensor_rescue12 ){
			relay_flag.can1_flag = 0x00;
			sensor_rescue = 0;
			sensor_rescue12 = 0;
			relay_flag.rescue_catched = 0;
			relay_flag.rescue = 0;
			mouse_press_flag_r = 1;
		}
//		else if(!mouse_press_flag_l && mouse_flag_l ){
//			relay_flag.rescue_catched = 1;
//			sensor_rescue = 0;
//			sensor_rescue12 = 0;
//			sensor_next_step = 0;
//		}
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
			if(relay_flag.status_flag == 0x01 && !relay_flag.up && !relay_flag.down && !relay_flag.take_bullet){
				Lidar_Func(relay_flag.status_flag,&lidar,lidar.flag.status_flag);
				canTrans(chassisControl(1), 1, &canM, wheelInfo.out);
			}
			if(relay_flag.up){
				if(redlaser.flag== 0x00){
				relay_count++;
				}
				auto_move(base_target,front_leg,back_leg);
				if(!sensor_56){
					if(!sensor_ && redlaser.flag == 0x00 && relay_count>100){
//						base_target = lidar.get_x;
						base_target = 40;
						front_leg = 1;
						back_leg = 1;
						sensor_ = 1;
						relay_count = 0;
					}
					if(sensor_ && redlaser.flag == 0x03){
						relay_count++;
						if(!sensor_12){
						relay_count = 0;
						}
						if(relay_count < 200){
							base_target = 0;  
						}
						else {
							base_target = 25;
							sensor_ = 0;
						}
						relay_flag.can1_flag = 0x02;
						sensor_12 = 1;
					
						angle_clear2();
//						base_target += 950; //850;
						front_leg = 1;
						back_leg = 1;
					}
					else if(sensor_12 && redlaser.flag == 0x0f){
						relay_count++;
						if(!sensor_34){
						relay_count = 0;
						}
						if(relay_count < 500){
							base_target = 0;  
						}
						else {
							base_target = 30;
							sensor_12 = 0;
						}
						relay_flag.can1_flag = 0x03;
//						base_target = 20;
//						if(sensor_12){
//							relay_count = 0;
//						}
//						if(relay_count < 50){
//							base_target = 0;  
//						}
//						else {
//							base_target = 0;
//							sensor_12 =0;
//						}
						sensor_34 = 1;
//						sensor_12 = 0;
						angle_clear2();
//						base_target = 700;
						front_leg = 1;
						back_leg = 1;
//						relay_count ++; 
//						if(relay_count < 100){
//							back_leg =0;
//							
//						}
					}
					else if(sensor_34 && redlaser.flag == 0x3f){
						relay_flag.can1_flag = 0x01;
						sensor_56 = 1;
						angle_clear2();
						base_target = 0;
						front_leg = 1;
						back_leg = 1;
						relay_count = 0;
						next_step = 0;
					}
				}
				else if(sensor_56){
					 if(!next_step && redlaser.flag == 0x00 && (relay_count>200)){
						angle_clear2();
						base_target = 35;
						sensor_34 = 1;
						next_step = 1;
						front_leg = 1;
						back_leg = 1;
						relay_count =0;
					}
					else if(next_step && redlaser.flag == 0x03){
						relay_flag.can1_flag = 0x02;
						relay_count++;
						if(!sensor_12){
							relay_count = 0;
						}
						if(relay_count< 300){
							base_target =  0;
						}
						else{
							base_target = 25;
							next_step = 0;
							//relay_count =0;
						}
						sensor_12 = 1;
						angle_clear2();
		//				base_target = 20;
						front_leg = 1;
						back_leg = 1;
						
//						relay_count++;
//						if(relay_count < 100){
//							front_leg = 0;
//						}
					}
					else if(sensor_12 && redlaser.flag == 0x0f){
						relay_flag.can1_flag = 0x03;
						relay_count ++;
						if(sensor_34){
							relay_count = 0;
						}
						if(relay_count< 500){
							base_target =  0;
						}
						else{
							base_target = 30;
						}
//						if(sensor_34){
//							relay_count = 0;
//						}
						sensor_34 = 0;
						angle_clear2();
//						base_target = 18;
						front_leg = 1;
						back_leg = 1;
						
						relay_count ++; 
//						if(relay_count < 100){
//							back_leg =0;
//						}
					}
					else if(!sensor_34 && redlaser.flag == 0x3f){
						relay_flag.can1_flag = 0x04;
						relay_flag.up = 0;
						relay_flag.take_bullet = 1;
						angle_clear2();
						base_target = 0;
						front_leg = 1;
						back_leg = 1;
						sensor_34 = 1;
						relay_count =0;
					}
				}
			}
			else if(relay_flag.take_bullet){
				relay_count ++;
				if(relay_flag.status_flag == 0x01 && (relay_count>100)){
					relay_flag.status_flag = 0x02;
					relay_count = 0;
					chassisPara.Fb = 0;
					chassisPara.Lr = 0;
					chassisPara.Rt = 0;
					canTrans(chassisControl(1), 1, &canM, wheelInfo.out);
				}
				else if(relay_flag.status_flag == 0x02 && lidar.flag.status_flag == 0x02){
					Lidar_Func(relay_flag.status_flag,&lidar,lidar.flag.status_flag);
					canTrans(chassisControl(1), 1, &canM, wheelInfo.out);
					movement = 0;
					relay_count = 0;
				}
				else if(!movement && relay_flag.status_flag == 0x03 && lidar.flag.status_flag == 0x03&& (relay_count>300)){
					movement = 1;
					relay_count = 0;
					chassisPara.Fb = 0;
					chassisPara.Lr = 0;
					chassisPara.Rt = 0;
					canTrans(chassisControl(1), 1, &canM, wheelInfo.out);
				}
				else if(movement && relay_flag.status_flag == 0x03 && lidar.flag.status_flag == 0x03){
					Lidar_Func(relay_flag.status_flag,&lidar,lidar.flag.status_flag);
					canTrans(chassisControl(1), 1, &canM, wheelInfo.out);
				}
				else{
					chassisPara.Fb = 0;
					chassisPara.Lr = 0;
					chassisPara.Rt = 0;
					canTrans(chassisControl(1), 1, &canM, wheelInfo.out);
				}
//				base_target = 0;
//				front_leg = 1;
//				back_leg = 1;
//				auto_move(base_target,front_leg,back_leg);
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
						base_target = -0;
						front_leg = 1;
						back_leg = 1;
						relay_count = 0;
					}
					else if(sensor_12 && sensor_56 && sensor_34 && redlaser.flag == 0x3f){
						relay_flag.can1_flag = 0x22;
						angle_clear();
						base_target = -25;
						front_leg = 1;
						back_leg = 1;
						relay_count = 0;
					}
					if(redlaser.flag == 0x03 ){
						relay_flag.can1_flag = 0x16;
						sensor_56 = 0;
						sensor_34 = 0;
						angle_clear2();
						base_target = -25;
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
						base_target = -35;
						front_leg = 1;
						back_leg = 1;
						relay_count = 0;
					}
				}
				else if(!sensor_12){
					if(redlaser.flag == 0x00){
						relay_count ++ ;
					}
					if(sensor_56 && sensor_34 && redlaser.flag == 0x00 && (relay_count > 300)){
						relay_flag.can1_flag = 0x17;
						angle_clear2();
						base_target = -30;
						front_leg = 1;
						back_leg = 1;
						relay_count = 0;
						sensor_34 = 0;
					}
					else if(sensor_56&& !sensor_34&& redlaser.flag == 0x03){
						relay_flag.can1_flag = 0x16;
						sensor_56 = 0;
						sensor_34 = 0;
						angle_clear2();
						base_target = -25;
						front_leg = 1;
						back_leg = 1;
						relay_count = 0;
					}
					else if(!sensor_56 && !sensor_34 && redlaser.flag == 0x00){
						relay_flag.can1_flag = 0x15;
						angle_clear2();
						base_target = -35;
						front_leg = 1;
						back_leg = 1;
						if(relay_count > 500){
							relay_flag.down = 0;
							chassisPara.Fb = 0.0f;
						}
					}
				}
			}
			if(!relay_flag.up && !relay_flag.down && !relay_flag.take_bullet && !relay_flag.status_flag && relay_flag.autolanding){
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

int8_t auto_move(float target, uint8_t front_leg1, uint8_t back_leg1){
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
	
	chassisPara.Fb =target_input;
//	pidGet(&move_base_pid, &move_base,target_input, wheelInfo.info.x);
	
	if(last_target1>0){
	if(chassisPara.Fb <0){
		chassisPara.Fb= 0;
	}
}
else if(last_target1<0){
	if(chassisPara.Fb >0){
		chassisPara.Fb= 0;
	}
}
	rotate_control();

	if(front_leg1 && back_leg1){
		wheelInfo.direction[0] = \
		chassisPara.x*(-chassisPara.Fb +chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[1] = \
		chassisPara.x*(chassisPara.Fb +chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[2] = \
		chassisPara.x*(chassisPara.Fb -chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[3] = \
		chassisPara.x*(-chassisPara.Fb -chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	}
	else if(front_leg1){
	wheelInfo.direction[0] = \
		chassisPara.x*(-chassisPara.Fb +chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[1] = \
		chassisPara.x*(chassisPara.Fb +chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[2] = \
		chassisPara.x*(chassisPara.Fb -chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[3] = \
		chassisPara.x*(-chassisPara.Fb -chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	}
	else if(back_leg1){
	wheelInfo.direction[0] = \
		0*(-chassisPara.Fb +chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[1] = \
		0*(chassisPara.Fb +chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[2] = \
		chassisPara.x*(chassisPara.Fb -chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	wheelInfo.direction[3] = \
		chassisPara.x*(-chassisPara.Fb -chassisPara.Lr*1.2f) + chassisPara.y* chassisPara.Rt;
	}
	else {
		wheelInfo.direction[0] = chassisPara.y* chassisPara.Rt;
		wheelInfo.direction[1] = chassisPara.y* chassisPara.Rt;
		wheelInfo.direction[2] = chassisPara.y* chassisPara.Rt;
		wheelInfo.direction[3] = chassisPara.y* chassisPara.Rt;
	}
	for(i=0; i<4; i++ ){
		wheelInfo.targetSpeed[i] = \
			-amplitudeLimiting(1, wheelInfo.direction[i]*wheelInfo.K_speed, wheelInfo.speedLimit);
	}
	for(i=0; i<4; i++){
		
		wheelInfo.out[i] = pidGet(&wheelpid,
															&wheelInfo.pid[i],
															wheelInfo.targetSpeed[i],
															(float)(wheelInfo.feedback.Speed[i]));}	
//	canTrans(lidar.flag, 1, &canM, wheelInfo.out);
															canTrans(1, 1, &canM, wheelInfo.out);
//	Send_data1[0] = wheelInfo.targetSpeed[0];
//	Send_data1[1] = wheelInfo.targetSpeed[1];		
//	Send_data1[2] = wheelInfo.info.x;
	return 1;
}

