#include "Relay.h"

_relay relay ={0};
_relay_flag relay_flag ={0};
_laser_sensor redlaser = {0}; 

_pid_Out move_base = {0};
_pid_Para move_base_pid = {
	12,	// kp
	0.0f,	// ki	
	0,	// kd
	0,	// i flag
	0,	// d flag
	0,	// i limit
	2000,	// out limit, limit range from -32768 ~ 32768
	1	//mode flag,  0 for disable, 3 for interval isolate
};

float base_target = 0;

int16_t* Key_detect(const _RC_Ctl remote1){
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
	static uint8_t sensor_ = 0;
	uint8_t press_flag  = 0;
	 

	if(key == remote1.key.v && key)
{
		count++;
		if(count > 120) press_flag = 1;
}
	else{
		count = 0;
		press_flag = 0;
		key = remote1.key.v;
}
	if(remote1.rc.s1 == 1 &&remote1.rc.s2 == 2){
		relay_flag.take_bullet = 0;
		relay_flag.down = 1;
		relay_count = 0;
	}
	else if(remote1.rc.s2 != 2){
		relay_flag.take_bullet = 1;
		relay_flag.down = 0;
//		relay_count = 0;
	}

	if(remote1.key.v && !press_flag){
		key_press_flag = 0;
}

	if( !relay_flag.manuallanding && !relay_flag.can1_flag && !relay_flag.autolanding){
		*flag = relay_flag.can1_flag;
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
		return flag;
}
	else{
		if(relay_flag.manuallanding){
			if(!key_press_flag && press_flag){
			if((remote1.key.v&key_Q) && (remote1.key.v&key_E)) {
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
			else if(remote1.key.v & key_Q) {
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
			else if(remote1.key.v & key_E) {
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
			if(!key_press_flag && press_flag){
				if((remote1.key.v & key_E) ){
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
				auto_move(base_target);
				if(!sensor_56){
					if(!sensor_ && redlaser.flag == 0x00){
						sensor_ = 1;
						base_target = 500;
					}
					if(sensor_ && redlaser.flag == 0x03){
						relay_flag.can1_flag = 0x02;
						sensor_12 = 1;
						sensor_ = 0;
						angle_clear();
						base_target = 2000;
					}
					else if(sensor_12 && redlaser.flag == 0x0f){
						relay_flag.can1_flag = 0x03;
						sensor_34 = 1;
						sensor_12 = 0;
						angle_clear();
						base_target = 800;
					}
					else if(sensor_34 && redlaser.flag == 0x3f){
						relay_flag.can1_flag = 0x01;
						sensor_56 = 1;
						sensor_34 = 0;
						angle_clear();
						base_target = 800;
					}
				}
				else if(sensor_56){
					if(redlaser.flag == 0x03){
						relay_flag.can1_flag = 0x02;
						sensor_12 = 1;
						angle_clear();
						base_target = 1100;
					}
					else if(sensor_12 && redlaser.flag == 0x0f){
						relay_flag.can1_flag = 0x03;
						sensor_34 = 1;
						angle_clear();
						base_target = 200;
					}
					else if(sensor_34 && redlaser.flag == 0x3f){
						relay_flag.can1_flag = 0x01;
						relay_flag.up = 0;
						relay_flag.take_bullet = 1;
						angle_clear();
						base_target = 1100;
					}
				}
			}
			else if(relay_flag.down){
				if(sensor_12){
					if(redlaser.flag == 0x00){
						relay_count ++ ;

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
					if(sensor_56 && sensor_34 && redlaser.flag == 0x00 && (relay_count > 800)){
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
						if(relay_count > 800){
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
//	key = remote1.key.v;
	return flag;
}

int8_t auto_move(float target){
	int i = 0;
	static float last_target = 0;
	static float target_input = 0;
	static uint8_t count_flag = 0;
	static uint16_t count =0;
//	if(last_target != target){
//		last_target = target;
//		count_flag = 1;
//		target_input = 400;
//	}
//	if(count_flag){
//		count ++ ;
//		if(count > 100){
//			count = 0;
//			count_flag = 0;
//		}
//	}
//	else{
		target_input = target;
//	}
	
	wheelInfo.targetSpeed[0] = pidGet(&move_base_pid, &move_base,target_input, wheelInfo.info.x);
	wheelInfo.targetSpeed[1] = -pidGet(&move_base_pid, &move_base,target_input, wheelInfo.info.x);
	wheelInfo.targetSpeed[2] = -pidGet(&move_base_pid, &move_base,target_input, wheelInfo.info.x);
	wheelInfo.targetSpeed[3] = pidGet(&move_base_pid, &move_base,target_input, wheelInfo.info.x);
	for(i=0; i<4; i++){
		
		wheelInfo.out[i] = pidGet(&wheelInfo.kpid,
															&wheelInfo.pid[i],
															wheelInfo.targetSpeed[i],
															(float)(wheelInfo.feedback.Speed[i]));}	
	canTrans(1, 1, &canM, wheelInfo.out);
	
	return 1;
}

float target_process(float target){
	
}
