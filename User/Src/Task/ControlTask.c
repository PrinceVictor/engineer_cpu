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
			Key_detect(&remote);
			canTrans(1, 3, &canM, Key_detect(&remote));
//			mode 1 to transferType,  mode 2 to read speed, 3 to read position
			
			
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
		case landing:{
			can1Trans(1);
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
if(abs(chassisPara.yaw.angle_speed) < 1.5f){
		chassisPara.yaw.angle_speed = 0;
}
		chassisPara.yaw.angle += chassisPara.yaw.angle_speed / 1000.0f;
}

int16_t* Key_detect(const _RC_Ctl* remote1){
	int16_t * flag;
	static uint8_t key_press_flag = 0;
	static uint16_t key;
	static uint32_t count = 0;
	uint8_t press_flag  = 0;
	if(remote1->key.v){
		key_press_flag = 0;
}
	if(key == remote1->key.v)
	{
		count++;
		if(count > 150) press_flag = 1;
}
	else{
		count = 0;
		press_flag = 0;
		key = remote1->key.v;
}
	if(!relay_flag.can1_flag){
		*flag = relay_flag.can1_flag;
		return flag;
}
	else{
		if(relay_flag.manuallanding){
			if(!key_press_flag&& press_flag){
			if((remote1->key.v&key_Q) && (remote1->key.v&key_E)) {
				relay_flag.can1_flag = 0x01;
				key_press_flag = 1;
			}
			else if(remote1->key.v&key_Q) {
				relay_flag.can1_flag = 0x02;
				key_press_flag = 1;
			}
			else if(remote1->key.v&key_E) {
				relay_flag.can1_flag = 0x03;
				key_press_flag = 1;
			}
			}
}
}
	*flag = relay_flag.can1_flag;
//	key = remote1->key.v;
	return flag;
}
