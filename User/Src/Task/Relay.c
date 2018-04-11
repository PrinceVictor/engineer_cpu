#include "Relay.h"

_relay relay ={0};
_relay_flag relay_flag ={0};
_laser_sensor redlaser = {0}; 
/*
void scan_relayflag(_relay* relay_flag, _RC_Ctl* remote){
	
	
	relay_flag->forward_leg_flag = 0;
	relay_flag->backward_leg_flag = 0;
	
	relay_flag->support_leg_flag = 0;
	relay_flag->middle_leg_flag = 0;                                //中间抬升
	
	relay_flag->rescue_left_flag = 0;
	relay_flag->rescue_right_flag = 0;
	
	relay_flag->distribute_Bigbullet_flag = 0;                       // 伸出取弹
	relay_flag->distribute_up_Samllbullet_flag = 0;          //夹紧
	
	relay_flag->distribute_down_Samllbullet_flag = 0;    // 大子弹仓门
	relay_flag->distribute_stretch_flag = 0;       //小弹分成两部分
	relay_flag->bullet_take_flag = 0;            //小弹仓门
	relay_flag->bullet_take_stretch_flag = 0;             //分弹装置伸出
	
}	
*/