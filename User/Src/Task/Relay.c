#include "Relay.h"

_relay relay ={0};
_relay_flag relay_flag ={0};
_laser_sensor redlaser = {0}; 
/*
void scan_relayflag(_relay* relay_flag, _RC_Ctl* remote){
	
	
	relay_flag->forward_leg_flag = 0;
	relay_flag->backward_leg_flag = 0;
	
	relay_flag->support_leg_flag = 0;
	relay_flag->middle_leg_flag = 0;                                //�м�̧��
	
	relay_flag->rescue_left_flag = 0;
	relay_flag->rescue_right_flag = 0;
	
	relay_flag->distribute_Bigbullet_flag = 0;                       // ���ȡ��
	relay_flag->distribute_up_Samllbullet_flag = 0;          //�н�
	
	relay_flag->distribute_down_Samllbullet_flag = 0;    // ���ӵ�����
	relay_flag->distribute_stretch_flag = 0;       //С���ֳ�������
	relay_flag->bullet_take_flag = 0;            //С������
	relay_flag->bullet_take_stretch_flag = 0;             //�ֵ�װ�����
	
}	
*/