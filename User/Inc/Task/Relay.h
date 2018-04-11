#ifndef __RELAY_H__
#define __RELAY_H__

#include "MyFunc.h"
#include "ComunicateTask.h"

#define key_A   0x0004
#define	key_W   0x0001
#define key_S   0x0002
#define key_Q   0x0040//KEYI
#define key_E   0x0080//KEYI
#define key_R   0x0100
#define key_D   0x0008
#define key_F   0x0200
#define key_G   0x0400
#define key_Z   0x0800
#define key_X   0x1000
#define key_C   0x2000
#define key_V   0x4000
#define key_B   0x8000
#define key_Shift   0x0010//KEYI
#define key_Ctrl   0x0020//KEYI

typedef struct{
	int8_t forward_leg_flag;
	int8_t backward_leg_flag;
	int8_t support_leg_flag;
	int8_t middle_leg_flag;
	int8_t rescue_left_flag;
	int8_t rescue_right_flag;
	int8_t distribute_Bigbullet_flag;
	int8_t distribute_up_Samllbullet_flag;
	int8_t distribute_down_Samllbullet_flag;
	int8_t distribute_stretch_flag;
	int8_t bullet_take_flag;
	int8_t bullet_take_stretch_flag;
}_relay;

typedef struct{
	uint8_t can1_flag;
	uint8_t autolanding;
	uint8_t manuallanding;
	uint8_t four_legs;
	uint8_t forward_leg;
	uint8_t back_leg;
	uint8_t support_leg;
	uint8_t up;
	uint8_t down;
	uint8_t take_bullet;
}_relay_flag;

typedef struct{
	uint8_t verifed_code;
	uint8_t flag;
	uint8_t _1st;
	uint8_t _2nd;
	uint8_t _3rd;
	uint8_t _4th;
}_laser_sensor;

extern _relay_flag relay_flag;
extern _laser_sensor redlaser;
#endif

