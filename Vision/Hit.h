#ifndef _HIT_H
#define _HIT_H

#include "stm32f4xx.h" 
#include "sys.h"
#include "Remote.h"
#include "Chassis_Control.h"
#include "Holder.h"
#include "Hit_calculation.h"
#include "Motor_out.h"
#include "Wheel_Speed.h"
#include "Load_motor.h"
#include "OtherConfig.h"
#include "clock.h"
struct RC_Ctl_key
{
	int16_t key_Q;
	int16_t key_S;
	int16_t key_W;
	int16_t key_E;
	int16_t key_R;
	int16_t key_A;
	int16_t key_D;
	int16_t key_F;
	int16_t key_G;
	int16_t key_Z;
	int16_t key_X;
	int16_t key_C;
	int16_t key_V;
	int16_t key_B;
	int16_t key_Shift;
	int16_t key_Ctrl;
};

extern struct RC_Ctl_key RC_Key;

extern float TargetTable[9][2];

extern uint8_t Hit_rev[5];
extern uint8_t Hit[5];
extern uint8_t HitMode;

static void Manual_Hit();
void Hit_Task(uint8_t flag);
void Hit_flag_init(void);
void CalculateKey(void);
#endif


