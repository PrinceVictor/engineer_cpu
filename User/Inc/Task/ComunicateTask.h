#ifndef __COMUNICATETASK_H_
#define __COMUNICATETASK_H_

#include "MyFunc.h"
#include "PidTask.h"
#include "ChassisTask.h"
#include "stm32f4xx_can.h"
#include "math.h"
#include "Referee.h"



#define YAW_SENSITY 0.005f		//鼠标yaw轴灵敏度
#define PITCH_SENSITY 0.03f	//鼠标pitch轴灵敏度

#define LEFT_LIMINT_ANGLE 40.0f //角度限制
#define RIGHT_LIMINT_ANGLE -40.0f//角度限制

typedef struct RC_Ctl
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t s1;
		uint8_t s2;
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	struct
	{
		uint16_t v;
	}key;

}_RC_Ctl;

typedef struct canMessage{
	CanTxMsg canTx;
	CanRxMsg canRx;
}_canMessage;

typedef struct {
		uint32_t lasttime;
		uint32_t now;
		float count;
		float out;
		int isSameKey;
		uint32_t clock_cnt;
}_RampTime;

typedef struct{
	_RampTime WS;
	_RampTime AD;
	int lastKey;
	uint32_t clock_cnt;
}_moveKey;

typedef struct{
	const int16_t normal_FB;
	const int16_t normal_LR;
	int16_t Fb;
	int16_t Lr;
}_speed;

int8_t canTrans(uint8_t, int8_t , _canMessage* , int16_t* );
void transferType(int8_t , _canMessage* , int16_t* );

extern _canMessage canM;
extern _canMessage canM1;

int8_t readRemote(unsigned char* );
int8_t commuiModeChange(const _RC_Ctl* , _chassis* );
int8_t computerControl(const _RC_Ctl*,_chassis*);
int8_t remoteControl(const _RC_Ctl* , _chassis*);

int8_t can1Trans(uint8_t);

extern _RC_Ctl remote;
extern _moveKey key;
extern _speed speed;
float RampCal(_RampTime *RampT);

#endif
