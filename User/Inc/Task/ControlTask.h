#ifndef __CONTROLTASK_H__
#define __CONTROLTASK_H__

#include "MyFunc.h"
#include "ChassisTask.h"
#include "ComunicateTask.h"
#include "Holder.h"
#include "6050.h"

#define init 1
#define run	 2
#define landing 3
#define stop 0
#define reset -1

typedef struct{
	int8_t state;							// 1 for init, more details see as above
	int8_t manualOrauto;			// 1 for manual, 0 for auto 
	int8_t remoteOrkeyboard;  // 1 for remote, 0 for keyboard and mouse
}_sysState;

extern _sysState sys;
extern int8_t runControl(_sysState*);
void angle_update();
#endif 

