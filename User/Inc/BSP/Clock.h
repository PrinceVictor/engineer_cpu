#ifndef _CLOCK_H
#define _CLOCK_H

#include "sys.h"
#include "MyFunc.h"
#include "ComunicateTask.h"
#include "ControlTask.h"

void sysConfig(void);
void clockConfig(void);
uint32_t Get_Time_Micros(void);

#endif
