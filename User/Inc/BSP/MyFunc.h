#ifndef __MYFUNC_H__
#define __MYFUNC_H__

#include "stm32f4xx.h"
#include "sys.h" 
#include "delay.h"
#include <stdio.h>
#include <stdint.h>
#include "math.h"

//����ֵ
#define abs(x) ((x)>0? (x):(-(x)))

//�����Сֵ
float amplitudeLimiting(uint8_t, float, float);

//2016 �ֶ�PID
float Subsection_PID(uint8_t, float, uint8_t, float*, float*);

//2017
float Subsection_PID_v2(uint8_t ,float ,float ,float ,float ,float );

//��Сֵ
float my_min( float , float );
//���ֵ
float my_max( float , float );



#endif

