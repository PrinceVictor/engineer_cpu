#include "Hit_calculation.h"
#include "math.h"
#include "stdio.h"


#define POINTtoPOINT 72


void TwoPointCal(float PixFirst,float PixLast,float *Result)
{
	float a,b,c,d,e,L1,Dp,A1,A2,average;
	
	uint8_t i;
	
	L1 = POINTtoPOINT/2.0f;
	
	Dp = PixLast - PixFirst;
	
	a = 90.0f - PixFirst;
	
	e = 90+PixFirst;
	
	b = (e * POINTtoPOINT)/Dp;
	
	c = b * cos(e) + POINTtoPOINT * cos(Dp);
	
	d = sqrt(c*c+L1*L1-2*c*L1*cos(e));
	
	A1 = acos((c*c+d*d-L1*L1)/(2*c*d));//1->中点
	
	A2 = acos((d*d+b*b-L1*L1)/(2*d*b));//中点<-9
	
	Result[0] = A1;
	
	Result[1] = A2;
		
}

