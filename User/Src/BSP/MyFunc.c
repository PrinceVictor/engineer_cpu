#include "MyFunc.h"

//最小值比较
float my_min( float a , float b )
{
	if(a>b)	return b;
	else	return a;
}

//最大值比较
float my_max( float a , float b )
{
	if(a>b)	return a;
	else	return b;
}

//限幅输出
float amplitudeLimiting(uint8_t flag , float input , float limit)
{
	if(!flag)	return input;
	if( input > limit)	return limit;
	else if( input < -limit )	return-limit;
	else	return input;
}
	

//分段PID
float Subsection_PID(uint8_t flag , float raw , uint8_t num , float * K , float * Scale)
{
	uint8_t cnt = 0;
	
	if( flag == 0 )
	{
		return raw;
	}
	//
	for( cnt = 0;cnt<num-1;cnt++ )
	{
		if( ( abs(raw) <= (*(Scale + cnt + 1)) )&&( abs(raw) > (*(Scale + cnt)) ) )
		{
			break;
		}
	}
	
	
	return ( (*(K+cnt)) * raw );
 
}


//之前范围都是线性区，直接用线性公式   没有测试
float Subsection_PID_v2(uint8_t flag,float Delta,float scale1,float scale2,float k1,float k2)
{
  float delta_k,scale_delta,delta_scale;
  if(flag==0)
  {
    return Delta;
  }
  if(( abs(Delta) >= scale1 ) && ( abs(Delta) <= scale2 ))
  {
    delta_scale = abs(scale2-scale1);
    scale_delta = ( Delta-scale1 ) / delta_scale;
    delta_k = abs(k2-k1);
    return (Delta*( k1 + scale_delta*delta_k ));
  }else
  {
    return k2 * Delta;
  }
}
