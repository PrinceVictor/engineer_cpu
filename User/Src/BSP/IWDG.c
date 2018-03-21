#include "IWDG.h"


void MyDog_Config(uint8_t flag)
{
	if(flag == 1)
	{
		IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);//允许访问看门狗寄存器
		
		IWDG_SetPrescaler(IWDG_Prescaler_8);//设置分频
		
		IWDG_SetReload(20);//20
		
		IWDG_Enable();
		
		IWDG_ReloadCounter();//配置完就喂一次狗
	}
}

