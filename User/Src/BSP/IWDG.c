#include "IWDG.h"


void MyDog_Config(uint8_t flag)
{
	if(flag == 1)
	{
		IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);//������ʿ��Ź��Ĵ���
		
		IWDG_SetPrescaler(IWDG_Prescaler_8);//���÷�Ƶ
		
		IWDG_SetReload(20);//20
		
		IWDG_Enable();
		
		IWDG_ReloadCounter();//�������ιһ�ι�
	}
}

