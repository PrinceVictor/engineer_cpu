#include "Clock.h"
uint32_t clock_count =0;
void sysConfig(void){
	TIM_TimeBaseInitTypeDef tim;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	tim.TIM_Period = 0xFFFFFFFF;
	tim.TIM_Prescaler = 84 - 1;	 //1M µÄÊ±ÖÓ  
	tim.TIM_ClockDivision = TIM_CKD_DIV1;	
	tim.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_ARRPreloadConfig(TIM4, ENABLE);	
	TIM_TimeBaseInit(TIM4, &tim);
	TIM_Cmd(TIM4,ENABLE);	
}

void clockConfig(void)
{
	TIM_TimeBaseInitTypeDef tim;
	NVIC_InitTypeDef nvic;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE); 
	
  tim.TIM_Period = 100-1; 	
	tim.TIM_Prescaler=840-1;  
	tim.TIM_CounterMode=TIM_CounterMode_Up; 
	tim.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM12,&tim);
	TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE); 
	TIM_Cmd(TIM12,ENABLE); 
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
	nvic.NVIC_IRQChannel=TIM8_BRK_TIM12_IRQn; 
	nvic.NVIC_IRQChannelPreemptionPriority=0; 
	nvic.NVIC_IRQChannelSubPriority=1; 
	nvic.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&nvic);
	
}
//

void TIM8_BRK_TIM12_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM12,TIM_IT_Update)==SET)
	{
		TIM_ClearITPendingBit(TIM12,TIM_IT_Update);
    TIM_ClearFlag(TIM12, TIM_FLAG_Update);
		
		runControl();
		
		clock_count++;
}
}

uint32_t Get_Time_Micros(void)
{
	return TIM4->CNT;
}

