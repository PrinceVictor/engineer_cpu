#include "Boot.h"
void IO_init(){
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}
void imu_reset(){
	uint8_t TransmitMailbox;//���������
	int16_t t;
	CanTxMsg canTx;
	canTx.StdId = 0x404;
	canTx.IDE=CAN_ID_STD;					
	canTx.RTR=CAN_RTR_DATA;				 
	canTx.DLC=8;
	canTx.Data[0] = (uint8_t) 0x00;			
	canTx.Data[1] = (uint8_t) 0x01;
	canTx.Data[2] = (uint8_t) 0x02;
	canTx.Data[3] = (uint8_t) 0x03;
	canTx.Data[4] = (uint8_t) 0x04;
	canTx.Data[5] = (uint8_t) 0x05;
	canTx.Data[6] = (uint8_t) 0x06;
	canTx.Data[7] = (uint8_t) 0x07;
	TransmitMailbox = CAN_Transmit(CAN2,&canTx);
	t=0;
	while((CAN_TransmitStatus(CAN2,TransmitMailbox)!=CANTXOK)&&(t<0xff))
	{
		t++;
	}
}

void steering_yaw_pitch(void){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	NVIC_InitTypeDef  NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM14ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORTFʱ��	
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3); //GPIOF9����Ϊ��ʱ��14
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM3);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_8;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //��ʼ��PF9
	 
	
	TIM_TimeBaseStructure.TIM_Prescaler= 8400-1	;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_Period=200-1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//��ʼ����ʱ��14
	
	//��ʼ��TIM14 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�
	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM14
}


void boot(void){
	
	delay_init(168);
	refereeConfig();
	can2Config();
	steering_yaw_pitch();
//	can1Config();
	delay_ms(1000);

//	I2C_INIT();
//	InitMPU6050();		
//	Gyro_OFFEST();
	imu_reset();
	sysConfig();
	
	remoteConfig();
	nucConfig();
	clockConfig();
	delay_ms(1000);
}

