#include "Communication.h"
unsigned char sbus_rx_buffer[18];


void can1Config(void){
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;				//����CAN_InitStructure��ʼ������Ĵ���
	CAN_FilterInitTypeDef  CAN_FilterInitStructure; //���ݹ�����CAN_FilterInitStructure��ʼ������Ĵ���
	NVIC_InitTypeDef  NVIC_InitStructure; //Ƕ���ж������������Ĵ���

	//ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

	//��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12

	//���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	
	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=DISABLE;	//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //ģʽ���� 
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=3;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
	
	//���ù�����
	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0 
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  //������������ʼ����ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //������λ��32λ 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;//��������ʶ�� �� 32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000; //��������ʶ�� ��
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��

	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);		

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}


void can2Config(void){
	
	CAN_InitTypeDef        can;
	CAN_FilterInitTypeDef  can_filter;
	GPIO_InitTypeDef       gpio;
	NVIC_InitTypeDef       nvic;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); 

	gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12 ;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &gpio);

	nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	CAN_DeInit(CAN2);
	CAN_StructInit(&can);

	can.CAN_TTCM = DISABLE;
	can.CAN_ABOM = DISABLE;    
	can.CAN_AWUM = DISABLE;    
	can.CAN_NART = DISABLE;    
	can.CAN_RFLM = DISABLE;    
	can.CAN_TXFP = DISABLE;     
	can.CAN_Mode = CAN_Mode_Normal; 
	can.CAN_SJW  = CAN_SJW_1tq;
	can.CAN_BS1 = CAN_BS1_9tq;
	can.CAN_BS2 = CAN_BS2_4tq;
	can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
	CAN_Init(CAN2, &can);
	
	can_filter.CAN_FilterNumber=14;
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
    
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
    //CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);

}

void remoteConfig(void){
	USART_InitTypeDef USART1_InitStructure;
	GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;
	DMA_InitTypeDef   dma;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7 ,GPIO_AF_USART1);
	
	gpio.GPIO_Pin = GPIO_Pin_7 ;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&gpio);
		
	USART_DeInit(USART1);
	USART1_InitStructure.USART_BaudRate = 100000;   //D-BUS 100K baudrate
	USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART1_InitStructure.USART_StopBits = USART_StopBits_1;
	USART1_InitStructure.USART_Parity = USART_Parity_Even;
	USART1_InitStructure.USART_Mode = USART_Mode_Rx;
	USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART1_InitStructure);
			
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	
	nvic.NVIC_IRQChannel = USART1_IRQn;                          
	nvic.NVIC_IRQChannelPreemptionPriority = 1;   //pre-emption priority 
	nvic.NVIC_IRQChannelSubPriority = 0;		    //subpriority 
	nvic.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&nvic);	
	
	DMA_DeInit(DMA2_Stream5);
	DMA_StructInit(&dma);
	dma.DMA_Channel= DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
	dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = 30;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5,&dma);
	
	DMA_Cmd(DMA2_Stream5,ENABLE);
	
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
	USART_Cmd(USART1, ENABLE);
}

#if 0
void Steering_Config(void)
{
	
		GPIO_InitTypeDef GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); 	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3); 
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;           
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
		GPIO_Init(GPIOB,&GPIO_InitStructure);    
		
		TIM_TimeBaseStructure.TIM_Prescaler=8400-1;  //?????
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //??????
		TIM_TimeBaseStructure.TIM_Period=200-1;   //??????
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
		
		TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = 0; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; 
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; 
 
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);  
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);     
   
    TIM_CtrlPWMOutputs(TIM3,ENABLE);  
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

}


/*******************************************************************************
* Function Name  : PMM_IO INIT
* Description    : ???? 
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
void PMM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE );
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

#endif


void USART1_IRQHandler(void)
{	
	static uint32_t this_time_rx_len = 0;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		//clear the idle pending flag 
		(void)USART1->SR;
		(void)USART1->DR;

		//Target is Memory0
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream5) == 0)
		{
			DMA_Cmd(DMA2_Stream5, DISABLE);
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream5);
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA2_Stream5, ENABLE);
      if(this_time_rx_len == 18){		
					readRemote( sbus_rx_buffer);
		}
}  
}
}

void CAN1_TX_IRQHandler(void)
{	
if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
  {
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);   
  }	 
	
}

void CAN1_RX0_IRQHandler(void)
{	
  if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_FIFO0);
		CAN_Receive(CAN1,CAN_FIFO0,&canM1.canRx);
		if(canM1.canRx.StdId == 0x01){
				can1Recieve = canM1.canRx.Data[0];
}
	}		 
	
}


void CAN2_TX_IRQHandler(void) //CAN TX
{
  if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
  {
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);   
  }
}

void CAN2_RX0_IRQHandler(void)
{
  if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_Receive(CAN2,CAN_FIFO0,&canM.canRx);
		if( canM.canRx.StdId == 0x206 )//Pitch
		{
	
			motor.pitch.fb_postion = canM.canRx.Data[0] * 256 + canM.canRx.Data[1];
			//RxMessage206 = RxMessage;
		}
		else if(  canM.canRx.StdId == 0x205 )//Yaw
		{
			motor.yaw.fb_postion = canM.canRx.Data[0] * 256 + canM.canRx.Data[1];
			//RxMessage205 = RxMessage;
		}
		
		else if( (canM.canRx.StdId >= 0x201)&&( canM.canRx.StdId <= 0x204 ) )
		{
			wheelInfo.feedback.Postion[canM.canRx.StdId - 0x201] = canM.canRx.Data[0]*256 +canM.canRx.Data[1];
			
			wheelInfo.feedback.Speed[canM.canRx.StdId - 0x201] = canM.canRx.Data[2]*256 +canM.canRx.Data[3];
		}
		else if(canM.canRx.StdId == 0x001){
			redlaser.verifed_code = canM.canRx.Data[0];
			redlaser.flag = canM.canRx.Data[1];
//			redlaser._1st = canM.canRx.Data[1] & 0x01;
//			redlaser._2nd = canM.canRx.Data[1] & 0x02;
//			redlaser._3rd = canM.canRx.Data[1] & 0x04;
//			redlaser._4th = canM.canRx.Data[1] & 0x08;
		}
		

   }
}


