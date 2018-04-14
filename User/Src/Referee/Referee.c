#include "referee.h"
/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/
uint8_t recieveData[14]={0};
_lidar_message lidar;

uint8_t meta_data[BSP_USART3_DMA_RX_BUF_LEN];

_JUDGMENT_01_DATA Judgment_01_data;
_JUDGMENT_02_DATA Judgment_02_data;
_JUDGMENT_03_DATA Judgment_03_data;
_JUDGMENT_04_DATA Judgment_04_data;

//WAU:where are u?
float temp_V,temp_A,temp_W,temp_J,temp_ss,temp_sf,temp_bs,temp_bf;

uint8_t Tx_Buf[TX_LEN];
uint8_t re_data[TX_LEN];
uint8_t Flag_Uart_Busy=0;

void refereeConfig(void){


    USART_InitTypeDef usart2;
		GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  NVIC_InitStructure;
		DMA_InitTypeDef   DMA_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
		
		gpio.GPIO_Mode  = GPIO_Mode_AF;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_PuPd  = GPIO_PuPd_UP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
		gpio.GPIO_Pin   = GPIO_Pin_2;  //USART2_TX
		GPIO_Init(GPIOA, &gpio);
						
		gpio.GPIO_Pin   = GPIO_Pin_3;   //USART2_RX
		GPIO_Init(GPIOA, &gpio);

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);        
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
		
		usart2.USART_BaudRate = 115200;
		usart2.USART_WordLength = USART_WordLength_8b;
		usart2.USART_StopBits = USART_StopBits_1;
		usart2.USART_Parity = USART_Parity_No;
		usart2.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
		usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART2,&usart2);

    USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
    
		USART_Cmd(USART2,ENABLE);
				
	//	memset(meta_data, 0xff, sizeof(meta_data));
			
		DMA_DeInit(DMA1_Stream5);
    DMA_InitStructure.DMA_Channel= DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)meta_data;//(uint32_t)EnemyData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = BSP_USART3_DMA_RX_BUF_LEN;   //BSP_USART3_DMA_RX_BUF_LEN;//100;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5,&DMA_InitStructure);
    
    DMA_DeInit(DMA1_Stream6);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4; 
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Tx_Buf;//send buffer
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = TX_LEN; //8
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);
    
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;  //TIM3中断
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级3级
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
		NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
    
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

    USART_ITConfig( USART2 , USART_IT_IDLE , ENABLE );//空闲中断
    DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);//DMA发送中断
    
    DMA_Cmd(DMA1_Stream5,ENABLE);//RX		 
    DMA_Cmd(DMA1_Stream6,ENABLE);//TX
    
		USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
    
		USART_Cmd(USART2,ENABLE);

}


void mainfoldConfig(void)
{
    USART_InitTypeDef usart3;
		GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  NVIC_InitStructure;
		DMA_InitTypeDef   DMA_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		
		gpio.GPIO_Mode  = GPIO_Mode_AF;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_PuPd  = GPIO_PuPd_UP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
		gpio.GPIO_Pin   = GPIO_Pin_10;
		GPIO_Init(GPIOB, &gpio);
						
		gpio.GPIO_Pin   = GPIO_Pin_11;
		GPIO_Init(GPIOB, &gpio);

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);        
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
		
		usart3.USART_BaudRate = 115200;
		usart3.USART_WordLength = USART_WordLength_8b;
		usart3.USART_StopBits = USART_StopBits_1;
		usart3.USART_Parity = USART_Parity_No;
		usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
		usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3,&usart3);

    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
    
		USART_Cmd(USART3,ENABLE);
			
		DMA_DeInit(DMA1_Stream1);
    DMA_InitStructure.DMA_Channel= DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)recieveData;//(uint32_t)EnemyData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = 14;   //BSP_USART3_DMA_RX_BUF_LEN;//100;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1,&DMA_InitStructure);
    
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  //TIM3中断
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
		NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
    
    USART_ITConfig( USART3 , USART_IT_IDLE , ENABLE );//空闲中断
    
    DMA_Cmd(DMA1_Stream1,ENABLE);//RX		 
    
		USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
    
		USART_Cmd(USART3,ENABLE);

}
void DMA1_Stream6_IRQHandler(void)
{
  if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET){
  DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
  DMA_Cmd(DMA1_Stream6,DISABLE);
  Flag_Uart_Busy = 0;
  }
}
void New_Send_Data(uint8_t *data,uint16_t size)
{
  while(Flag_Uart_Busy);
  
  Flag_Uart_Busy = 1;
  
  memcpy(Tx_Buf,data,size); 
  
  DMA_SetCurrDataCounter(DMA1_Stream6,size);
  
  DMA_Cmd(DMA1_Stream6,ENABLE); 
}

void send_odm_msg1(float * data)
{ 
	int i=0;
	//PID_out;
  uint8_t uart2_send_buff[32]; 
	uint8_t sum=0;
	uart2_send_buff[0] = 0xAA;
	uart2_send_buff[1] = 0xAA;
	uart2_send_buff[2] = 0xF1;
	uart2_send_buff[3] = 24;
	for(i=0;i<6;i++) 
	{
		uart2_send_buff[i*4+0+4] = BYTE3(*(data+i));
		uart2_send_buff[i*4+1+4] = BYTE2(*(data+i));
		uart2_send_buff[i*4+2+4] = BYTE1(*(data+i));
		uart2_send_buff[i*4+3+4] = BYTE0(*(data+i));
	}
	for(i = 0; i<28; i++)
		{
			sum +=uart2_send_buff[i];
		}
		uart2_send_buff[28] = sum;	

    New_Send_Data(uart2_send_buff,29);
}

void send_check(uint16_t data)
{
  uint8_t uart2_send_buff[32]; 
	uint16_t sum=0,i;
	uart2_send_buff[0] = 0xAA;
	uart2_send_buff[1] = 0xAA;
	uart2_send_buff[2] = 0xEF;
	uart2_send_buff[3] = 2;
  uart2_send_buff[4] = 0x10;
  uart2_send_buff[5] = data;
  
  for(i=0;i<6;i++)
  {
    sum+=uart2_send_buff[i];
  }
  uart2_send_buff[6] = sum;
  New_Send_Data(uart2_send_buff,7);
}

int dara = 0,time_count = 0;

void USART3_IRQHandler(void)
{
 static uint32_t this_time_rx_len = 0;
  uint8_t i=0;
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		//clear the idle pending flag 
		(void)USART3->SR;
		(void)USART3->DR;
       
   if(DMA_GetCurrentMemoryTarget(DMA1_Stream1) == 0)
		{
			DMA_Cmd(DMA1_Stream1, DISABLE);
    
			this_time_rx_len = 14 - DMA_GetCurrDataCounter(DMA1_Stream1);
		
			DMA1_Stream1->NDTR = (uint16_t)14;     //relocate the dma memory pointer to the beginning position
			//DMA1_Stream1->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA1_Stream1, ENABLE);
			if(recieveData[0] ==  0xab){
			lidar.flag = recieveData [13];

			BYTE0(lidar.d1) = recieveData[0+1 + 0*4];
      BYTE1(lidar.d1) = recieveData[1+1 + 0*4];
      BYTE2(lidar.d1) = recieveData[2+1+ 0*4];
      BYTE3(lidar.d1) = recieveData[3+1+ 0*4];
				
			BYTE0(lidar.d2) = recieveData[0+1 + 1*4];
      BYTE1(lidar.d2) = recieveData[1+1 + 1*4];
      BYTE2(lidar.d2) = recieveData[2+1+ 1*4];
      BYTE3(lidar.d2) = recieveData[3+1+ 1*4];
				
			BYTE0(lidar.angle) = recieveData[0+1 + 2*4];
      BYTE1(lidar.angle) = recieveData[1+1 + 2*4];
      BYTE2(lidar.angle) = recieveData[2+1+ 2*4];
      BYTE3(lidar.angle) = recieveData[3+1+ 2*4];
}

}
}
}


void USART2_IRQHandler(void)
{
  static uint32_t this_time_rx_len = 0;
  uint8_t i=0;
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		//clear the idle pending flag 
		(void)USART2->SR;
 		(void)USART2->DR;
       
   if(DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0)
		{
			DMA_Cmd(DMA1_Stream5, DISABLE);
    
			this_time_rx_len = BSP_USART3_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream5);
			DMA1_Stream5->NDTR = (uint16_t)BSP_USART3_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
			//DMA1_Stream1->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA1_Stream5, ENABLE);
		
      //视具体协议而定
      
      //裁判系统消息
      if(meta_data[0]==0xA5)
      {
        switch(meta_data[5])
        {
          case 0x01:{
            
            //从600s减到0 已确定
          Judgment_01_data.left_time_S = meta_data[7]+(meta_data[8]<<8)+ \
                                        (meta_data[9]<<16)+(meta_data[10]<<24);
            
          Judgment_01_data.left_HP = meta_data[11] + (meta_data[12]<<8);
            
          BYTE0(temp_V) = meta_data[13];
          BYTE1(temp_V) = meta_data[14];
          BYTE2(temp_V) = meta_data[15];
          BYTE3(temp_V) = meta_data[16];
          
          Judgment_01_data.voltage_V = temp_V;
          
          BYTE0(temp_A) = meta_data[17];
          BYTE1(temp_A) = meta_data[18];
          BYTE2(temp_A) = meta_data[19];
          BYTE3(temp_A) = meta_data[20];
          
          Judgment_01_data.current_A = temp_A;
          
          Judgment_01_data.power_W = Judgment_01_data.voltage_V * Judgment_01_data.current_A;		

          BYTE0(temp_J) = meta_data[38];
          BYTE1(temp_J) = meta_data[39];
          BYTE2(temp_J) = meta_data[40];
          BYTE3(temp_J) = meta_data[41];
          
          Judgment_01_data.remainJ = temp_J;
          break;}
        //打击信息
      
						case 0x02:{
          
          Judgment_02_data.weak = (meta_data[7]&0xf0)>>4;
          Judgment_02_data.weakid = meta_data[7]&0x0f;
          Judgment_02_data.ValueChange = meta_data[8]+(meta_data[9]<<8); 

          break;}
        case 0x03:{
          
          BYTE0(temp_ss) = meta_data[7];
          BYTE1(temp_ss) = meta_data[8];
          BYTE2(temp_ss) = meta_data[9];
          BYTE3(temp_ss) = meta_data[10];
          
          Judgment_03_data.small_bullet_speed = temp_ss;
          
          BYTE0(temp_sf) = meta_data[11];
          BYTE1(temp_sf) = meta_data[12];
          BYTE2(temp_sf) = meta_data[13];
          BYTE3(temp_sf) = meta_data[14];
          
          Judgment_03_data.small_bulet_frequency = temp_sf;
          
          BYTE0(temp_bs) = meta_data[15];
          BYTE1(temp_bs) = meta_data[16];
          BYTE2(temp_bs) = meta_data[17];
          BYTE3(temp_bs) = meta_data[18];
          
          Judgment_03_data.big_bullet_speed = temp_bs;
          
          BYTE0(temp_bf) = meta_data[19];
          BYTE1(temp_bf) = meta_data[20];
          BYTE2(temp_bf) = meta_data[21];
          BYTE3(temp_bf) = meta_data[22];
          
          Judgment_03_data.big_bulet_frequency = temp_bf;
      
        break;}
				
				case 0x04:{
					          
					Judgment_04_data.Robot_Color = (meta_data[7]&0x01);
					
					
					Judgment_04_data.Red_Base_Robot_status = (meta_data[7]&0x04)>>2;
					Judgment_04_data.Blue_Base_Robot_status = (meta_data[7]&0x10)>>4;
					Judgment_04_data.ResourceIsland_status = (meta_data[7]&0xC0)>>6;				
					Judgment_04_data.RedAirPortSta = (meta_data[8]&0x07);
					Judgment_04_data.BlueAirPortSta = (meta_data[8]&0x70)>>4;				
					Judgment_04_data.No1PillarSta = (meta_data[9]&0x07);
					Judgment_04_data.No2PillarSta = (meta_data[9]&0x70)>>4;
					Judgment_04_data.No3PillarSta = (meta_data[10]&0x07);
					Judgment_04_data.No4PillarSta = (meta_data[10]&0x70)>>4;
					Judgment_04_data.No5PillarSta = (meta_data[11]&0x07);
					Judgment_04_data.No6PillarSta = (meta_data[11]&0x70)>>4;
					Judgment_04_data.RedBulletBoxSta = (meta_data[12]&0x1);
					Judgment_04_data.BlueBulletBoxSta = (meta_data[12]&0x10)>>4;
					Judgment_04_data.RedBulletAmount = (meta_data[13]&0xFF);
					Judgment_04_data.RedBulletAmount = (meta_data[14]&0xFF)<<8;
					Judgment_04_data.BlueBulletAmount = (meta_data[15]&0xFF);
					Judgment_04_data.BlueBulletAmount = (meta_data[16]&0xFF)<<8;
					Judgment_04_data.No0BigRuneSta = (meta_data[17]&0x07);
					Judgment_04_data.No1BigRuneSta = (meta_data[17]&0x70)>>4;
					Judgment_04_data.AddDefendPrecent = (meta_data[18]&0xFF);
}
      
       }//switch 
      }//协议解算部分       
     }
	}
		
}


void SendtoReferee(uint8_t * send_data)
{

	send_data[0] = 0xA5;	
	send_data[1] = 0x0C;
	send_data[2] = 0x00;
	send_data[3] = 0x04;

	Append_CRC8_Check_Sum(send_data,5);
	
	send_data[5] = 0x05;
	send_data[6] = 0x00;

	Append_CRC16_Check_Sum(send_data , 21);

}



//crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned
char ucCRC8)
{
unsigned char ucIndex;
while (dwLength--)
{
ucIndex = ucCRC8^(*pchMessage++);
ucCRC8 = CRC8_TAB[ucIndex];
}
return(ucCRC8);
}

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
unsigned char ucExpected = 0;
if ((pchMessage == 0) || (dwLength <= 2)) return 0;
ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
return ( ucExpected == pchMessage[dwLength-1] );
}


void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
unsigned char ucCRC = 0;
if ((pchMessage == 0) || (dwLength <= 2)) {
	return;
}
ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
pchMessage[dwLength-1] = ucCRC;
}

uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] =
{
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
uint8_t chData;
if (pchMessage == NULL)
{
return 0xFFFF;
}
while(dwLength--)
{
chData = *pchMessage++;
(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &
0x00ff];
}
return wCRC;
}

uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
uint16_t wExpected = 0;
if ((pchMessage == NULL) || (dwLength <= 2))
{
return 0;
}
wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==
pchMessage[dwLength - 1]);
}

void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
uint16_t wCRC = 0;
if ((pchMessage == NULL) || (dwLength <= 2))
{
return;
}
wCRC = Get_CRC16_Check_Sum ( (u8 *)pchMessage, dwLength-2, CRC_INIT );
pchMessage[dwLength-2] = (u8)(wCRC & 0x00ff);
pchMessage[dwLength-1] = (u8)((wCRC >> 8)& 0x00ff);
}



