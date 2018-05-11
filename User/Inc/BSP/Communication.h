#ifndef __COMMUNICATION_H_
#define __COMMUNICATION_H_

#include "MyFunc.h"
#include "ComunicateTask.h"
#include "ControlTask.h"
#include "Holder.h"

void can2Config(void);
void remoteConfig(void);
void can1Config(void);
void PMM_Init(void);
void Steering_Config(void);

#define BSP_USART1_DMA_RX_BUF_LEN	30u


//extern unsigned char sbus_rx_buffer[18];


#endif

