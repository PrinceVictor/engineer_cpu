#include "ComunicateTask.h"
_moveKey key = {0};
_speed speed = {
	200,
	150,
	0,
	0
};

_RC_Ctl remote = {0};
_canMessage canM = {0};
_canMessage canM1 = {0};
uint8_t can1transmit =0;
uint8_t can1Recieve = 0;


int8_t commuiModeChange(int8_t* flag, _RC_Ctl* data, _chassis* chassis){
	if( data->rc.s1 == 2)	*flag = 0;
	else *flag = 1;
	if(data->rc.s1 == 1) return 0;
	if(*flag) remoteControl(data , chassis); 
	else computerControl(data , chassis);	
	key.clock_cnt ++;
	return 2;
}

int8_t readRemote(_RC_Ctl* data, unsigned char * buffer){

	data->rc.ch0 = (buffer[0]| (buffer[1] << 8)) & 0x07ff; //!< Channel 0
	data->rc.ch1 = ((buffer[1] >> 3) | (buffer[2] << 5)) & 0x07ff; //!< Channel 1
	data->rc.ch2 = ((buffer[2] >> 6) | (buffer[3] << 2) |(buffer[4] << 10)) & 0x07ff; //!< Channel 2
	data->rc.ch3 = ((buffer[4] >> 1) | (buffer[5] << 7)) & 0x07ff; //!< Channel 3
	data->rc.s1 = ((buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
	data->rc.s2 = ((buffer[5] >> 4)& 0x0003); //!< Switch right
	data->mouse.x = -(buffer[6] | (buffer[7] << 8)); //!< Mouse X axis
	data->mouse.y = buffer[8] | (buffer[9] << 8); //!< Mouse Y axis
	data->mouse.z = buffer[10] | (buffer[11] << 8); //!< Mouse Z axis
	data->mouse.press_l = buffer[12]; //!< Mouse Left Is Press 
	data->mouse.press_r = buffer[13]; //!< Mouse Right Is Press 
	data->key.v = buffer[14] | (buffer[15] << 8); //!< KeyBoard value
	return 1;
}

float RampCal(_RampTime *RampT)
{
	if(!RampT->isSameKey )
	{
		RampT->lasttime = key.clock_cnt;
	}
	RampT->count = (float)(key.clock_cnt - RampT->lasttime) / 100;//按键持续的时间
	if(RampT->count > 5) RampT->out = 1;
	else RampT->out = 1 - (float)exp(-RampT->count) ;
	return RampT->out;
}

int8_t remoteControl(_RC_Ctl* data, _chassis* chassis){
	static int count = 0;
	
	if(abs( data->rc.ch1-1024)> 100){
			key.WS.isSameKey = 1;
		if(( data->rc.ch1-1024)> 100){
		speed.Fb = speed.normal_FB*RampCal(&key.WS);
		}
		else if(( data->rc.ch1-1024)< -100){
		speed.Fb = -speed.normal_FB*RampCal(&key.WS);
		}
	 }
	else{
			key.WS.isSameKey = 0;
			speed.Fb = 0; 
			RampCal(&key.WS);
	 }
	if(abs( data->rc.ch0-1024)> 100){
			key.AD.isSameKey = 1;
		if(( data->rc.ch0-1024)> 100){
		speed.Lr = speed.normal_LR*RampCal(&key.AD);
		}
		else if(( data->rc.ch0-1024)< -100){
		speed.Lr = -speed.normal_LR*RampCal(&key.AD);
		}
	 }
	else{
		 key.AD.isSameKey = 0;
		 speed.Lr = 0;
		 RampCal(&key.AD);
	 }
	
	chassis->Fb = speed.Fb;
	chassis->Lr = speed.Lr;
	if(abs( 1024 - data->rc.ch2 )< 50)  data->rc.ch2 = 1024;
	chassis->yaw.temp = \
		( 1024 - data->rc.ch2 ) * 0.0005f;//测试视觉时，注释此举
	chassis->yaw.temp = amplitudeLimiting(1 , chassis->yaw.temp , 60);
	
	chassis->yaw.target = chassis->yaw.target - chassis->yaw.temp ;
	
	 if(count > 5){
		if(chassis->yaw.target != chassis->yaw.last_target) chassis->yaw.target_changeMode = 1;
		 else chassis-> yaw.target_changeMode =0;
		count = 0;
}
	count++;
	return 1;
}


int8_t computerControl(_RC_Ctl* data, _chassis* chassis){

	if(data->key.v & 0x01)//  w
	{
		if(key.lastKey & 0x01){
			key.WS.isSameKey =1;
		}
		else {
			key.WS.isSameKey =0;
		}
		speed.Fb = speed.normal_FB*RampCal(&key.WS);
	}
	else if(data->key.v & 0x02) //   s
	{
		if(key.lastKey & 0x02){
			key.WS.isSameKey =1;
		}
		else {
			key.WS.isSameKey =0;
		}
		speed.Fb = -speed.normal_FB*RampCal(&key.WS);
	}
	else
	{
		speed.Fb =0;
		key.WS.isSameKey =0;
	}
	
	if(data->key.v & 0x04)  //    d
	{
		if(key.lastKey & 0x04){
			key.AD.isSameKey =1;
		}
		else {
			key.AD.isSameKey =0;
		}
		speed.Lr = speed.normal_LR*RampCal(&key.AD);//缓慢加速的过程，按键时间超过500ms则原数输出
	}
	else if(data->key.v & 0x08)    //   a
	{
		if(key.lastKey & 0x08){
			key.AD.isSameKey =1;
		}
		else {
			key.AD.isSameKey =0;
		}
		speed.Lr = -speed.normal_LR*RampCal(&key.AD);
	}
	else
	{
		speed.Lr=0;
		key.AD.isSameKey =0;
	}
	
	key.lastKey =data->key.v;

	chassis->Fb = speed.Fb;
	chassis->Lr = speed.Lr;
					
		/*yaw 向左*/
		if( data->mouse.x > 0 )
			{
				if(chassis->yaw.angle < LEFT_LIMINT_ANGLE)
				{
					chassis->yaw.temp = chassis->yaw.target + ( data->mouse.x ) * YAW_SENSITY ;
				}
			}
			else if( data->mouse.x < 0 )
			{
				if(chassis->yaw.angle > RIGHT_LIMINT_ANGLE)
				{
					chassis->yaw.temp = chassis->yaw.target + ( data->mouse.x ) * YAW_SENSITY ;
				}
			}
			chassis->yaw.target = chassis->yaw.temp;		
	return 1;
}

void transferType(int8_t mode, _canMessage* message, int16_t* data){
	int i=0;
	switch(mode){
		case 0: 
			for(i=0; i<4; i++){
				message->canTx.StdId = 0x200;
				message->canTx.IDE=CAN_ID_STD;					
				message->canTx.RTR=CAN_RTR_DATA;				 
				message->canTx.DLC=8;	
				message->canTx.Data[0+i*2] = (uint8_t)(0);
				message->canTx.Data[1+i*2] = (uint8_t)(0);
			}
			break;
		case 1:{
			message->canTx.StdId = 0x200;
			message->canTx.IDE=CAN_ID_STD;					
			message->canTx.RTR=CAN_RTR_DATA;				 
			message->canTx.DLC=8;							
			for(i=0; i<4; i++){
				message->canTx.Data[0+i*2] = (uint8_t)(*(data+i) >> 8);
				message->canTx.Data[1+i*2] = (uint8_t)(*(data+i));
			}
			break;}
		case 2:{
			message->canTx.StdId = 0x1ff;
			message->canTx.IDE=CAN_ID_STD;					
			message->canTx.RTR=CAN_RTR_DATA;				 
			message->canTx.DLC=8;							
			for(i=0; i<2; i++){
				message->canTx.Data[0+i*2] = (uint8_t)(*(data+i) >> 8);
				message->canTx.Data[1+i*2] = (uint8_t)(*(data+i));
			}
			break;}
		default:  break;}	
}


int8_t canTrans(uint8_t flag, 
								int8_t mode,
								_canMessage* message, 
								int16_t* data)
{	
	uint8_t TransmitMailbox;//发送信箱号
	int16_t t;
	if(!flag) return 0;
		else{
		transferType(mode, message, data);
		TransmitMailbox = CAN_Transmit(CAN2, &message->canTx);
		t=0;
		while((CAN_TransmitStatus(CAN2,TransmitMailbox)!=CANTXOK)&&(t<0xff))
		{
			t++;
		}
		return 1;
}
}

int8_t can1Trans(uint8_t flag)
{	
	uint8_t TransmitMailbox;//发送信箱号
	int16_t t;
	canM1.canTx.StdId = 0x02;
	canM1.canTx.Data[0]  = can1transmit;
	if(!flag) return 0;
		else{
		TransmitMailbox = CAN_Transmit(CAN2, &canM1.canTx );
		t=0;
		while((CAN_TransmitStatus(CAN2,TransmitMailbox)!=CANTXOK)&&(t<0xff))
		{
			t++;
		}
		return 1;
}
}

