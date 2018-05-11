#include "main.h"
#include "Relay.h"
float Send_data1[6];
float Send_data2[4];
uint32_t count =0;

int main(void)
{  
	uint16_t co_unt = 0;
	boot();
	while(1){
		led_status = 1;
	//	imu(1);	
		if(co_unt >30){
//			Send_data1[0] = chassisPara.Rt;
//			Send_data1[1] = wheelInfo.feedback.Speed[1];
//			Send_data1[1] = chassisPara.Fb;
//			Send_data1[3] = wheelInfo.feedback.Speed[3];
//      
//			Send_data1[2] = chassisPara.Lr;
//			Send_data1[1] = wheelInfo.targetSpeed[1];
//			Send_data1[3] =wheelInfo.feedback.Speed[3];
//			Send_data1[3] = wheelInfo.targetSpeed[3];
//      Send_data1[4] = (float)wheelInfo.out[0];
//			Send_data1[5] =  (float)wheelInfo.out[2];
	      Send_data1[4] = chassisPara.yaw.angle_speed	;
				Send_data1[5] = chassisPara.yaw.angle;		
//			send_odm_msg2(&wheelInfo.info);	
//			if(relay_flag.status_flag )
//			send_odm_msg2(&wheelInfo.info);	
 			send_odm_msg1(Send_data1);
//			send_odm_msg2(Send_data2);
			co_unt = 0;
		}
		co_unt ++;
		count++;
	}

	
}
