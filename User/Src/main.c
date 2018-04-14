#include "main.h"
float Send_data1[6];
float Send_data2[4];
uint32_t count =0;
int main(void)
{  
	uint16_t co_unt = 0;
	boot();
	while(1){
		imu(1);	
		
		if(co_unt >30){
			send_odm_msg1(Send_data1);
//			send_odm_msg2(Send_data2);
			co_unt = 0;
		}
		co_unt ++;
	}
	count++;
	
}
