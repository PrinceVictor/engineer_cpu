#include "main.h"
float Send_data[5];
uint32_t count =0;
int main(void)
{  
	uint16_t co_unt = 0;
	boot();
	while(1){
		imu(1);	
		
		if(co_unt >30){
			send_odm_msg(Send_data);
			co_unt = 0;
		}
		co_unt ++;
	}
	count++;
	
}
