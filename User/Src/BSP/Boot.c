#include "Boot.h"


void boot(void){
	
	delay_init(168);
	refereeConfig();
	can2Config();
	delay_ms(1000);
//	PMM_Init();
//	Steering_Config();
	I2C_INIT();
	InitMPU6050();		
	Gyro_OFFEST();
	sysConfig();
	
	remoteConfig();
	mainfoldConfig();
	clockConfig();
}

