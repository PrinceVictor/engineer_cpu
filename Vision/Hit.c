 #include "Hit.h"

#define ENABLE 1
#define DISABLE 0

struct RC_Ctl_key RC_Key;

uint8_t HitMode=0,TestSDSU=0,FetchMode=0,FetchFinish=0,CanChangeFlag=0,CanFetchFlag=0,ManualMode=0;
uint32_t LastTarget = 0;
uint8_t Fetch_cnt=0,CanShootFlag=0;

uint8_t Hit_rev[5] ={0};//妙算接收缓冲区
uint8_t Hit[5] ={0};

static void SelectMode(uint8_t mode);
static void HitSDSU(uint8_t mode,uint8_t point);

static void HitSDSU_Pro(uint8_t flag);
static void HitSDSU_Key_scan(uint8_t flag);

float TargetTable[9][2]=
{
{9.18,-9.05396},//1号
{0,-9.05396},
{-8.5,-9.05396},
{9.18,-4.8848},
{0,-4.8848},
{-8.2,-4.8848},
{9.3075,-0.70244},
{0,-0.70244},
{-8.67,-0.70244}
};


void CalculateKey(void)
{
	RC_Key.key_A  = 0x0004;
	RC_Key.key_W  = 0x0001;
	RC_Key.key_S  = 0x0002;
	RC_Key.key_Q  = 0x0040;//KEYI
	RC_Key.key_E  = 0x0080;//KEYI
	RC_Key.key_R  = 0x0100;
	RC_Key.key_D  = 0x0008;
	RC_Key.key_F  = 0x0200;
	RC_Key.key_G  = 0x0400;
	RC_Key.key_Z  = 0x0800;
	RC_Key.key_X  = 0x1000;
	RC_Key.key_C  = 0x2000;
	RC_Key.key_V  = 0x4000;
	RC_Key.key_B  = 0x8000;
	
	RC_Key.key_Shift  = 0x0010;//KEYI
	RC_Key.key_Ctrl  = 0x0020;//KEYI
	
}


void Hit_flag_init(void)
{
	HitMode =0;//九宫格模式
	FetchMode = 0;//采集模式
}


void Hit_Task(uint8_t flag)
{
	if( flag == 0 )
	{
		return;
	}
	HitSDSU_Key_scan(1);
	
	HitSDSU_Pro(1);
}


static void HitSDSU_Key_scan(uint8_t flag)
{

	if( flag == 0 )
	{
		return;
	}
	
	//采集模式按键处理
		if(CanChangeFlag)
		{
			if(RC_Ctl.key.v == RC_Key.key_E)
			{
				CanFetchFlag = 1;
				CanChangeFlag = 0;
			}
		}
		
		if((FetchMode == 1)&&(RC_Ctl.key.v == 0))
		{
			CanChangeFlag = 1;
		}
//********************************************		
		
		//采集模式 Q开启 Ctrl+Q关闭
	if((RC_Ctl.key.v == RC_Key.key_Q)&& (!ManualMode))
	{
		FetchMode = 1;
		HitMode = 0;
		TestSDSU = 0;
	}
	if((RC_Ctl.key.v & RC_Key.key_Ctrl)&&(RC_Ctl.key.v & RC_Key.key_Q))
	{
		FetchMode = 0;
		HitMode = 0;
		TestSDSU = 0;
		Fetch_cnt =0;
		ManualMode = 0;
/********************************************/		
		
		
		Detect_Data.Ctrl_Quit_Flag = 1;
//		yaw_Hold_Info.angle_target = 0;
//		Pitch_Hold_Info.angle_target = 0;
//		ComeToZero(1);
//		Wheel_Speed_control(0);
//		yaw_Hold_Info.angle = 0;


/********************************************/
	}
	if((RC_Ctl.key.v & RC_Key.key_B)){
		
		ManualMode = 1;

}
	if((RC_Ctl.key.v & RC_Key.key_Ctrl)&&(RC_Ctl.key.v & RC_Key.key_B)){
		
		ManualMode = 0;

}
		
}
int jjjjwww = 0;
 static void HitSDSU_Pro(uint8_t flag)
{
//	static uint32_t jjjjwww = 0;
	
	if( flag == 0 )
	{
		return;
	}
	
	if(!FetchMode)
	{
		if(!HitMode)
		{	
			yaw_Hold_Info.HitOrNot = 0;
			//正常运行模式
			PAout(4) = 1;	

			Holder_Control(ENABLE , &Pitch_para, &Pitch_Hold_Info);//pitch
			Holder_Control(ENABLE , &Yaw_para  , &yaw_Hold_Info);//YAW
			
			//Holder_Motor_output(HOLDER_OUT);
			Holder_Motor_output(1);
			
			Chassis_Control(1);
			Chassis_Remote_Dispack(1);
			
			Wheel_Speed_control(1);	
		}
		else
		{				
			if(ManualMode){
				PAout(4) = 1;
				
				Manual_Hit();

				HitSDSU(jjjjwww,jjjjwww);		
			}	
		else {			
				PAout(4) = 0;//关闭激光笔
				if((Detect_Data.Hit_Flag == 1)&&(Hit[3] != 0xFF)){
				HitSDSU(Hit[3],Hit[1]);		
				}

			}
		}
	}
	else
	{
		//采集模式
		
		PAout(4) = 1;					
		
		Holder_Control(ENABLE , &Pitch_para, &Pitch_Hold_Info);//pitch
		Holder_Control(ENABLE , &Yaw_para  , &yaw_Hold_Info);//YAW
		Holder_Motor_output(1);
		
		Wheel_Speed_control(0);
		
		SelectMode(3);				
	}
}

static void HitSDSU(uint8_t mode,uint8_t point)
{
	
	uint32_t Target = mode;
	
	yaw_Hold_Info.HitOrNot = 1;
	Pitch_Hold_Info.angle_target = TargetTable[point-1][1];
	yaw_Hold_Info.angle_target = TargetTable[point-1][0];
	Holder_Control(ENABLE ,&Pitch_para, &Pitch_Hold_Info);//pitch
	Holder_Control(ENABLE, &Yaw_para  , &yaw_Hold_Info);//YAW
	
	Holder_Motor_output(1);
		
  if(LastTarget == Target)
  {
		

  }else if(Shoot_Info.load_command  == 1)
  {
//		Detect_Data.Hit_cnt++;
    
//		if(Detect_Data.Hit_cnt > 1){
			Load_Motor_position_plus(LaserAndPrep[TANK_SERIAL_NUMBER-1][4]);
		}
	

	//	Detect_Data.Hit_Last_cnt = Detect_Data.Hit_cnt;
 // }
  
	LastTarget = Target;
 
}



static void SelectMode(uint8_t mode)
{
	float Pix[9][2];
	float ReturnRes[2];
	static uint8_t j=1;
//	float dev_Yaw,dev_Pitch;
	switch(mode){
	case 1:{break;}
	case 2:{
				Pix[0][0] = yaw_Hold_Info.can_angle;
				Pix[0][1] = Pitch_Hold_Info.can_angle;

				
				Pix[8][0] = yaw_Hold_Info.can_angle;
				Pix[8][1] = Pitch_Hold_Info.can_angle;
				
				TwoPointCal(abs(Pix[0][0]),abs(Pix[8][0]),ReturnRes);
				TargetTable[0][0] = Pix[0][0];
				TargetTable[1][0] = TargetTable[0][0]+ReturnRes[0];
				TargetTable[2][0] = TargetTable[1][0]+ReturnRes[1];
				TargetTable[0][1] = Pix[0][1];
				TargetTable[1][1] = Pix[0][1];
				TargetTable[2][1] = Pix[0][1];
				
				TwoPointCal(abs(Pix[0][1]),abs(Pix[8][1]),ReturnRes);
								
	break;}
	case 3:{
				if(CanFetchFlag)
				{	
					if(j==1)
					{
						TargetTable[0][0] = yaw_Hold_Info.can_angle;
						TargetTable[0][1] =Pitch_Hold_Info.angle;					
					}else if(j==2)
					{	
						TargetTable[4][0] = yaw_Hold_Info.can_angle;
						TargetTable[4][1] = Pitch_Hold_Info.angle;					
					}
					else if(j == 3)
					{
						TargetTable[8][0] = yaw_Hold_Info.can_angle;
						TargetTable[8][1] = Pitch_Hold_Info.angle;
						FetchFinish = 1;
					}

					j++;
					CanFetchFlag = 0;
				}
				if(FetchFinish)
				{			
					TargetTable[1][0] = TargetTable[4][0];
					TargetTable[1][1] = TargetTable[0][1];
					
					TargetTable[2][0] = TargetTable[8][0];
					TargetTable[2][1] = TargetTable[0][1];
					
					TargetTable[3][0] = TargetTable[0][0];
					TargetTable[3][1] = TargetTable[4][1];
					
					TargetTable[5][0] = TargetTable[8][0];
					TargetTable[5][1] = TargetTable[4][1];
					
					TargetTable[6][0] = TargetTable[0][0];
					TargetTable[6][1] = TargetTable[8][1];
					
					TargetTable[7][0] = TargetTable[4][0];
					TargetTable[7][1] = TargetTable[8][1];
					
					FetchMode = 0;
					HitMode = 1;
					TestSDSU = 0;
					FetchFinish =0;
					j=1;
					return;
					}
//			if(j>3)
//				{j=1;}	
	break;}
	case 9:{
					if(CanFetchFlag)
					{
						TargetTable[Fetch_cnt][0] = Chassis_Control_Info.Chassis_angle;
						TargetTable[Fetch_cnt][1] = Pitch_Hold_Info.can_angle;
						Fetch_cnt++;
						CanFetchFlag = 0;
					}
			if(Fetch_cnt>8)
				{	
					Fetch_cnt=0;
					FetchMode = 0;
					HitMode = 1;
					TestSDSU = 0;
				}	
					break;
				}
	}
}

static void Manual_Hit(){
	switch(RC_Ctl.key.v){
		case 0x0040 :{ jjjjwww = 1;
			break;
}
		case 0x0001 :{ jjjjwww = 2;
			break;
}
		case 0x0080 :{ jjjjwww = 3;
			break;
}
		case 0x0004 :{ jjjjwww = 4;
			break;
}
		case 0x0002 :{ jjjjwww = 5;
			break;
}
		case 0x0008 :{ jjjjwww = 6;
			break;
}
		case 0x0800 :{ jjjjwww = 7;
			break;
}
		case 0x1000 :{ jjjjwww = 8;
			break;
}
		case 0x2000 :{ jjjjwww = 9;
			break;
}

		default : break;
}
}



