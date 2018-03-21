#include "Imu.h"

#define GYRO_GAP 30
#define K_ANGLESPEED_2_ANGLE 0.0000305f

_angle angle;
volatile uint32_t lastUpdate, now; // ≤…—˘÷‹∆⁄º∆ ˝ µ•Œª us
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
float Q = 0.05f, R = 1.5f;
static double KalmanFilter_x(const double ,double ,double );
static double KalmanFilter_y(const double ,double ,double );
static double KalmanFilter_z(const double ,double ,double );
static double KalmanFilter_speed(const double ,double ,double );

uint8_t mpu6050_error_flag = 0;

float Gyro_File_Buf[3][GYRO_FILTER_NUM];

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void imu(int8_t flag){
	readIMU(1);
	IMUupdate(sensor.gyro.radian.x,
						sensor.gyro.radian.y,
						sensor.gyro.radian.z,
						sensor.acc.averag.x,
						sensor.acc.averag.y,
						sensor.acc.averag.z);	
}

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
		int i =0;
    float norm;
//    float hx, hy, hz, bx, bz;
    float vx, vy, vz;//, wx, wy, wz;
    float ex, ey, ez,halfT;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;   

		
	  now = Get_Time_Micros();  //∂¡»° ±º‰ µ•Œª «us   
    if(now<lastUpdate)
    {
    //halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
    }
    else	
    {
       halfT =  ((float)(now - lastUpdate) / 2000000.0f);
    }
    lastUpdate = now;	//∏¸–¬ ±º‰

    //øÏÀŸ«Û∆Ω∑Ω∏˘À„∑®
    norm = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
//    //∞—º”º∆µƒ»˝Œ¨œÚ¡ø◊™≥…µ•ŒªœÚ¡ø°£
//    norm = invSqrt(mx*mx + my*my + mz*mz);          
//    mx = mx * norm;
//    my = my * norm;
//    mz = mz * norm; 
//    // compute reference direction of flux
//    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
//    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
//    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
//    bx = sqrt((hx*hx) + (hy*hy));
//    bz = hz; 
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
//    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
//    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
//    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy);// + (my*wz - mz*wy);
    ey = (az*vx - ax*vz);// + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx);// + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
			exInt = exInt + ex * Kii * halfT;
			eyInt = eyInt + ey * Kii * halfT;	
			ezInt = ezInt + ez * Kii * halfT;
			// ”√≤Êª˝ŒÛ≤Ó¿¥◊ˆPI–ﬁ’˝Õ”¬›¡„∆´
			gx = gx + Kp*ex + exInt;
			gy = gy + Kp*ey + eyInt;
			gz = gz + Kp*ez + ezInt;
    }
    // Àƒ‘™ ˝Œ¢∑÷∑Ω≥Ã
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

    // Àƒ‘™ ˝πÊ∑∂ªØ
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
		
		angle.yaw= -atan2(2 * q1 * q2 + 2 * q0* q3, -2 * q2*q2 - 2 * q3 * q3 + 1)*RtA; // yaw        -pi----pi
    angle.pitch= -asin(-2 * q1 * q3 + 2 * q0 * q2)*RtA; // pitch    -pi/2    --- pi/2 
    angle.roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* RtA; // roll       -pi-----pi 	
		
	}
/*∏¸–¬À´÷· Ω«ÀŸ∂» Ω«∂» can±‡¬Î∆˜–≈œ¢*/
void readIMU(uint8_t flag)
{
	float sumx,sumy,sumz;
		
	static uint8_t gyro_filter_cnt = 0;
		
	int i =0;
		
	if(flag == 0) 
			return;
	else
		{
		
		MPU6050_Read();
		sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) ;
		sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) ;
		sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);
		
		sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9])- sensor.gyro.quiet.x;
		sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11])- sensor.gyro.quiet.y;
		sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13])- sensor.gyro.quiet.z;
		
		Gyro_File_Buf[0][gyro_filter_cnt] = sensor.gyro.origin.x ;
		Gyro_File_Buf[1][gyro_filter_cnt] = sensor.gyro.origin.y ;
		Gyro_File_Buf[2][gyro_filter_cnt] = sensor.gyro.origin.z ;
			
			sumx = 0;
			sumy = 0;
			sumz = 0;
		for(i=0;i<GYRO_FILTER_NUM;i++)
		{
			sumx += Gyro_File_Buf[0][i];
			sumy += Gyro_File_Buf[1][i];
			sumz += Gyro_File_Buf[2][i];
		}

		
		gyro_filter_cnt = ( gyro_filter_cnt + 1 ) % GYRO_FILTER_NUM;
		
		sensor.gyro.radian.x  = sumx / (float)GYRO_FILTER_NUM * Gyro_Gr;    //radian speed  unit: pi/s
		sensor.gyro.radian.y  = sumy / (float)GYRO_FILTER_NUM * Gyro_Gr;
		sensor.gyro.radian.z  = sumz / (float)GYRO_FILTER_NUM * Gyro_Gr;
			
		sensor.acc.averag.x = KalmanFilter_x(sensor.acc.origin.x,KALMAN_Q,KALMAN_R);  // ACC X÷·ø®∂˚¬¸¬À≤®
		sensor.acc.averag.y = KalmanFilter_y(sensor.acc.origin.y,KALMAN_Q,KALMAN_R);  // ACC Y÷·ø®∂˚¬¸¬À≤®
		sensor.acc.averag.z = KalmanFilter_z(sensor.acc.origin.z,KALMAN_Q,KALMAN_R);  // ACC Z÷·ø®∂˚¬¸¬À≤
	
	}
}


static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=‘Î…˘
   kg=p_mid/(p_mid+R); //kgŒ™kalman filter£¨RŒ™‘Î…˘
   x_now=x_mid+kg*(ResrcData-x_mid);//π¿º∆≥ˆµƒ◊Ó”≈÷µ
                
   p_now=(1-kg)*p_mid;//◊Ó”≈÷µ∂‘”¶µƒcovariance       
   p_last = p_now; //∏¸–¬covariance÷µ
   x_last = x_now; //∏¸–¬œµÕ≥◊¥Ã¨÷µ
   return x_now;                
 }

static double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=‘Î…˘
   kg=p_mid/(p_mid+R); //kgŒ™kalman filter£¨RŒ™‘Î…˘
   x_now=x_mid+kg*(ResrcData-x_mid);//π¿º∆≥ˆµƒ◊Ó”≈÷µ
                
   p_now=(1-kg)*p_mid;//◊Ó”≈÷µ∂‘”¶µƒcovariance       
   p_last = p_now; //∏¸–¬covariance÷µ
   x_last = x_now; //∏¸–¬œµÕ≥◊¥Ã¨÷µ
   return x_now;                
 }

static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=‘Î…˘
   kg=p_mid/(p_mid+R); //kgŒ™kalman filter£¨RŒ™‘Î…˘
   x_now=x_mid+kg*(ResrcData-x_mid);//π¿º∆≥ˆµƒ◊Ó”≈÷µ
                
   p_now=(1-kg)*p_mid;//◊Ó”≈÷µ∂‘”¶µƒcovariance       
   p_last = p_now; //∏¸–¬covariance÷µ
   x_last = x_now; //∏¸–¬œµÕ≥◊¥Ã¨÷µ
   return x_now;                
}

static double KalmanFilter_speed(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=‘Î…˘
   kg=p_mid/(p_mid+R); //kgŒ™kalman filter£¨RŒ™‘Î…˘
   x_now=x_mid+kg*(ResrcData-x_mid);//π¿º∆≥ˆµƒ◊Ó”≈÷µ
                
   p_now=(1-kg)*p_mid;//◊Ó”≈÷µ∂‘”¶µƒcovariance       
   p_last = p_now; //∏¸–¬covariance÷µ
   x_last = x_now; //∏¸–¬œµÕ≥◊¥Ã¨÷µ
   return x_now;                
}

