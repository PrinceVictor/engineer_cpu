#ifndef __IMU_H_
#define __IMU_H_

#include "Clock.h"
#include "math.h"
#include "MyFunc.h"
#include "6050.h"
#include "ChassisTask.h"

#define GYRO_FILTER_NUM 10
#define Kp 1.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Kii 0.01f                     // integral gain governs rate of convergence of gyroscope biases
#define KALMAN_Q        0.02
#define KALMAN_R        6.0000
#define GYRO_FILTER_NUM 10

float invSqrt(float x);

#define RtA 		57.324841f		//  180/3.1415  角度制 转化为弧度制	

typedef struct{
	float roll;
	float pitch;
	float yaw;
}_angle;

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void readIMU(uint8_t);
void imu(int8_t);

extern _angle angle;

#endif
				

