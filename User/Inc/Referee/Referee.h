#ifndef __REFEREE_H__
#define __REFEREE_H__

#include "MyFunc.h"
#include "string.h"

#define TX_LEN 29

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define BSP_USART3_DMA_RX_BUF_LEN	30
#define REFEREE_REV_DMA_CNT (DMA1_Stream1->MA_CNDTR)//DMA?????

typedef struct
{
	uint8_t status_flag;
	uint8_t turn_flag;
	uint8_t move_flag;
  uint8_t direction;
}_lidar_flag;

typedef struct
{
	_lidar_flag flag;
	float d1;
	float d2;
	float angle;
	float get_x;
  
}_lidar_message;

typedef struct
{
	uint16_t left_time_S;
	uint8_t game_status;
	uint8_t robot_level;
	uint16_t left_HP;
	uint16_t full_HP; 
}_JUDGMENT_01_DATA;

typedef struct
{
	uint8_t was_attacked_id; //0 front, 1 left, 2 back, 3 right, 4 up1 5 up2
	uint8_t was_attacked_type; //0x0 attacked 0x01 module offline
}_JUDGMENT_02_DATA;

typedef struct
{
	uint8_t bullet_type;
	float bullet_freqz;
	float bullet_speed;
  
}_JUDGMENT_03_DATA;

typedef struct
{
	float chassis_voltage;
	float chassis_current;
	float chassis_power;
	float chassis_power_left;
	uint16_t small_shooter_heat;
	uint16_t big_shooter_heat;
  
}_JUDGMENT_04_DATA;

typedef struct
{
	uint8_t cardtype;
	uint8_t cardsubnum;
}_JUDGMENT_05_DATA;

typedef struct
{
	uint8_t game_result;
}_JUDGMENT_06_DATA;

typedef struct
{
	uint8_t buff;
	uint8_t buff_percent;
}_JUDGMENT_07_DATA;

typedef struct
{
	float x;
	float y;
	float z;
	float degree;
}_JUDGMENT_08_DATA;

typedef struct
{
	float data1;
	float data2;
	float data3;
	uint8_t whole_status;
}_diy_send;


void New_Send_Data(uint8_t *data,uint16_t size);
void refereeConfig(void);
void nucConfig(void);
extern _JUDGMENT_01_DATA Judgment_01_data;
extern _JUDGMENT_02_DATA Judgment_02_data;
extern _JUDGMENT_03_DATA Judgment_03_data;
extern _lidar_message lidar;

void send_odm_msg1(float *);


extern uint8_t Tx_Buf[TX_LEN];
extern uint8_t Tx_Buf2[TX_LEN]; 

extern unsigned char Get_CRC8_Check_Sum(unsigned char *,unsigned int,unsigned char );
unsigned int Verify_CRC8_Check_Sum(unsigned char *, unsigned int );
void Append_CRC8_Check_Sum(unsigned char *, unsigned int );
uint16_t Get_CRC16_Check_Sum(uint8_t *,uint32_t ,uint16_t );
uint32_t Verify_CRC16_Check_Sum(uint8_t *, uint32_t );
void Append_CRC16_Check_Sum(uint8_t * ,uint32_t );
extern void SendtoReferee(uint8_t *);

#endif


