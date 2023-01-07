#ifndef __MACHCONFIG_H__
#define __MACHCONFIG_H__
#include "stdint.h"

#define  APP_VER_H     0x01
#define  APP_VER_L     0x02

#define INIT_BOARDID 0
#define NUM_MOTOR 2
#define INPUT_N 3 //3·��չ�����
#define OUTPUT_N 3
#define TIME0_LEVEL 1000//1000us
#define N_MAX_VALVE  256  //������������
#define N_VALVEGROUP_DEF  16//Ĭ�ϵ���������
#define N_VALVE_EVERYGROUP 8//ÿ����������
#define BOARDTYPE_N  10//Ԥ��10�����͵İ���
#define HOSTMAX  2
#define MOT_CUR_DEFAULT  3000//���Ĭ�����õĵ��� 3000mA
//#define MOT_CUR_DEFAULT  3500//���Ĭ�����õĵ��� 3000mA
enum
{
 DEVID_MOT1=10, //�������
 DEVID_MOT2=9,    //���ŵ��
 DEVID_VALVE=11,
 DEVID_INOUT=12,
};
enum 
{ 
 CAN_RX_ID=0x01,
 CAN_TX_ID=0x100,    
};
 typedef struct
{
 uint16_t  hardware;
}_BOARDDEF;


extern const _BOARDDEF boarddef[BOARDTYPE_N];

#endif












