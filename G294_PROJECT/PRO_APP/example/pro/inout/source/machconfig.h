#ifndef __MACHCONFIG_H__
#define __MACHCONFIG_H__
#include "stdint.h"

#define  APP_VER_H     0x01
#define  APP_VER_L     0x02

#define INIT_BOARDID 0
#define NUM_MOTOR 2
#define INPUT_N 3 //3路扩展输入口
#define OUTPUT_N 3
#define TIME0_LEVEL 1000//1000us
#define N_MAX_VALVE  256  //最多的气阀数量
#define N_VALVEGROUP_DEF  16//默认的气阀组数
#define N_VALVE_EVERYGROUP 8//每组气阀个数
#define BOARDTYPE_N  10//预留10种类型的板子
#define HOSTMAX  2
#define MOT_CUR_DEFAULT  3000//电机默认设置的电流 3000mA
//#define MOT_CUR_DEFAULT  3500//电机默认设置的电流 3000mA
enum
{
 DEVID_MOT1=10, //升降电机
 DEVID_MOT2=9,    //风门电机
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












