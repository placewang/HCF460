#ifndef __MOT_H
#define __MOT_H
#include "machconfig.h"


#define  MT_SPD_ADJUST     1180 // demo经验值
#define  MT_XF             256 //电机默认细分
#define  MT_DEFAULT_V      500 //电机默认的最大速度
#define  MT_DEFAULT_RSTEPS 400 //一圈值200   电机默认的复位步数

#define  MT_RESET_SPD  50 
#define  MT_RUN_SPD 500 
#define  MT_MICRO_STEPS  (MT_XF*200))
#define  MT_RESET_SPD_TYPE   0
#define  MT_RUN_SPD_TYPE 1

enum
{
 MOT_RESETSTS_IDLE=0,
 MOT_RESETSTS_GOIN=1,
 MOT_RESETSTS_GOOUT=2,
 MOT_RESETSTS_REACHZERO=3,
};

typedef struct
{
 uint8_t config_dir;
 uint8_t dir;
 uint8_t isrunning;
 uint16_t xf;
 int32_t pos;
 uint32_t reset_maxstep;
 uint8_t  reset_sts;
 uint32_t  reset_speed;
 uint32_t  run_speed;
 uint16_t delayset;
 uint8_t reset_flag ;
 int    default_current ;
 int32_t limit_pos ;
 uint32_t d_speed ; //减速速度
 uint32_t cur_speed;
 uint32_t reset_pos_limit ;//上升下降最大行程设置
 int32_t  beg_reset_pos ;
 uint16_t up_run_spd ;
 uint16_t down_run_spd ;
}_MOTDEF;
extern _MOTDEF mot[NUM_MOTOR];






extern void mot_init(void);
extern void mot_spd_config(uint8_t idx,uint8_t type,uint16_t rpm_s);
extern void mot_runbysped_mode(uint8_t idx,uint8_t dir,uint32_t speed);
extern void mot_runreset(uint8_t idx);
extern void mot_xf_config(uint8_t idx,uint16_t xf);
extern int mot_curpos_get(uint8_t idx);
extern void mot_curpos_set(uint8_t idx,int pos);
extern uint8_t mot_get_curspeed(uint8_t idx);
extern void mot_runpos_mode(uint8_t idx,uint32_t pos);
extern void mot_dir_config(uint8_t idx,uint8_t dir);
extern void mot_current_set(uint8_t idx,uint32_t curvalue);
extern uint8_t mot_leftlimit_get(uint8_t idx);
extern uint8_t mot_rightlimit_get(uint8_t idx);
extern void mot_errdeal_register(void (*callback)(uint8_t));
extern void mot_enable(uint8_t idx,uint8_t en);//0使能
extern int mot_limit_pos_get(uint8_t idx) ;
extern void mot_d_spd_config(uint8_t idx,uint16_t spd) ;

extern void mot_V1_set(uint8_t idx,uint32_t value);
extern void mot_A1_set(uint8_t idx,uint32_t value);
extern void mot_AMAX_set(uint8_t idx,uint32_t value);


extern void mot_VStop_set(uint8_t idx,uint32_t value) ;
extern void mot_D1_set(uint8_t idx,uint32_t value);
extern void mot_DMAX_set(uint8_t idx,uint32_t value);
extern void mot_reset_pos_set (uint8_t idx,uint32_t value) ;
extern void mot_run_relative_pos_mode(uint8_t idx,uint16_t spd,uint32_t pos);
#endif


































