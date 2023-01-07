#include "mot.h"
#include "board_g294.h"
#include "tmc5610.h"

_MOTDEF mot[NUM_MOTOR];
void mot_enable(uint8_t idx,uint8_t en);

int modetest=0;
void mot_init(void)
{
 //uint8_t regconst;
 uint8_t i;
 unsigned char setcur_a,lockcur_a;
 int cur_reg=0;  	
 setcur_a=32*(4000)/6600;//最大有效电流硬件配置为4A   
 lockcur_a=setcur_a/5;

 cur_reg=((0x60000|(setcur_a<<8))|lockcur_a);
 memset(mot,0,sizeof(mot));
 mot[0].default_current = cur_reg ; //升降电机
	
 setcur_a=32*(MOT_CUR_DEFAULT)/6600;//最大有效电流硬件配置为3A
// lockcur_a=setcur_a/5;
 lockcur_a=setcur_a/3;
	//锁定电流
 cur_reg=((0x60000|(setcur_a<<8))|lockcur_a);
 mot[1].default_current = cur_reg ; //风门电机
 for(i=0; i<NUM_MOTOR; i++)
   { 
    mot[i].dir=1;//0
    mot[i].xf=MT_XF;  
    mot[i].reset_maxstep=MT_DEFAULT_RSTEPS;
 		mot[i].reset_flag = 0 ;
		mot[i].run_speed = 100;
		mot[i].d_speed = 100 ;
		mot[i].reset_pos_limit =0 ;
    mot_spd_config(i,MT_RESET_SPD_TYPE,MT_RESET_SPD); // RPM
    mot_spd_config(i,MT_RUN_SPD_TYPE,MT_RUN_SPD); //  RPM         
    motor_en(i,1);
    tmc5160_writeInt(i,TMC5160_CHOPCONF,0x100C3);

    mot_xf_config(i,mot[i].xf);
    mot_dir_config(i,mot[i].dir);
    tmc5160_writeInt(i,TMC5160_IHOLD_IRUN,mot[i].default_current);//0x61003);  // IHOLD IRUN  Imax*3/32    Imax*16/32
    tmc5160_writeInt(i,TMC5160_TPOWERDOWN,0x0A);  // TPOWERDOWN  DELAY BEFORE POWER DOWN IN STAND STILL
		tmc5160_writeInt(i,TMC5160_VSTART,0x00008);
	  //	
	/* 	if(i==0)
		{
			tmc5160_writeInt(i,TMC5160_V1,0x000400);
			tmc5160_writeInt(i,TMC5160_A1,0x008800);     // A1 > AMAX
			tmc5160_writeInt(i,TMC5160_AMAX,0x001000);  
			
		}
		else */
		{
			tmc5160_writeInt(i,TMC5160_V1,0x000800);
			tmc5160_writeInt(i,TMC5160_A1,0x00FF00);     // A1 > AMAX
			tmc5160_writeInt(i,TMC5160_AMAX,0x00F000);
		}
		 
		tmc5160_writeInt(i,TMC5160_DMAX,0x00F000);   //  
		tmc5160_writeInt(i,TMC5160_D1,0x00FF00);     // D1 > DMAX
		 
    tmc5160_writeInt(i,TMC5160_VSTOP,0x7D0);
	//	tmc5160_writeInt(i,TMC5160_VSTOP,400*MT_SPD_ADJUST);
	 
/*  tmc5160_writeInt(i,TMC5160_DMAX,0x000500);   //  
	 	tmc5160_writeInt(i,TMC5160_D1,0x000500);     // D1 > DMAX
    tmc5160_writeInt(i,TMC5160_VSTOP,0xD0);	 */
    tmc5160_writeInt(i,TMC5160_VMAX,MT_DEFAULT_V*MT_SPD_ADJUST);        
    tmc5160_writeInt(i,TMC5160_RAMPSTAT,0xFF);//新界面增加  

    tmc5160_writeInt(i, TMC5160_XACTUAL,0);//设置位置
  /*  if(i==0) //升降电机 需要用到限位
		{
			  tmc5160_writeInt(i, TMC5160_SWMODE,0x5f);//设置左右限位
		}*/
  }	
}
//电机使能
void mot_enable(uint8_t idx,uint8_t en)//0使能
{
 motor_en(idx,en);
}
void mot_d_spd_config(uint8_t idx,uint16_t spd)
{
	 if(idx>=NUM_MOTOR)
    return;
   mot[idx].d_speed = spd;
}
//电机方向配置
void mot_dir_config(uint8_t idx,uint8_t dir)
{
  uint32_t config;
  _MOTDEF *smot=NULL;
  if(idx>=NUM_MOTOR)
    return;
  smot=&mot[idx];
  smot->config_dir=dir;
  
  config=tmc5160_readInt(idx,TMC5160_GCONF); 
   
  if(smot->config_dir)
     config |=(1<<4); 
  else
     config &=(~(1<<4));        
  tmc5160_writeInt(idx,TMC5160_GCONF,config);
}

//电机速度设置
void mot_spd_config(uint8_t idx,uint8_t type,uint16_t rpm_s)
{
   _MOTDEF *smot=NULL;
  if(idx>=NUM_MOTOR)
    return;
  smot=&mot[idx];
 if(type==MT_RESET_SPD_TYPE)
   smot->reset_speed=rpm_s;
 else
   smot->run_speed=rpm_s;      
  //tmc5160_writeInt(idx,TMC5160_VMAX,smot->run_speed*MT_SPD_ADJUST); 
}
//获取当前电机的实际速度
uint8_t mot_get_curspeed(uint8_t idx)
{
 if(tmc5160_readInt(idx,TMC5160_RAMPSTAT)&(1<<10))
   return 0;
 else
   return 1;

}

//电机细分设置
const int xf_list[9]={0,1,2,3,4,5,6,7,8};
void mot_xf_config(uint8_t idx,uint16_t xf)
{
 uint32_t config_para;
 _MOTDEF *smot=NULL;
  if(idx>=NUM_MOTOR)
    return;
 smot=&mot[idx];
 smot->xf=xf;
  switch(xf)
   {
    case 256: config_para=((xf_list[0]<<24)| 0x100C3);break; 
    case 128: config_para=((xf_list[1]<<24)| 0x100C3);break; 
    case 64:  config_para=((xf_list[2]<<24)| 0x100C3);break; 
    case 32:  config_para=((xf_list[3]<<24)| 0x100C3);break; 
    case 16:  config_para=((xf_list[4]<<24)| 0x100C3);break; 
    case 8:   config_para=((xf_list[5]<<24)| 0x100C3);break; 
    case 4:   config_para=((xf_list[6]<<24)| 0x100C3);break; 
    case 2:   config_para=((xf_list[7]<<24)| 0x100C3);break; 
    case 1:   config_para=((xf_list[8]<<24)| 0x100C3);break;     
   }
  tmc5160_writeInt(idx,TMC5160_CHOPCONF,config_para);
 // code_reg_const=tmc_codeconst(rx_msg.Data[1],mot[rx_msg.Data[1]].xf,mot[rx_msg.Data[1]].code_const);
//  tmc5160_writeInt(rx_msg.Data[1],TMC5160_ENC_CONST,code_reg_const); 
}

//电机位置获取
int mot_curpos_get(uint8_t idx)
{
  if(idx>=NUM_MOTOR)
    return 0;
  return tmc5160_readInt(idx,TMC5160_XACTUAL)/mot[idx].xf;
//	return tmc5160_readInt(idx,TMC5160_XACTUAL);
}
int mot_limit_pos_get(uint8_t idx)
{
	 if(idx>=NUM_MOTOR)
		 return 0 ; 
	// return tmc5160_readInt(idx,TMC5160_XLATCH)/mot[idx].xf ;  
	 return tmc5160_readInt(idx,TMC5160_XLATCH);  
}
uint8_t mot_leftlimit_get(uint8_t idx)
{
 return ((tmc5160_readInt(idx,TMC5160_INP_OUT)&0x01));//读左限位  
}

//减速点
uint8_t mot_rightlimit_get(uint8_t idx)
{
 return ((tmc5160_readInt(idx,TMC5160_INP_OUT)&0x02));//读右限位  
}



//设置位置
void mot_curpos_set(uint8_t idx,int pos)
{
  if(idx>=NUM_MOTOR)
    return;
 setPosition(idx,pos*mot[idx].xf);
}
//运动电流
void mot_current_set(uint8_t idx,uint32_t curvalue)
{
 unsigned char setcur_a,lockcur_a;
 int cur_reg=0;   

 setcur_a=(32*curvalue)/6600;//最大有效电流硬件配置为4.7A   正弦峰值电流配置为6.6A
 lockcur_a=setcur_a/5;
 cur_reg=((0x60000|(setcur_a<<8))|lockcur_a);
 tmc5160_writeInt(idx,TMC5160_IHOLD_IRUN,cur_reg);//0x60803     
}
//初始段速度
void mot_V1_set(uint8_t idx,uint32_t value)
{
	 tmc5160_writeInt(idx,TMC5160_V1,value);
}
//初始段加速度
void mot_A1_set(uint8_t idx,uint32_t value)
{
	 tmc5160_writeInt(idx,TMC5160_A1,value);
}
//目标速度
void mot_AMAX_set(uint8_t idx,uint32_t value)
{
		tmc5160_writeInt(idx,TMC5160_AMAX,value);
}
//停止速度初始段
void mot_VStop_set(uint8_t idx,uint32_t value)
{
		tmc5160_writeInt(idx,TMC5160_VSTOP,value);
}
//减速度
void mot_D1_set(uint8_t idx,uint32_t value)
{
		tmc5160_writeInt(idx,TMC5160_D1,value);
}
//目标减速度
void mot_DMAX_set(uint8_t idx,uint32_t value)
{
		tmc5160_writeInt(idx,TMC5160_DMAX,value);
}
//复位位置
void mot_reset_pos_set (uint8_t idx,uint32_t value)
{
		mot[idx].reset_pos_limit = value ;
}
//电机速度模式下运行
void mot_runbysped_mode(uint8_t idx,uint8_t dir,uint32_t speed)//rpm
{
	uint32_t speedadjust=0;

  if(idx>=NUM_MOTOR)
    return;

 speedadjust=speed*MT_SPD_ADJUST;
  if(dir)
  {
   left(idx,speedadjust);
  }
  else
  {
   right(idx,speedadjust);
  }
}

//电机跑位置 绝对位置
void mot_runpos_mode(uint8_t idx,uint32_t pos)
{
  if(idx>=NUM_MOTOR)
    return;
  tmc5160_writeInt(idx,TMC5160_VMAX,mot[idx].run_speed*MT_SPD_ADJUST); 
  moveTo(idx,pos*mot[idx].xf);
}
//电机跑位置 相对位置
void mot_run_relative_pos_mode(uint8_t idx,uint16_t spd,uint32_t pos)
{
	int32_t target_pos = pos*mot[idx].xf;
	 if(idx>=NUM_MOTOR)
    return;
  tmc5160_writeInt(idx,TMC5160_VMAX,spd*MT_SPD_ADJUST); 
  moveBy(idx,&target_pos);
}
//电机复位
void mot_runreset(uint8_t idx)
{
  _MOTDEF *smot=NULL;
  if(idx>=NUM_MOTOR)
    return;
   smot=&mot[idx];
  if(smot->isrunning)
    return;
  
   if(zerosignal(idx)==0)//已经在零位
   {  
     mot_curpos_set(idx,0);
     smot->pos=0;
     smot->reset_sts=MOT_RESETSTS_IDLE;
     smot->isrunning=0;       
		 smot->reset_flag =1;
   }
   else
   {
		// smot->pos=1000;
     tmc5160_writeInt(idx,TMC5160_VMAX,smot->reset_speed*MT_SPD_ADJUST); 
     tmc5160_writeInt(idx, TMC5160_XACTUAL,smot->reset_maxstep*smot->xf);
     moveTo(idx,0);
     smot->reset_sts=MOT_RESETSTS_GOIN;
     smot->isrunning=1;      	 
   }

}

void mot_errdeal_register(void (*callback)(uint8_t))
{
 spi_err_register(callback);
}

























