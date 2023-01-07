/***********************************************************************
G294在黄冈代码基础上新增需求
浙江恒强科技
软件平台部:F11500
2022/11/22
************************************************************************/

#include "motordemand.h"
#include "message.h"
#include "logic_lay.h"
#include "mot.h"
#include "tmc5610.h"

MotorParameter MrPr={1,1,4,19,\
										400,400,100,5000 
										};
								
/*
运动电流
*/
void MotRunCurrentSet(uint8_t idx,unsigned char* ru)
{
	int cur_reg=0;   
	unsigned short curvalue=0;
	curvalue =ru[3];
	curvalue|=ru[2]<<8;
  
	MrPr.CurrentRun=(32*curvalue)/6600;//最大有效电流硬件配置为4.7A   正弦峰值电流配置为6.6A
  cur_reg=((0x60000|(MrPr.CurrentRun<<8))|MrPr.Currentlock);
	tmc5160_writeInt(idx,TMC5160_IHOLD_IRUN,cur_reg);//0x60803     
}

/*
锁定电流
	根据总电流划分1-15个挡位
*/
void MotClockCurrentSet(uint8_t idx,unsigned char* ck)
{
	unsigned short curvalue=0;
	int cur_reg=0; 
  curvalue=ck[3];
	curvalue|=ck[2]<<8;
	if(curvalue>=1&&curvalue<=15)
	{	
		MrPr.Currentlock=	MrPr.CurrentRun/curvalue;
		cur_reg=((0x60000|(MrPr.CurrentRun<<8))|MrPr.Currentlock);	
		tmc5160_writeInt(idx,TMC5160_IHOLD_IRUN,cur_reg);
	}
}

/*
设置最行程
*/			
signed SetMaxLeadScrewStroke(unsigned char* Vl)
{
	MrPr.LeadScrewStroke_Max = Vl[3];
  MrPr.LeadScrewStroke_Max|= Vl[4]<<8;
	return 0;
}	

/*
电机使能
*/
void SetMotorEn(unsigned char *en)
{
	if(en[2]==1)
	{
		mot_enable(0,0);
	}
	else if(en[2]==0)
	{
		mot_enable(0,1);
	}
}

/*
走上偏移量
offset:协议数据
*/
signed char MoveOffsetUp(unsigned char* offset)
{
	unsigned short moveoffset=0;
	MotMove.MotorTimeOutFlagUP=0;
	MotMove.MotorTimeOutFlagDwon=0;
	
	moveoffset =offset[4];
	moveoffset|=offset[3]<<8;
	mot_run_relative_pos_mode(0,MrPr.UpsSpeed,moveoffset);
	return 0;
}
/*
走下偏移量
offset:协议数据
*/
signed char MoveOffsetDwon(unsigned char* offset)
{
	signed short moveoffset=0;
	
	MotMove.MotorTimeOutFlagUP=0;
	MotMove.MotorTimeOutFlagDwon=0;
	
	moveoffset =offset[4];
	moveoffset|=offset[3]<<8;
	moveoffset=0-moveoffset;
	mot_run_relative_pos_mode(0,MrPr.DwonSpeed,moveoffset);
	return 0;
}


/*
Can消息接受解析
dat:数据首地址
*/
signed char CanRaedMassgaAnalysis(unsigned char * dat)
{
	SensorGetMotorStatePoll();
	
	MoveUpTask();
	//停止
	if(dat[0]==0xD4&&dat[1]==0x01&&dat[2]==0x00)
	{			
		mot_run_relative_pos_mode(0,MrPr.UpsSpeed,0);
		MotMove.StartUpFlag=0;
		MotMove.StartDwonFlag=0;
//		ClearMotorState(0);
//		ClearMotorState(1);
	}
	//上偏移	
	else if(dat[0]==0xD4&&dat[1]==0x02&&dat[2]==0x01)
	{
		MoveOffsetUp(dat);
	}
	//下偏移
	else if(dat[0]==0xD4&&dat[1]==0x02&&dat[2]==0x02)
	{
		MoveOffsetDwon(dat);
	}
	//信号返回
	else if(dat[0]==0x71&&dat[1]==0x0c)
	{
		 requestflag=1;
		 StartActiveReporting=1;	
		 signalreturnflag=2;	
	}
	//方向设置
	else if(dat[0]==0xD4&&dat[1]==0x10&&(dat[2]==0||dat[2]==1))
	{
			MrPr.Mtdir=dat[2];
			mot_dir_config(0,MrPr.Mtdir);
	}		
	//使能
	else if(dat[0]==0xD4&&dat[1]==0x11)
	{
		SetMotorEn(dat);
	}	
	//上运行速度设置
	else if(dat[0]==0xD4&&dat[1]==0x12)
	{
		MrPr.UpsSpeed=dat[3];
		MrPr.UpsSpeed |=dat[2]<<8;
//		tmc5160_writeInt(0,TMC5160_VMAX,MrPr.UpsSpeed*MT_SPD_ADJUST);
	}	
	//下运行速度设置
	else if(dat[0]==0xD4&&dat[1]==0x13)
	{
		MrPr.DwonSpeed =dat[3];
		MrPr.DwonSpeed |=dat[2]<<8;
//		tmc5160_writeInt(0,TMC5160_VMAX,MrPr.DwonSpeed*MT_SPD_ADJUST);
	}
	//减速度
	else if(dat[0]==0xD4&&dat[1]==0x14)
	{
		MrPr.StopSpeed =dat[3];
		MrPr.StopSpeed |=dat[2]<<8;
		mot_D1_set(0,MrPr.StopSpeed);
	}
	//运行电流	
	else if(dat[0]==0xD4&&dat[1]==0x15)
	{
		MotRunCurrentSet(0,dat);
	}
	//锁定电流
	else if(dat[0]==0xD4&&dat[1]==0x16)
	{
		MotClockCurrentSet(0,dat);
	}
	//到上限位
	else if(dat[0]==0xD4&&dat[1]==0x01&&dat[2]==0x01)
	{
		ClearMotorState(0);
		MotMove.OriginPosUp=mot_curpos_get(0);
		MotMove.StartUpFlag=1;	
	}
	//到下限位	
	else if(dat[0]==0xD4&&dat[1]==0x01&&dat[2]==0x02)
	{
		ClearMotorState(1);
		MotMove.OriginPosDwon=mot_curpos_get(0);
		MotMove.StartDwonFlag=1;
	}	
	//运动模式0
	else if(dat[0]==0xD4&&dat[1]==0x01&&dat[2]==0x04)
	{
		MotMove.MotorRunMode=0;
	}		
	//运动模式1
	else if(dat[0]==0xD4&&dat[1]==0x01&&dat[2]==0x03)
	{
		MotMove.MotorRunMode=1;
	}		
	//设置行程
	else if(dat[0]==0xD4&&dat[1]==0x17)
	{
		SetMaxLeadScrewStroke(dat);
	}		
	memset(dat,0,8);
	return 0;
}






