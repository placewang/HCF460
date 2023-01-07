/***********************************************************************
G294在黄冈代码基础上新增需求
浙江恒强科技
软件平台部:F11500
2022/11/27
************************************************************************/

#include "motordemand.h"
#include "message.h"
#include "logic_lay.h"
#include "mot.h"
#include "tmc5610.h"

MotorMove MotMove={0};	

char startstate=0;                         //执行一次位置指令（锁）

#define     ADDSPEED             5

unsigned short    OFFSETINPLACE =5;




/*
清电机状态
Ud:0:上限位
	 1:下限位
*/

void ClearMotorState(unsigned char Ud)
{
	MotMove.MotorTimeOutFlagUP=0;
	MotMove.MotorTimeOutFlagDwon=0;
	startstate=0;
	if(Ud==0)
	{
		MotMove.MotorMoveCountTime=0;

		MotMove.CurrentPosUp=0;
		MotMove.SpeedUpCountTime=0;
		MotMove.SpeedUp=0;	
	}
	else if(Ud==1)
	{
		MotMove.MotorMoveCountTime=0;

		MotMove.CurrentPosDwon=0;
		MotMove.SpeedDwonCountTime=0;
		MotMove.SpeedDwon=0;
		MotMove.SpeedReductionLock=0;	
	}
}

/*
电机加速
dr:0 上升加速
	 1 下降加速
*/
signed char MotorAddSpeed(unsigned dr)
{
	if(dr==0)
	{
		if(MrPr.UpsSpeed>=MotMove.SpeedUp&&MotMove.SpeedUpCountTime>=10)
		{
			MotMove.SpeedUp+=ADDSPEED;
			MotMove.SpeedUpCountTime=0;
			tmc5160_writeInt(0,TMC5160_VMAX,MotMove.SpeedUp*MT_SPD_ADJUST);
		}
	}
	else if(dr==1)
	{
		if(MrPr.DwonSpeed >=MotMove.SpeedDwon && MotMove.SpeedDwonCountTime>=10)
		{
			MotMove.SpeedDwon+=ADDSPEED;
			MotMove.SpeedDwonCountTime=0;
			tmc5160_writeInt(0,TMC5160_VMAX,MotMove.SpeedDwon*MT_SPD_ADJUST);
		}
		
	}
	return 0;
	
}	



/*
到上限位 模式 0
*/
signed MotorMoveUpMode0(void)
{
	unsigned short timeout=0;
	signed int pos=0;	
	
	MotorAddSpeed(0);
	switch (startstate)
	{	
		case 0 :
				startstate=1;
				mot_run_relative_pos_mode(0,MotMove.SpeedUp,MrPr.LeadScrewStroke_Max);
				break;
		case 1:
			pos=mot_curpos_get(0);

			if(inputsignal(UP_LIMIT_SIGNAL) ==0)
			{
				mot_run_relative_pos_mode(0,MotMove.SpeedUp/3,OFFSETINPLACE);		
				MotMove.StartUpFlag=0;
				startstate=0;
				return 1;
			}
			if(inputsignal(UP_LIMIT_SIGNAL) ==1&&pos==(MrPr.LeadScrewStroke_Max+MotMove.OriginPosUp)-1)
			{
				mot_runbysped_mode(0,0,0);
				MotMove.MotorTimeOutFlagUP=1;
				MotMove.StartUpFlag=0;
				startstate=0;
				return -1;
			}
			break;			
	}
	return 0;
}
/*
到上限位 模式 1
*/
signed MotorMoveUpMode1(void)
{
	unsigned short timeout=0;
	signed int pos=0;
	
	MotorAddSpeed(0);
	switch (startstate)
	{	
		case 0 :
				startstate=1;
				mot_run_relative_pos_mode(0,MotMove.SpeedUp,MrPr.LeadScrewStroke_Max);
				break;
		 case 1:
				pos=mot_curpos_get(0);			 
				if(inputsignal(UP_LIMIT_SIGNAL) ==0)
				{
					mot_run_relative_pos_mode(0,MotMove.SpeedUp/3,OFFSETINPLACE);		
					MotMove.StartUpFlag=0;
					startstate=0;
					return 1;
				}
				if(inputsignal(UP_LIMIT_SIGNAL) ==1&&pos==(MrPr.LeadScrewStroke_Max+MotMove.OriginPosUp)-1)
				{
					mot_runbysped_mode(0,0,1);
					MotMove.MotorMoveCountTime=0;
					startstate=2;
				}
				break;
		 case 2:
				if(MotMove.MotorMoveCountTime>=100*5)
				{
					mot[0].run_speed=MrPr.UpsSpeed;
					mot_runpos_mode(0,MotMove.OriginPosUp);	
					MotMove.StartUpFlag=0;
					startstate=0;
					MotMove.MotorTimeOutFlagUP=1;
					return -1;		
				}
				break;
	}
	return 0;
}

/*
到下限位 模式 0
*/
signed MotorMoveDwonMode0(void)
{
	unsigned short timeout=0;
	signed int pos=0,posval=0;	
	MotorAddSpeed(1);
	switch (startstate)
	{	
		case 0 :
			startstate=1;
			posval=0-MrPr.LeadScrewStroke_Max;
			mot_run_relative_pos_mode(0,MotMove.SpeedDwon,posval);
			break;
		case 1:
				pos=mot_curpos_get(0);				
			//减速点
			if(mot_rightlimit_get(0)==0)
			{
				tmc5160_writeInt(0,TMC5160_VMAX,10*MT_SPD_ADJUST);
			}				
			if(inputsignal(DOWN_LIMIT_SIGNAL) ==0)
			{
				mot_run_relative_pos_mode(0,MotMove.SpeedDwon/3,OFFSETINPLACE*-1);		
				MotMove.StartDwonFlag=0;
				startstate=0;
				return 1;
			}
			else if(inputsignal(DOWN_LIMIT_SIGNAL) ==1&&pos==(MotMove.OriginPosDwon-MrPr.LeadScrewStroke_Max)+1)
			{
				mot_runbysped_mode(0,0,0);
				MotMove.MotorTimeOutFlagDwon=1;
				MotMove.StartDwonFlag=0;
				startstate=0;
				return -1;
			}
			break;		
	}
		return 0;
}



/*
到下限位 模式 1
*/
signed MotorMoveDwonMode1(void)
{
	unsigned short timeout=0;
	signed int pos=0,posval=0;	
	MotorAddSpeed(1);
	switch (startstate)
	{	
		case 0 :
			startstate=1;
			posval=0-MrPr.LeadScrewStroke_Max;
			mot_run_relative_pos_mode(0,MotMove.SpeedDwon,posval);
			break;
		case 1:		
			pos=mot_curpos_get(0);			
			//减速点
			if(mot_rightlimit_get(0)==0&&MotMove.MotorTimeOutFlagDwon==0)
			{
				tmc5160_writeInt(0,TMC5160_VMAX,10*MT_SPD_ADJUST);
			}
			if(inputsignal(DOWN_LIMIT_SIGNAL) ==0)
			{
				mot_run_relative_pos_mode(0,MotMove.SpeedDwon/3,OFFSETINPLACE*-1);		
				MotMove.StartDwonFlag=0;
				startstate=0;
				return 1;
			}
			else if(inputsignal(DOWN_LIMIT_SIGNAL) ==1&&pos==(MotMove.OriginPosDwon-MrPr.LeadScrewStroke_Max)+1)
			{
				mot_runbysped_mode(0,0,1);
				MotMove.MotorMoveCountTime=0;
				startstate=2;
			}
			break;
		case 2:
			if(MotMove.MotorMoveCountTime>=100*5)
			{
				mot[0].run_speed=MrPr.DwonSpeed;
				mot_runpos_mode(0,MotMove.OriginPosDwon);
				startstate=0;
				MotMove.StartDwonFlag=0;		
				MotMove.MotorTimeOutFlagDwon=1;
				return -1;
			}			
			break;
	}
	return 0;
}



/*
运动任务
*/
signed char MoveUpTask(void)
{	
	if(MotMove.StartUpFlag&&MotMove.MotorRunMode==0)
	{
		MotorMoveUpMode0();
	}
	else if(MotMove.StartDwonFlag&&MotMove.MotorRunMode==0)
	{
		MotorMoveDwonMode0();
	}
	else if(MotMove.StartUpFlag&&MotMove.MotorRunMode==1)
	{
		MotorMoveUpMode1();
	}	
	else if(MotMove.StartDwonFlag&&MotMove.MotorRunMode==1)
	{
		MotorMoveDwonMode1();
	}	
	return 0;
}