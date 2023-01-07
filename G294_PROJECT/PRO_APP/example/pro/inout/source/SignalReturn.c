#include "motordemand.h"
#include "message.h"
#include "logic_lay.h"
#include "mot.h"
#include "tmc5610.h"


unsigned char  	SensorReturnDa[8]={0x71,0x0c,0,0,0,0,0,0};           //信号返回数据


unsigned char   StartActiveReporting=0;                              //主动上报启动条件标志

unsigned char   signalreturnflag=0;                                  //信号翻转标志
unsigned char   Motreturnflag=0;                                     //电机启停翻转标志
unsigned char   Errreturnflag=0;                                     //电机报错翻转标志

unsigned char		requestflag=0;																			 //上报请求标志
	
unsigned char   SensorSig=0;                                         //信号当前状态 
unsigned char   LastSensorsignal=0;                                  //上一次信号记录
unsigned char   MotorStateCache=0;                                   //电机运行缓存
unsigned char   MotorErrsta =0;                                      //电机错误状态

unsigned char   MotorStateSendclock=0;                                //电机报错状态发送锁  
unsigned char   MotorStateclock=0;                                    //电机运行状态发送锁

unsigned short  MotorDataSendTimeCount=0;                            //SPI发送计时间隔
/*
电机报错状态
*/
unsigned char GetMotorErrState(void) 
{
		if(StartActiveReporting)
		{
				if((!MotMove.MotorTimeOutFlagUP&&!MotMove.MotorTimeOutFlagDwon)&&MotorStateSendclock==1)
				{
					MotorErrsta=0x00;
					MotorStateSendclock=0;
				}		
				else if(MotMove.MotorTimeOutFlagDwon&&MotorStateSendclock==0)
				{
					MotorErrsta=0x02;
					MotorStateSendclock=1;
					signalreturnflag=4;
				}
				else if(MotMove.MotorTimeOutFlagUP&&MotorStateSendclock==0)
				{
					MotorErrsta=0x02;
					MotorStateSendclock=1;
					signalreturnflag=4;
				}
	 }
	 else
	 {
			MotMove.MotorTimeOutFlagUP=0;
			MotMove.MotorTimeOutFlagDwon=0;
	 }	
			return 0;
}
/*
电机运行状态
*/
 unsigned char GetMotorRunState(void) 
 {
	if(StartActiveReporting)
	{
		if(MotorStateCache==0x01&&MotorStateclock==0)
		{
					signalreturnflag=3;
					MotorStateclock=1;
		}			
  	else if(MotorStateCache==0x00&&MotorStateclock==1)
		{
			signalreturnflag=3;
			MotorStateclock=0;				
		}
	}

return 0;	
}

/*
传感器信号装载
*/
signed char SensorGetStatePoll(void)
{

	if(MotorDataSendTimeCount>=15*1)
	{
		SensorReturnDa[3] = inputsignal(UP_LIMIT_SIGNAL);
	  SensorReturnDa[3]|= inputsignal(DOWN_LIMIT_SIGNAL)<<1;
		SensorReturnDa[3]|= (mot_rightlimit_get(0))<<1;
	
		SensorSig=SensorReturnDa[3];
		MotorDataSendTimeCount=0;	
		
		MotorStateCache=mot_get_curspeed(0);
		
	}
	 
	return 0;
}

/*
传感器信号返回
	任意一个信号翻转立即主动上报一次
*/
signed SensorSignalSend(void)
{
		if(StartActiveReporting&&((SensorSig&0xff)!=(LastSensorsignal&0xff)))
		{
			LastSensorsignal=SensorSig;
			signalreturnflag=1;
			return 1;
		}
		else
		{
			 LastSensorsignal=SensorSig;	
		}
}


/*
信号返回状态任务
*/
signed char SensorGetMotorStatePoll(void)
{
	SensorGetStatePoll();
	SensorSignalSend();	
	
	GetMotorRunState();	
	GetMotorErrState();	
	switch (signalreturnflag)
	{	
			case 1:	
					CanMessagePushlist(&cantx_cmdlist,CAN_TX_ID+boardinfo.id,SensorReturnDa);
					signalreturnflag=0;						
					break;
			case 2:
					CanMessagePushlist(&cantx_cmdlist,CAN_TX_ID+boardinfo.id,SensorReturnDa);
					signalreturnflag=0;				
					break;			
			case 3:
				if(SensorReturnDa[2]==2&&MotorStateCache==0)
				{
						signalreturnflag=0;
				}
				else
				{
					SensorReturnDa[2]=MotorStateCache;
					CanMessagePushlist(&cantx_cmdlist,CAN_TX_ID+boardinfo.id,SensorReturnDa);
					signalreturnflag=0;
				}					
				break;
			case 4:
					SensorReturnDa[2]=MotorErrsta;
					CanMessagePushlist(&cantx_cmdlist,CAN_TX_ID+boardinfo.id,SensorReturnDa);
					signalreturnflag=0;		
					break;			
			
	}
	return 0;
	
}	

































