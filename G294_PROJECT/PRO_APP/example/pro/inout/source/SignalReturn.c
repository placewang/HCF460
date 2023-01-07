#include "motordemand.h"
#include "message.h"
#include "logic_lay.h"
#include "mot.h"
#include "tmc5610.h"


unsigned char  	SensorReturnDa[8]={0x71,0x0c,0,0,0,0,0,0};           //�źŷ�������


unsigned char   StartActiveReporting=0;                              //�����ϱ�����������־

unsigned char   signalreturnflag=0;                                  //�źŷ�ת��־
unsigned char   Motreturnflag=0;                                     //�����ͣ��ת��־
unsigned char   Errreturnflag=0;                                     //�������ת��־

unsigned char		requestflag=0;																			 //�ϱ������־
	
unsigned char   SensorSig=0;                                         //�źŵ�ǰ״̬ 
unsigned char   LastSensorsignal=0;                                  //��һ���źż�¼
unsigned char   MotorStateCache=0;                                   //������л���
unsigned char   MotorErrsta =0;                                      //�������״̬

unsigned char   MotorStateSendclock=0;                                //�������״̬������  
unsigned char   MotorStateclock=0;                                    //�������״̬������

unsigned short  MotorDataSendTimeCount=0;                            //SPI���ͼ�ʱ���
/*
�������״̬
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
�������״̬
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
�������ź�װ��
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
�������źŷ���
	����һ���źŷ�ת���������ϱ�һ��
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
�źŷ���״̬����
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

































