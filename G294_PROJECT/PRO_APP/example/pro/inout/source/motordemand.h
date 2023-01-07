/***********************************************************************
G294在黄冈代码基础上新增需求定义
浙江恒强科技
软件平台部:F11500
2022/11/22
************************************************************************/

#ifndef  __MOTORDEMAND__
#define  __MOTORDEMAND__

typedef struct
{
	unsigned char  Mtdir :1;    //电机方向
	unsigned char  Men   :1;    //电机使能上电默认
	unsigned char  Currentlock; //锁定电流ma
	unsigned char  CurrentRun;  //运行电流	ma	
	unsigned short UpsSpeed;    //上升速度
	unsigned short DwonSpeed;   //下降速度
	unsigned short StopSpeed;   //减速速度
	unsigned short LeadScrewStroke_Max; //默认丝杆最大行程脉冲数
}MotorParameter;


typedef struct
{
	unsigned char  MotorTimeOutFlagUP :1;    //上运动超时标志
	unsigned char  MotorTimeOutFlagDwon :1;  //下运动超时标志
	unsigned char  MotorRunMode :4;          //运动模式 0:超时停机在当前位\
																											1:超时返回起点位
	unsigned char  SpeedReductionLock :1;    //下降锁定
	unsigned char  StartUpFlag :1;           //到上限位标志
	unsigned char  StartDwonFlag :1;         //到下限位标志    	
	unsigned short SpeedUp;                  //上限位速度
	unsigned short SpeedDwon;                //下限位速度	
	signed int     OriginPosUp;              //上运动前位置记录
	signed int     OriginPosDwon;            //下运动前位置记录	
  signed int     CurrentPosUp;             //上运动当前位置
	signed int     CurrentPosDwon;           //下运动当前位置	
	unsigned int   MotorMoveCountTime;       //运动计时
	unsigned int   SpeedUpCountTime;         //加速度计时
	unsigned int   SpeedDwonCountTime;       //减速计时	
}MotorMove;

#define MOTOROUTTIME                       5000
#define MAXPOS                             8000

extern unsigned char  	SensorReturnDa[8];           //传感器信号返回数据
extern unsigned char    MotorStateclock;  
extern unsigned char    MotorStateSendclock;        //电机报错状态发送锁  


extern unsigned char   SensorSig;                                         //信号当前状态 
extern unsigned char   LastSensorsignal;                                  //上一次信号记录 

extern unsigned char   StartActiveReporting;                              //主动上报启动条件标志
extern unsigned char   signalreturnflag;                                  //上报请求标志
extern unsigned char	 requestflag;																			  //上报请求标志

extern unsigned char  MotorStateSendclock;                                //电机报错状态发送锁  
extern unsigned char  MotorStateclock;                                    //电机运行状态发送锁  
extern unsigned char  MotorStateCache;                                    //电机状态缓存

extern unsigned short  MotorDataSendTimeCount;                            //SPI发送计时间隔



extern MotorMove MotMove;
extern MotorParameter MrPr;
extern unsigned short  MotorDataSendTimeCount;     
signed char CanRaedMassgaAnalysis(unsigned char * dat);

signed char SensorGetMotorStatePoll(void);

void ClearMotorState(unsigned char Ud);
signed char MoveUpTask(void);

void ClearMotorState(unsigned char Ud);
#endif


