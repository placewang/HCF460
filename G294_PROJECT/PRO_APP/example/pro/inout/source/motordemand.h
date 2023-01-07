/***********************************************************************
G294�ڻƸԴ������������������
�㽭��ǿ�Ƽ�
���ƽ̨��:F11500
2022/11/22
************************************************************************/

#ifndef  __MOTORDEMAND__
#define  __MOTORDEMAND__

typedef struct
{
	unsigned char  Mtdir :1;    //�������
	unsigned char  Men   :1;    //���ʹ���ϵ�Ĭ��
	unsigned char  Currentlock; //��������ma
	unsigned char  CurrentRun;  //���е���	ma	
	unsigned short UpsSpeed;    //�����ٶ�
	unsigned short DwonSpeed;   //�½��ٶ�
	unsigned short StopSpeed;   //�����ٶ�
	unsigned short LeadScrewStroke_Max; //Ĭ��˿������г�������
}MotorParameter;


typedef struct
{
	unsigned char  MotorTimeOutFlagUP :1;    //���˶���ʱ��־
	unsigned char  MotorTimeOutFlagDwon :1;  //���˶���ʱ��־
	unsigned char  MotorRunMode :4;          //�˶�ģʽ 0:��ʱͣ���ڵ�ǰλ\
																											1:��ʱ�������λ
	unsigned char  SpeedReductionLock :1;    //�½�����
	unsigned char  StartUpFlag :1;           //������λ��־
	unsigned char  StartDwonFlag :1;         //������λ��־    	
	unsigned short SpeedUp;                  //����λ�ٶ�
	unsigned short SpeedDwon;                //����λ�ٶ�	
	signed int     OriginPosUp;              //���˶�ǰλ�ü�¼
	signed int     OriginPosDwon;            //���˶�ǰλ�ü�¼	
  signed int     CurrentPosUp;             //���˶���ǰλ��
	signed int     CurrentPosDwon;           //���˶���ǰλ��	
	unsigned int   MotorMoveCountTime;       //�˶���ʱ
	unsigned int   SpeedUpCountTime;         //���ٶȼ�ʱ
	unsigned int   SpeedDwonCountTime;       //���ټ�ʱ	
}MotorMove;

#define MOTOROUTTIME                       5000
#define MAXPOS                             8000

extern unsigned char  	SensorReturnDa[8];           //�������źŷ�������
extern unsigned char    MotorStateclock;  
extern unsigned char    MotorStateSendclock;        //�������״̬������  


extern unsigned char   SensorSig;                                         //�źŵ�ǰ״̬ 
extern unsigned char   LastSensorsignal;                                  //��һ���źż�¼ 

extern unsigned char   StartActiveReporting;                              //�����ϱ�����������־
extern unsigned char   signalreturnflag;                                  //�ϱ������־
extern unsigned char	 requestflag;																			  //�ϱ������־

extern unsigned char  MotorStateSendclock;                                //�������״̬������  
extern unsigned char  MotorStateclock;                                    //�������״̬������  
extern unsigned char  MotorStateCache;                                    //���״̬����

extern unsigned short  MotorDataSendTimeCount;                            //SPI���ͼ�ʱ���



extern MotorMove MotMove;
extern MotorParameter MrPr;
extern unsigned short  MotorDataSendTimeCount;     
signed char CanRaedMassgaAnalysis(unsigned char * dat);

signed char SensorGetMotorStatePoll(void);

void ClearMotorState(unsigned char Ud);
signed char MoveUpTask(void);

void ClearMotorState(unsigned char Ud);
#endif


