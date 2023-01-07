#ifndef _LOGIC_LAY_H
#define _LOGIC_LAY_H
#include "message.h"
#include "board_g294.h"

#define ADDR_UID                 0x40010450       //����Ψһ����
#define MAINBOARD_HARDWARE_ADDR (0x20000-0x4000) 
#define BOARD_VER_ADDR          (0x20000-0x2000) 
#define APP_RUN_ADDRESS         (0x20000)
#define ADDR_BOARDTYPE          (0x40000-0x2000)  //��150K��ʼ 
#define ADDR_UPGRADE_START      (0x40000-0x4000)  //��¼������ʼ��־
#define ADDR_BOARD_ID           (0x40000-0x6000)  //����id 
#define BOARD_VER_APP           (0x40000-0x8000)  //����APP�汾 
#define SRAM_BASE               ((uint32_t)0x1FFF8000) 
#define RAM_SIZE                0x2F000ul


#define ALARM_INFO_CAN_CMD 0xD6


//ÿ������8��
typedef struct
{
 uint8_t left_ngroup;  //�����������
 uint8_t right_ngroup; //�����ҵ�����
    
}_VALVE_CONFIG;

extern _VALVE_CONFIG valveconfig;


#define N_PERIPH  200
typedef struct 
 {
  uint8_t has;
  uint8_t whathost; 
  uint8_t id;
  int8_t type;
  uint8_t input_sts;
  uint8_t upgradesuccess_flag;     
}_NEWCONFIG_INFOR;
 
typedef struct 
 {
  uint8_t cnt;//����485�����������     
 _NEWCONFIG_INFOR periph[N_PERIPH];	
}NEWSYSCONFIG;
extern NEWSYSCONFIG sysdef;


typedef struct
{
 uint32_t curtype;
 uint8_t  app_or_boot;
 uint8_t  step;
 uint32_t delays;

}_UPGRADE;

extern _UPGRADE boardupgrade;


enum
{
  //UART��������
 CMDUART_COM_CONFIGPARA=0x00,
 CMDUART_COM_SETUID=0x01,
 CMDUART_COM_CHECKCONFIG=0x02,
 CMDUART_COM_CLEANALARM=0x03,
 CMDUART_COM_JUMTOAPPBY_BOAEDTYPE=0x04,
 CMDUART_COM_CLEANAPP_CODE=0x05,
 CMDUART_COM_UPGDATA=0x06,
 CMDUART_COM_UPGEND=0x07,
 CMDUART_COM_UPGCHECK=0x08,
 CMDUART_COM_SETID_ONLY=0x09,
 CMDUART_COM_GET_THISALARM_SIZE=0x0A,
 CMDUART_COM_GET_ALARMINFO=0x0B,
};


enum
{
 //���в���
 P_BOARD_NUM=0,
 P_BOARDTYPE=1,
 P_UID1=2,
 P_UID2=3,
 P_UID3=4,
 P_VHARDWARE=5,
 P_VSOFTWARE_BOOT=6,
 P_VSOFTWARE_APP=7,
    
 //���˽�в���
 P_MOT_D_SPEED=0x0F,  //������ٶ��ٶ�
 P_MOT_DIR_1=0x10,    //�������
 P_MOT_CUR_1=0x11,    //�������
 P_MOT_SUB_1=0x12,    //���ϸ��
 P_MOT_POS_1=0x13,    //���λ��
 P_MOT_ZWREMAP_1=0x14,//�����λ��λӡ��
 P_MOT_SENSOR_1=0x15, //������״̬
 P_MOT_ZWIDTH_1=0x16, //�����λ���
 P_MOT_WWIDTH_1=0x17, //�����λ���   
 P_MOT_VRUN_1=0x18,   //��������ٶ�
 P_MOT_VRESET_1=0x19, //�����λ�ٶ�
 P_MOT_SRESET_1=0x1A,  //�����λ�г�
 P_MOT_RUNMODE_1=0x1B, //�����λ��ģʽ   
 P_MOT_RUN_GOZEROMODE_1=0x1C,//�������ģʽ
 P_MOT_RUN_SENSORMODE_1=0x1D,//�����λ�ô������Ƿ�������  
 P_MOT_RESETMODE_1=0x1E,//�����λģʽ
 P_MOT_ISFAST_1=0x1F,   //���������г�ģʽ
 P_MOT_SRESETIN_MAX_1=0x20,//�����λ�߽�������г�
 P_MOT_SRESETOUT_MAX_1=0x21,//�����λ���ߵ�����г�
 P_MOT_RESETOUT_CHECKEN_1=0x22,
 p_MOT_V1 =0x23,
 p_MOT_A1 = 0x24,
 p_MOT_AMAX = 0x25, 
 p_MOT_DMAX = 0x26,
 p_MOT_D1 = 0x27,
 p_MOT_VSTOP = 0x28,
 p_MOT_ACC_TIME =0x29,
 P_VALVE_LEFTCONFIG=0x30,
 P_VALVE_RIGHTCONFIG=0x31,
 
 
 P_INPUT_STS=0x40,
};
enum 
{
	 UP_LIMIT_SIGNAL =0 ,
	 DOWN_LIMIT_SIGNAL =1 ,
};
 enum
 {
  ALARM_SPI1_ERR=1,
  ALARM_SPI2_ERR=2,
  ALARM_ALARM8803_ERR=3,  //8803��������
  ALARM_GOIN_ERR=4,//��λ�߽��������껹û��Ӧ��������
 };
 

enum
{
	 LIFT_STOP =0,
	 LIFT_RUN_UP ,
	 LIFT_RUN_DOWN,
	 LIFT_RUN_A ,//��������ģʽ
	 LIFT_RUN_A_DOWN, //�½�����ģʽ
	 LIFT_RUN_D, //����ģʽ
	 LIFT_PAUSE,
};
enum
{
	 LIFT_RUN_IDLE_FLAG = 0,
	 LIFT_RUN_UP_FLAG,
	 LIFT_RUN_DOWN_FLAG,
};


#define PARA_CNT 0x7

enum
{
 //���幫�в���
 P_MAIN_BOARD_NUM=0,
 P_MAIN_BOARDTYPE=1,
 P_MAIN_UID1=2,
 P_MAIN_UID2=3,
 P_MAIN_UID3=4,
 P_MAIN_VHARDWARE=5,
 P_MAIN_VSOFTWARE_BOOT=6,
 P_MAIN_VSOFTWARE_APP=7,
};

enum
{
 P_CANSET=1,
 P_CANNOTSET=0,
};


//��Э��

#define PARA_SETORGET  0xD2
#define OTHER_CAN_CMD  0xD4



enum
{
 OTHER_CAN_CMD_SONBOARD_JUMPAPP=0x02,
 OTHER_CAN_CMD_INSET_UID=0x03,
 OTHER_CAN_CMD_SEND_UID1=0x04,
 OTHER_CAN_CMD_SEND_UID2=0x05,
 OTHER_CAN_CMD_SEND_UID3=0x06,
};



#define N_PARA 50
typedef struct 
{
 uint8_t  boardid;  //id
 uint8_t  character_cmdtype;//���λ��������������� 0:����������  1��������������
 uint8_t  cmdnum;//
 uint8_t  parameter[4]; 
}P_PERPHA;

#define ALARM_MAX 17
typedef struct
{
 uint8_t   index;
 uint8_t bytes_ch;//���ı������ֽ���
 uint8_t bytes_en;//Ӣ�ı������ֽ���
 uint8_t    info_ch[50];
 uint8_t    info_en[50];
}_ALARMCONTENT;


extern uint8_t lift_mot_state;
extern _ALARMCONTENT alarmcontent[ALARM_MAX];
extern uint8_t mot_r_limit  ;
extern uint32_t time_led;
extern uint32_t time_valve;
extern uint32_t read_status_cnt  ;	
extern P_PERPHA perpha_para[N_PARA];
extern void CheckBoardOnline(void);
extern void SonBoardGoBootByType(int upgradetype);
extern void mbh_exec(uint8_t idx,uint8_t addr,uint8_t *pframe,uint8_t len);
extern void Init_ParaList(void);
extern void CAN_RxIrqCallBack(void);  
extern void Tim0_Callback(void);
extern void MotResetLoop(void);
extern void ValveDriverChangeBit(uint8_t nspi,uint8_t *TxData,uint8_t len);
extern void InitDefaultValveConfig(void);
extern void ValveDriverByNum(uint16_t num,uint8_t sts);
extern void Init_GetBoardInfo(void);
extern void AlarmInfoInit(void);
extern void Init_RegisterSpiErr(void);
extern void lift_mot_run_loop(void) ;
extern void CanSendSignal(void);

#endif


















