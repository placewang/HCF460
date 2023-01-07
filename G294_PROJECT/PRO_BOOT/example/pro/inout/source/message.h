#ifndef __MESSAGE_H__
#define __MESSAGE_H__
#include "bsp.h"
#include "mb_port.h"

enum
{
 CMD_SPI_GETBOARD_ALARM=0x60,//获取某个子板在线和报警状态
 CMD_SPI_GETBOARD_VER=0x61,//获取子板的版本号
 CMD_SPI_GOTO_BOOT=0x62,//设置子板跳转到BOOT
 CMD_SPI_GRADE_STS=0x63,//查询子板升级状态f
 CMD_SPI_SEND_FILELEN=0x64,//下发升级文件的大小 分四字节
 CMD_SPI_SEND_CRC=0x65,//下发CRC校验码  分两字节
};

enum
{
 CMD_ALLBACK_GETBOARDTYPE=0x01,
 CMD_ALLBACK_GETUID=0x02,



};


enum
{
 CMD_MAIN_GET_UID=0x00,
 CMD_MAIN_GET_HARDWARE=0x01,
 CMD_MAIN_GET_SOFTVER=0x02,
 CMD_MAIN_GET_VI1=0x03,
 CMD_MAIN_GET_VI2=0x04,
 CMD_MAIN_GET_INPUT=0x05,
 CMD_BOARD_CONFIG_ID=0x06,
 CMD_BOARD_GOIN_CONFIG=0x07,
};//针对主板的命令


enum
{
 CMD_SON_GET_UID=0x00,
 CMD_SON_GET_HARDWARE=0x01,
 CMD_SON_GET_SOFTVER=0x02,
 CMD_SON_GET_VI1=0x03,
 CMD_SON_GET_VCOLLECT=0x04,
 CMD_SON_GET_INPUT=0x05,
 CMD_SON_SET_OUTPUT=0x06,
 CMD_SON_SET_VHL=0x07,
 CMD_SON_SET_ALARM_IRQLINE1=0x08,//配置子板某些输入与中断线关联
 CMD_SON_SET_ALARM_REMAP=0x09,//配置子板某些输入与中断线重印射
 CMD_SON_GET_BOARDTYPE=0x0A,
};//针对子板的命令


enum
{
 CMD_MOT_SET_RESETSPEED=0x00,
 CMD_MOT_SET_RUNSPEED=0x01,
 CMD_MOT_SET_RESETDIR=0x02,
 CMD_MOT_SET_RESETMODE=0x03,
 CMD_MOT_SET_SENSORWIDTH=0x04,
 CMD_MOT_SET_RESET=0x05,
 CMD_MOT_SET_GOPSO=0x06,
 CMD_MOT_GET_CURPOS=0x07,
};//电机相关的命令


enum
{
 GETCMD=0x00,
 GETBOARDTYPE=0x01,
 GETBOARDUID=0x02,    
    
    
    
    
 CONFIGCMD=0x01,   
    
 CONFIGCMD_SETID=0x00,
 CONFIGCMD_ASK=0x01,
 CONFIGCMD_PIN_PURPOSE=0x02,
 CONFIGCMD_PIN_REMAP=0x03,
 CONFIGCMD_GETUID=0x04,
 CONFIGCMD_SETVAL=0x05,
 CONFIGCMD_ID_0XFF=0x06,
 CONFIGCMD_GETBOARDTYPE=0x07, 
    
 CHECKCMD=0x02,
 CHECKCMD_ONLINE=0x00,
 SETCMD_OUTPUT=0x01,
 CLEAN_ALARM=0x02,
};



#define CMDTYPE_ALARM  4
enum
{
 CMD_ALARM_MAIN_PINREPORT=1,
 CMD_ALARM_SON_PINREPORT=2,
 CMD_ALARM_SON_REQUESTSTOP=3,
 CMD_ALARM_MAIN_INFOR=4,
 CMD_ALARM_SON_INFOR=5,
 CMD_ALARM_CLEAN=6,
};//报警相关的命令


enum
{
 BIT_CONFIGTYPE_ERR=0,//子板配置类型不对
 BIT_UNLINK_SONBOARD=1,//子板通讯异常
};//报警相关的命令

extern uint16_t bit_alarminfor;


#define CAN_RX_LEN 150
typedef struct {
	unsigned char cmd_type;
	unsigned char cmd_data;
	unsigned char arg1_l;
	unsigned char arg1_h;
	unsigned char arg2_l;
	unsigned char arg2_h;
	unsigned char arg3_l;
	unsigned char arg3_h;
	unsigned short arg1;
	unsigned short arg2;
	unsigned short arg3;
}NEWCMDMASSAGE;

typedef struct
{
 uint8_t writecnt;
 uint8_t readcnt;
 uint8_t  buf[CAN_RX_LEN][8];      
}CAN_CMD_LIST;


#define ALARMLEN  5
typedef struct
{
 uint8_t id;
 int infor;
}CodeArg_TypeDef;

typedef struct
{
 uint8_t alarmflag;
 uint8_t writecnt;
 uint8_t readcnt;
 CodeArg_TypeDef val[ALARMLEN];
}AlarmList_TypeDef;


#define CMDLEN  100
enum
{
 UART_LEVEL0=0,
 UART_LEVEL1=1,
 UART_LEVEL2=2,
 UART_LEVELMAX=3,
};

typedef struct 
{
 uint8_t write;
 uint8_t read;    
 uint8_t addr[CMDLEN];     //从机地址
 uint8_t cmd[CMDLEN];      //从机功能码
 uint8_t datlen[CMDLEN];   //数据长度
 uint8_t data[CMDLEN][50];     
}UART_TX_LIST;

extern UART_TX_LIST uart_tx_list0[HOSTMAX];
extern UART_TX_LIST uart_tx_list1[HOSTMAX];
extern UART_TX_LIST uart_tx_list2[HOSTMAX];

extern AlarmList_TypeDef alarmlist;
extern CAN_CMD_LIST canrx_cmdlist;
extern CAN_CMD_LIST cantx_cmdlist;
extern NEWCMDMASSAGE cmdmessage;
extern AlarmList_TypeDef alarmlist;
extern unsigned short canid_upgrade;





extern void CanMessageInitlist(void);
extern void CanMessagePushlist(CAN_CMD_LIST *list,unsigned char *dat);
extern int  CanMessagePoplist(CAN_CMD_LIST *list,unsigned char *dat);
extern void CanReciveDataDeal(unsigned char *dat);
extern void Massage_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4);
extern void Massage_Send_Alert(unsigned char whatsys,unsigned int Msg, unsigned int arg1);
extern void CanReciveUpgradeDeal(unsigned char whatsys,unsigned char *dat);
extern void CanReciveComDeal(unsigned char whatsys,unsigned char *dat);
extern void DebugInforByCan(char arg1,int arg2);

extern int CanMessageSendReal22(int canid,unsigned int buf);
extern void CanReciveSyncDataDeal(unsigned char whatsys,unsigned char *dat);
extern void Massage_Back_SyncCmd(unsigned char board,unsigned int item,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4);
extern void alarmlist_init(void);
extern void Alarmlist_Pop(AlarmList_TypeDef *alarml);
extern void Alarmlist_Push(AlarmList_TypeDef *alarml,uint8_t id,int infor);
extern void Massage_BackSysConfig_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4);
extern void Command_exec_alarm(NEWCMDMASSAGE *cmd);
extern void DebugInforDisplay(char *arg1);

extern void UartMessageInitlist(void);
extern void UartMessagePushTxlist(uint8_t level,uint8_t host,uint8_t addr,uint8_t cmd,uint8_t *data,uint8_t len);
extern void UartListPop(void);
extern void InitList(void);
extern void mbh_exec(uint8_t idx,uint8_t addr,uint8_t *pframe,uint8_t len);
extern int CanMessageSendReal(int canid,unsigned char *buf);











#endif
