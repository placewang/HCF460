#ifndef __BSP_H__
#define __BSP_H__
#include "hc32f46x_gpio.h"
#include "hc32f46x_pwc.h"
#include "hc32f46x_timer0.h"
#include "hc32f46x_spi.h"
#include "hc32f46x_can.h"
#include "hc32f46x_interrupts.h"
#include "hc32f46x_timera.h"
#include "mb_host.h"
#include "hc32f46x_usart.h"

#define BOOT_VER_H   0x01
#define BOOT_VER_L   0x00


#define MAINBOARD_HARDWARE_ADDR (0x20000-0x4000) 
#define BOARD_VER_ADDR          (0x20000-0x2000) 
#define APP_RUN_ADDRESS         (0x20000)
#define ADDR_BOARDTYPE          (0x40000-0x2000)         //��150K��ʼ 
#define ADDR_UPGRADE_START      (0x40000-0x4000)  //��¼������ʼ��־
#define ADDR_BOARD_ID           (0x40000-0x6000)  //����id 
#define BOARD_VER_APP           (0x40000-0x8000)  //����APP�汾 



#define SONBOARD_HARDWARE_ADDR   0x2A00  //�Ӱ�Ӳ���汾�洢��ַ
#define APP_SONBOARD_INFO         (0x7600-0x3000)
#define SRAM_BASE               ((uint32_t)0x1FFF8000) 
#define RAM_SIZE                0x2F000ul
#define ADDR_UID    0x40010450    //����Ψһ����


enum
{
 IO_VCC_CONTROL=0,
 IO_NFAULT_VCC_CURRENT,
 IO_LED,
 IO_ADC_5V,
 IO_VCC_CURRENT_AD,
 IO_VCC_IN_AD,
 IO_ADC_12V_OUT,
 IO_ADC_12V_485,
 IO_ADC_VCC_LVDS,
 IO_MCU_IN0,
 IO_MCU_IN1,
 IO_MCU_IN2,
 IO_MCU_IN3,
 IO_MCU_IN4,
 IO_MCU_IN5,
 IO_MCU_IN6,
 IO_MCU_IN7,
 IO_MCU_IN8,
 IO_POFF,
 IO_OC_IN1,
 IO_OC_IN2,
 IO_OC_IN3,
 IO_OC_IN4,
 IO_485DIR_1,
 IO_UART3_RX,
 IO_UART3_TX,
 IO_UART4_RX,
 IO_UART4_TX,
 IO_485DIR_2,
 IO_CAN_RX,
 IO_CAN_TX,
 
  N_USEDIO,
};

enum
{
 UART_CH3=0,
 UART_CH4=1,
};

enum
{
 SENDSTS=0,
 RECIVESTS=1,
};

enum
{
  FUNC_01_ONLYSEND=0x01,//������01  ������   ��������  
  FUNC_02=0x02,//������02  ��ظ�

};

#define LED0_TOGGLE()                   (PORT_Toggle(mcu_port[IO_LED], mcu_pin[IO_LED]))
#define LED0_OFF()                      (PORT_SetBits(mcu_port[IO_LED], mcu_pin[IO_LED]))
#define LED0_ON()                       (PORT_ResetBits(mcu_port[IO_LED], mcu_pin[IO_LED]))
#define TMR_UNIT            (M4_TMR02)
#define TMR_INI_GCMA        (INT_TMR02_GCMA)
#define TMR_INI_GCMB        (INT_TMR02_GCMB)
#define ENABLE_TMR0()      (PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02, Enable))

#define ENABLE_CLOCK_REG_WRITE()            M4_SYSREG->PWR_FPRC = 0xa501
#define DISABLE_CLOCK_REG_WRITE()           M4_SYSREG->PWR_FPRC = 0xa500
#define SYSCLK_SEL_MRC      ((uint8_t)0x1)
#define CLK_STABLE_TIME     (0x1000ul)
#define DISABLE_TMR0()        (PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02, Disable))





#define CMD_UPGRADE_START	0xA050AA00
#define CMD_UPGRADE_RCVDATA	0xA050AA55
#define CMD_UPGRADE_BURN	0xA0505A5A
#define CMD_UPGRADE_SUCCESS	0xA05055AA

enum 
{ 
 TIME0_VAL=1000,//���嶨ʱ��0��us
 

 NEW_CANID_COM_RX=0x01,
 NEW_CANID_COM_TX=0x100,      
};


enum
{
 E786_INPUT_BOARD=0,
 E787_INOUT_BOARD=1,
 E788_ADCIN_BOARD=2,

};

 
#define N_PERIPH  200
typedef struct 
 {
  uint8_t whathost; 
  uint8_t id;
  uint8_t type;
  uint8_t input_sts;     
 }_NEWCONFIG_INFOR;
 
 
typedef struct 
 {
  uint8_t cnt;//����485�����������     
 _NEWCONFIG_INFOR periph[N_PERIPH];	
}NEWSYSCONFIG;
 
extern NEWSYSCONFIG sysdef;

typedef struct 
{
 int bitalarmflag; 
 int delayclearalarm;
 int clearalarmcnt;
	
}MACHPARA;

extern MACHPARA mach;



typedef struct 
{
 unsigned char isopen; //�����Ƿ��  0 �ر�   1��
 unsigned int checkdelay;	
 unsigned int checkcnt;
}TANZHENPARA;

extern TANZHENPARA tzcheck[2];//0��  1ǰ��



typedef struct
{
 unsigned char type;
 unsigned char n_periphcnt;
 unsigned char step;
 unsigned short crc;
 int steptimcnt;
 int sts;
 int file_len;
 int boardtype;
 int timescnt;
 int sendtimes;
 unsigned char lastlen;
 unsigned char *pbuf;

}UPGRADE_INF;

extern UPGRADE_INF upgradeinf;

extern unsigned int timdelay,timcan,delay_opencheck,opencheck_flag;
extern stc_can_txframe_t       stcTxFrame;
extern uint8_t curboard_uid[12];
extern stc_can_rxframe_t       stcRxFrame;
extern const en_port_t mcu_port[N_USEDIO];
extern const en_pin_t mcu_pin[N_USEDIO];
extern uint8_t config_failflag,configend_sendonline;
extern int configclose_delay,configclose_flag;
extern uint8_t  mainboardid,mainboardtype;




enum//�Ӱ�Ĺ�������
{
 SPI_COMCMD_GETSTSONLING=0x60,//
 SPI_COMCMD_GETVER=0x61,//��ȡĳ���Ӱ�İ汾
 SPI_COMCMD_GOINBOOT=0x62,//�����Ӱ���תBOOT����
 SPI_COMCMD_ISUPGRADED=0x63,//��ѯ�Ӱ��Ƿ��������
 SPI_COMCMD_UPFILELEN=0x64,//�����ֽڷ��������ļ���С
 SPI_COMCMD_SENDCRC=0x65,//�����ֽڷ���CRCУ����
 SPI_COMCMD_INFORCEUPGRADE=0x66,//����ǿ�������׶�
 SPI_COMCMD_SETNUM=0x67,
 SPI_COMCMD_CHECKNUM=0x68,//���Ӱ���  ��Ŵ�0��ʼ 
 SPI_COMCMD_CLEARALARM=0x69,  //�������
 SPI_COMCMD_GOAPP=0x6A,  //�Ӱ���ת��APP
 SPI_COMCMD_GOINFILE_RECIVE=0x6B,//�����ļ�����ģʽ
 SPI_COMCMD_GETHARDWARE=0x6C,//��ȡӲ���汾
 SPI_COMCMD_BOOTVER=0x6D,//��ȡBOOT�汾
 SPI_COMCMD_APPVER=0x6E,//��ȡAPP�汾
 SPI_COMCMD_GETSENSORALARM=0x6F,//3�ֽڻ�ȡ����������״̬
 SPI_COMCMD_UPGRADE_HEAD=0x70,//�����ļ���ͷ
 SPI_COMCMD_UPGRADE_LASTHEAD=0x71,//�����������һ������
 SPI_COMCMD_UPGRADE_ISFINISH=0x72,//ѯ�������Ƿ�ɹ�
 SPI_COMCMD_SONBOARD_ATWHERE=0x73,//ѯ���Ӱ崦��APP����BOOT��
 SPI_COMCMD_GETUID=0x74,//��ȡ����UID
 SPI_COMCMD_CLEAN_ID=0x75,//����Ӱ�ID
 SPI_COMCMD_SET_ID=0x76,//�����Ӱ�ID
};




extern void EFM_EraseProgram(uint32_t u32Addr,uint32_t dat);
extern uint32_t EFM_ReadByAddr(uint32_t u32Addr);
extern void CAN_RxIrqCallBack(void);
extern void UpgradeSendDataLoop(void);
extern void InitSysSonboardToApp(void);
extern int JudgePeriphIsThisType(int n_periph,int type,int boardid);
extern int JudgePeriphBytype(int n_periph,int type);
extern void SysGetBoardUid(void);  
extern void SysCleanBoardUid(void);     
extern void SysSetBoardUid(uint8_t num);  
extern void Init_Timer0(void);
extern void InitGpioDrive(void);
extern void InitConfigDrive(void);
extern void Rs485_DirContrl(int idx,int sts);
extern int LookUpHostById(uint8_t id);
extern int LookUpNumById(uint8_t id);
extern void CheckBoardOnline(void);
extern void SystemClock_DeInit(void);
extern void Timer_DeInit(void);
#endif


