#include "bsp.h"
#include "hc32f46x_interrupts.h"
#include "hc32f46x_clk.h"
#include "hc32f46x_gpio.h"
#include "hc32_ddl.h"
#include "hc32f46x_efm.h"
#include "machconfig.h"
#include "mb_host.h"
#include "message.h"
#include "upgrade.h"

void ExtInt1R485_1_Callback(void);
void ExtInt1R485_2_Callback(void);
void ExtInt2R485_1_Callback(void);
void ExtInt2R485_2_Callback(void);
void GpioIrq_Init(void);

unsigned int timdelay=0,delay_opencheck=0,opencheck_flag=0;
NEWSYSCONFIG sysdef;
uint8_t config_failflag=0,configend_sendonline=0;
int configclose_delay=0,configclose_flag=0;
uint8_t  mainboardid=0,mainboardtype=0;

uint8_t  mainarry_uid[12] __attribute__((at(ADDR_UID))); 
const uint8_t boot_ver[] __attribute__((at(BOARD_VER_ADDR))) ={BOOT_VER_L,BOOT_VER_H};
const uint8_t hardware_ver2[] __attribute__((at(MAINBOARD_HARDWARE_ADDR+4*6))) ={0x94,0x02,0x00,0x00};
const uint8_t BOARD_TYPE  __attribute__((at(MAINBOARD_HARDWARE_ADDR+4*10))) =0;



const en_port_t mcu_port[N_USEDIO]=
{
 PortH,
 PortC,
 PortC,
 PortA,
 PortA,
 PortA,
 PortA,
 PortA,
 PortA,
 PortA,
 PortB,
 PortB,   
 PortB,
 PortB,
 PortB,
 PortB,   
 PortB,
 PortB,
 PortA,
 PortA,
 PortA,
 PortA,
 PortA,
 PortA,
 PortB,
 PortB,
 PortB,   
 PortB,
 PortB,
 PortB,
 PortB,    
};

const en_pin_t mcu_pin[N_USEDIO]=
{
 Pin02,
 Pin13,
 Pin14,
 Pin00,
 Pin01,    
 Pin02,
 Pin03,
 Pin04,
 Pin05,
 Pin07,
 Pin00,
 Pin01,
 Pin02,
 Pin10,
 Pin12,
 Pin13,
 Pin14,
 Pin15,
 Pin08,
 Pin09,
 Pin10,
 Pin11,
 Pin12,
 Pin15,
 Pin03,
 Pin04,
 Pin05,
 Pin06,
 Pin07,
 Pin08,
 Pin09,
};


   


void InitGpioDrive()
{
  stc_port_init_t stcPortInit;
    
  MEM_ZERO_STRUCT(stcPortInit);

  stcPortInit.enPinMode = Pin_Mode_Out;
  
  stcPortInit.enPinDrv=Pin_Drv_H;
    
  stcPortInit.enPinOType=Pin_OType_Cmos;
    
  stcPortInit.enExInt = Enable;
    
 
 stcPortInit.enPinMode = Pin_Mode_Out;    
  PORT_Init(mcu_port[IO_VCC_CONTROL], mcu_pin[IO_VCC_CONTROL], &stcPortInit);

  stcPortInit.enPinMode = Pin_Mode_In;    
  PORT_Init(mcu_port[IO_NFAULT_VCC_CURRENT], mcu_pin[IO_NFAULT_VCC_CURRENT], &stcPortInit);
    
  stcPortInit.enPinMode = Pin_Mode_Out;    
  PORT_Init(mcu_port[IO_LED], mcu_pin[IO_LED], &stcPortInit);

   stcPortInit.enPinMode = Pin_Mode_In;    
  PORT_Init(mcu_port[IO_ADC_5V], mcu_pin[IO_ADC_5V], &stcPortInit);

  stcPortInit.enPinMode = Pin_Mode_In;    
  PORT_Init(mcu_port[IO_VCC_CURRENT_AD], mcu_pin[IO_VCC_CURRENT_AD], &stcPortInit);
  
  stcPortInit.enPinMode = Pin_Mode_In;    
  PORT_Init(mcu_port[IO_VCC_IN_AD], mcu_pin[IO_VCC_IN_AD], &stcPortInit);
  
  stcPortInit.enPinMode = Pin_Mode_In;    
  PORT_Init(mcu_port[IO_ADC_12V_OUT], mcu_pin[IO_ADC_12V_OUT], &stcPortInit);
  
  stcPortInit.enPinMode = Pin_Mode_In;    
  PORT_Init(mcu_port[IO_ADC_12V_485], mcu_pin[IO_ADC_12V_485], &stcPortInit);
  
  stcPortInit.enPinMode = Pin_Mode_In;    
  PORT_Init(mcu_port[IO_ADC_VCC_LVDS], mcu_pin[IO_ADC_VCC_LVDS], &stcPortInit);
  
  stcPortInit.enPinMode = Pin_Mode_In;    
  PORT_Init(mcu_port[IO_MCU_IN0], mcu_pin[IO_MCU_IN0], &stcPortInit);
  PORT_Init(mcu_port[IO_MCU_IN1], mcu_pin[IO_MCU_IN1], &stcPortInit);
  PORT_Init(mcu_port[IO_MCU_IN2], mcu_pin[IO_MCU_IN2], &stcPortInit);
  PORT_Init(mcu_port[IO_MCU_IN3], mcu_pin[IO_MCU_IN3], &stcPortInit);
  PORT_Init(mcu_port[IO_MCU_IN4], mcu_pin[IO_MCU_IN4], &stcPortInit);
  PORT_Init(mcu_port[IO_MCU_IN5], mcu_pin[IO_MCU_IN5], &stcPortInit);
  PORT_Init(mcu_port[IO_MCU_IN6], mcu_pin[IO_MCU_IN6], &stcPortInit);
  PORT_Init(mcu_port[IO_MCU_IN7], mcu_pin[IO_MCU_IN7], &stcPortInit);
  PORT_Init(mcu_port[IO_MCU_IN8], mcu_pin[IO_MCU_IN8], &stcPortInit);
  
  stcPortInit.enPinMode = Pin_Mode_In;    
  PORT_Init(mcu_port[IO_POFF], mcu_pin[IO_POFF], &stcPortInit);

  stcPortInit.enPinMode = Pin_Mode_In;    
 /* PORT_Init(mcu_port[IO_OC_IN1], mcu_pin[IO_OC_IN1], &stcPortInit);
  PORT_Init(mcu_port[IO_OC_IN2], mcu_pin[IO_OC_IN2], &stcPortInit);
  PORT_Init(mcu_port[IO_OC_IN3], mcu_pin[IO_OC_IN3], &stcPortInit);
  PORT_Init(mcu_port[IO_OC_IN4], mcu_pin[IO_OC_IN4], &stcPortInit);
  GpioIrq_Init();*/
  
  
  stcPortInit.enPinMode = Pin_Mode_Out;    
  PORT_Init(mcu_port[IO_485DIR_1], mcu_pin[IO_485DIR_1], &stcPortInit); 
  
  stcPortInit.enPinMode = Pin_Mode_In;    
  PORT_Init(mcu_port[IO_UART3_RX], mcu_pin[IO_UART3_RX], &stcPortInit);
  stcPortInit.enPinMode = Pin_Mode_Out;    
  PORT_Init(mcu_port[IO_UART3_TX], mcu_pin[IO_UART3_TX], &stcPortInit); 
  
  stcPortInit.enPinMode = Pin_Mode_In;    
  PORT_Init(mcu_port[IO_UART4_RX], mcu_pin[IO_UART4_RX], &stcPortInit);
  stcPortInit.enPinMode = Pin_Mode_Out;    
  PORT_Init(mcu_port[IO_UART4_TX], mcu_pin[IO_UART4_TX], &stcPortInit); 
  
  stcPortInit.enPinMode = Pin_Mode_Out;    
  PORT_Init(mcu_port[IO_485DIR_2], mcu_pin[IO_485DIR_2], &stcPortInit); 
  
  stcPortInit.enPinMode = Pin_Mode_In;    
  PORT_Init(mcu_port[IO_CAN_RX], mcu_pin[IO_CAN_RX], &stcPortInit);
  stcPortInit.enPinMode = Pin_Mode_Out;    
  PORT_Init(mcu_port[IO_CAN_TX], mcu_pin[IO_CAN_TX], &stcPortInit);
  
  
}

void GpioIrq_Init(void)
{
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    stcExtiConfig.enExitCh = ExtiCh09;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    stcExtiConfig.enExtiLvl = ExIntBothEdge;
    EXINT_Init(&stcExtiConfig);

    /* Set External Int Ch input */
    stcPortInit.enExInt = Enable;
    PORT_Init(mcu_port[IO_OC_IN1], mcu_pin[IO_OC_IN1], &stcPortInit);
   

    /* Select External Int Ch */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ9;
    /* Register External Int to Vect */
    stcIrqRegiConf.enIRQn = Int010_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = ExtInt1R485_1_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    
    
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    stcExtiConfig.enExitCh = ExtiCh10;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    stcExtiConfig.enExtiLvl = ExIntBothEdge;
    EXINT_Init(&stcExtiConfig);

    /* Set External Int Ch input */
    stcPortInit.enExInt = Enable;
    PORT_Init(mcu_port[IO_OC_IN2], mcu_pin[IO_OC_IN2], &stcPortInit);
   

    /* Select External Int Ch */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ10;
    /* Register External Int to Vect */
    stcIrqRegiConf.enIRQn = Int011_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = ExtInt1R485_2_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    
    
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    stcExtiConfig.enExitCh = ExtiCh11;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    stcExtiConfig.enExtiLvl = ExIntBothEdge;
    EXINT_Init(&stcExtiConfig);

    /* Set External Int Ch input */
    stcPortInit.enExInt = Enable;
    PORT_Init(mcu_port[IO_OC_IN3], mcu_pin[IO_OC_IN3], &stcPortInit);
   

    /* Select External Int Ch */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ11;
    /* Register External Int to Vect */
    stcIrqRegiConf.enIRQn = Int012_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = ExtInt2R485_1_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    stcExtiConfig.enExitCh = ExtiCh12;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    stcExtiConfig.enExtiLvl = ExIntBothEdge;
    EXINT_Init(&stcExtiConfig);

    /* Set External Int Ch input */
    stcPortInit.enExInt = Enable;
    PORT_Init(mcu_port[IO_OC_IN3], mcu_pin[IO_OC_IN3], &stcPortInit);
   

    /* Select External Int Ch */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ12;
    /* Register External Int to Vect */
    stcIrqRegiConf.enIRQn = Int013_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = ExtInt2R485_2_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

void Rs485_DirContrl(int idx,int sts)
{
 if(idx==HOST1)
  {
      sts ? PORT_ResetBits(mcu_port[IO_485DIR_1], mcu_pin[IO_485DIR_1]):PORT_SetBits(mcu_port[IO_485DIR_1], mcu_pin[IO_485DIR_1]); 
  }
 else if(idx==HOST2)
  {
      sts ? PORT_ResetBits(mcu_port[IO_485DIR_2], mcu_pin[IO_485DIR_2]):PORT_SetBits(mcu_port[IO_485DIR_2], mcu_pin[IO_485DIR_2]);
  }

}

void Init_Flash()
{
    
    EFM_Unlock();

    EFM_FlashCmd(Enable);
    
    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY))
    {
      ;
    }
   mainboardid=*(__IO uint32_t *)ADDR_BOARD_ID;
   mainboardtype=*(__IO uint32_t *)ADDR_BOARDTYPE;
   canid_upgrade_rx=NEW_CANID_COM_RX;
   canid_upgrade_tx=NEW_CANID_COM_TX;
}


void EFM_EraseProgram(uint32_t u32Addr,uint32_t dat)
{
    EFM_Unlock();
    EFM_SectorErase(u32Addr);
    EFM_SingleProgram(u32Addr,dat);
    EFM_Lock();
}

uint32_t EFM_ReadByAddr(uint32_t u32Addr)
{
   return *((__IO uint32_t *)u32Addr);
}



void Timer0B_CallBack(void)//1ms
{
 static int t_cnt=0;
 t_cnt++;
 timdelay++; 
 timcan++;
 delay_opencheck++;  
 configclose_delay++;  
 timdelay_upgrade++;    
 if(t_cnt>=500)
 {
   t_cnt=0;
   LED0_TOGGLE(); 
  }
}


void Init_Timer0()
{
    stc_tim0_base_init_t stcTimerCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_clk_freq_t stcClkTmp;
    MEM_ZERO_STRUCT(stcTimerCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcClkTmp);
    
    CLK_GetClockFreq(&stcClkTmp); 
   
    
    ENABLE_TMR0();
    stcTimerCfg.Tim0_CounterMode = Tim0_Sync;
    stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;//21M
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv4;//5.25M
    stcTimerCfg.Tim0_CmpValue = (uint16_t)(stcClkTmp.pclk1Freq/4/((1000000ul/(TIME0_VAL)) - 1ul));//200us
    TIMER0_BaseInit(TMR_UNIT,Tim0_ChannelB,&stcTimerCfg);

    /* Enable channel B interrupt */
    TIMER0_IntCmd(TMR_UNIT,Tim0_ChannelB,Enable);
    /* Register TMR_INI_GCMB Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = Int002_IRQn;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = TMR_INI_GCMB;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &Timer0B_CallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    TIMER0_Cmd(TMR_UNIT,Tim0_ChannelB,Enable);

}



void Can_Config()
{
    stc_can_init_config_t   stcCanInitCfg;
    stc_can_filter_t        stcFilter;
    stc_irq_regi_conf_t     stcIrqRegiConf;
    stc_pwc_ram_cfg_t       stcRamCfg;
    
    MEM_ZERO_STRUCT(stcRamCfg);
    MEM_ZERO_STRUCT(stcCanInitCfg);
    MEM_ZERO_STRUCT(stcFilter);
    MEM_ZERO_STRUCT(stcRxFrame);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    
   stcRamCfg.enRamOpMd = HighSpeedMd;
    stcRamCfg.enCan = DynamicCtl;
    PWC_RamCfg(&stcRamCfg);
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_CAN, Enable);

    //<<CAN GPIO config
    PORT_SetFunc(PortB, Pin08, Func_Can1_Rx, Disable);
    PORT_SetFunc(PortB, Pin09, Func_Can1_Tx, Disable);
    PORT_ResetBits(PortD, Pin15);
    PORT_OE(PortD, Pin15, Enable);

    //<<Can bit time config
    stcCanInitCfg.stcCanBt.PRESC = 1u-1u;
    stcCanInitCfg.stcCanBt.SEG_1 = 6u-2u;//5  后边的主控会bus off报警
    stcCanInitCfg.stcCanBt.SEG_2 = 2u-1u;//3
    stcCanInitCfg.stcCanBt.SJW   = 1u-1u;//3

    stcCanInitCfg.stcWarningLimit.CanErrorWarningLimitVal = 10u;
    stcCanInitCfg.stcWarningLimit.CanWarningLimitVal = 16u-1u;

    stcCanInitCfg.enCanRxBufAll  = CanRxNormal;
    stcCanInitCfg.enCanRxBufMode = CanRxBufNotStored;
    stcCanInitCfg.enCanSAck      = CanSelfAckEnable;
    stcCanInitCfg.enCanSTBMode   = CanSTBFifoMode;

    CAN_Init(&stcCanInitCfg);

    //<<Can filter config
    stcFilter.enAcfFormat = CanAllFrames;
    stcFilter.enFilterSel = CanFilterSel1;
    stcFilter.u32CODE     = 0x00000000ul;
    stcFilter.u32MASK     = 0x1FFFFFFFul;
    CAN_FilterConfig(&stcFilter, Enable);
       
    CAN_IrqCmd(CanRxIrqEn, Enable);
     
    stcIrqRegiConf.enIRQn = Int000_IRQn;
    stcIrqRegiConf.enIntSrc = INT_CAN_INT;
    stcIrqRegiConf.pfnCallback = &CAN_RxIrqCallBack;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_00);//12-17 DDL_IRQ_PRIORITY_DEFAULT  t0  0
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(Int000_IRQn);
}




void Uart_Config()
{

}


void InitConfigDrive()
{
  Init_Flash();
    
  Init_Timer0();

  Can_Config();
    
  mbh_init(115200);
}

MACHPARA mach;
stc_can_txframe_t       stcTxFrame; 
stc_can_rxframe_t       stcRxFrame;
UPGRADE_INF upgradeinf;
TANZHENPARA tzcheck[2];
uint8_t curboard_uid[12]={0,0,0,0,0,0,0,0,0,0,0,0};

    
        
void CAN_RxIrqCallBack(void);

unsigned int timcan=0;


void Timer0B_CallBack(void);




void CAN_RxIrqCallBack(void)
{
  if(true == CAN_IrqFlgGet(CanRxIrqFlg))
    {
      CAN_IrqFlgClr(CanRxIrqFlg);
        
      CAN_Receive(&stcRxFrame);
     
      if((stcRxFrame.StdID==NEW_CANID_COM_RX)||(stcRxFrame.StdID==NEW_CANID_COM_RX+mainboardid))
      CanMessagePushlist(&canrx_cmdlist,stcRxFrame.Data);             
     }
}



#define JUMPTOAPPBYLOOP



void CalculatePeriphSpiByType(unsigned char *bit_nspi,int type)
{
 
}


int JudgePeriphIsThisType(int n_periph,int type,int boardid)
{
 return 0;
}


int JudgePeriphBytype(int n_periph,int type)
{

 return 0;
}
 
void UpgradeSuccessGoToApp()
{
}
 

void UpgradeSendDataLoop()
{
 /* unsigned char datbuf[8],data_rx[8]={0,0,0,0,0,0,0,0},buf[8]={0,0,0,0,0,0,0,0};  
 char dis[8]; 
    
  if(upgradeinf.sts!=0x8F)
    return;
  
  
  switch(upgradeinf.step)
  {
   case 1: //升级检索  
      if((sysconfig.periph[upgradeinf.n_periphcnt].enable==1)&&(sysconfig.periph[upgradeinf.n_periphcnt].type==upgradeinf.boardtype))
        {
         upgradeinf.step=2;
         sprintf(dis,"spi:%d-",sysconfig.periph[upgradeinf.n_periphcnt].n_spi);
         DebugInforDisplay(dis);
         sprintf(dis,"num:%d-",sysconfig.periph[upgradeinf.n_periphcnt].id);
         DebugInforDisplay(dis);
        
         upgradeinf.pbuf=(unsigned char *)ADDR_APP_BACKUP;                          
        }
      else
       {
         upgradeinf.n_periphcnt++;
         if(upgradeinf.n_periphcnt>=(sysconfig.sysnum*PERIPHAS_EVERYSYS))
          {
           UpgradeSuccessGoToApp();
           break;
          }
       }
       break;       
      
   case 2://升级跳转 发送CRC
      {
                   
      if(upgradeinf.type==0)//shengjiAPP
      {
        DebugInforDisplay("跳转至BOOT");  
        DebugInforDisplay("\r\n");  
       // SpiSendCmdDataAtOnce(SPI_COMCMD_GOINBOOT,upgradeinf.n_periphcnt,NULL,NULL,0,0); 
      }
       DebugInforDisplay("开始升级");
       
       upgradeinf.step=3;
       upgradeinf.steptimcnt=0;
       Ddl_Delay1ms(1000);
      
      #ifdef NEWBOOT_APP
      // SpiSendCmdDataAtOnce(SPI_COMCMD_SONBOARD_ATWHERE,upgradeinf.n_periphcnt,buf,data_rx,1,5);
      
       if(upgradeinf.type==(data_rx[4]&0x01))
       {
         DebugInforDisplay("升级类型");  
         DebugInforDisplay("出错"); 
         DebugInforDisplay("子板不在"); 
         DebugInforDisplay("区域"); 
         upgradeinf.sts=0;
         break;
       }
      #endif      
       datbuf[0]=upgradeinf.file_len&0xFF;
       datbuf[1]=(upgradeinf.file_len>>8)&0xFF;
       datbuf[2]=(upgradeinf.file_len>>16)&0xFF;
       datbuf[3]=(upgradeinf.file_len>>24)&0xFF;
      // SpiSendCmdDataAtOnce(SPI_COMCMD_UPFILELEN,upgradeinf.n_periphcnt,datbuf,NULL,4,0); 

       datbuf[0]=upgradeinf.crc&0xFF;
       datbuf[1]=(upgradeinf.crc>>8)&0xFF; 
     //  SpiSendCmdDataAtOnce(SPI_COMCMD_SENDCRC,upgradeinf.n_periphcnt,datbuf,NULL,2,0);
       upgradeinf.timescnt=0;
       upgradeinf.step=3;
      }
    break;
   
   case 3://发送升级包   
       if(upgradeinf.timescnt==(upgradeinf.sendtimes-1)) 
        {  
        if(upgradeinf.lastlen==0)
          {            
       //   SpiSendCmdDataAtOnce(SPI_COMCMD_UPGRADE_LASTHEAD,upgradeinf.n_periphcnt,upgradeinf.pbuf,NULL,8,0);  
          upgradeinf.pbuf+=8;            
          }            
        else
          {
        //  SpiSendCmdDataAtOnce(SPI_COMCMD_UPGRADE_LASTHEAD,upgradeinf.n_periphcnt,upgradeinf.pbuf,NULL,upgradeinf.lastlen,0); 
          upgradeinf.pbuf+=upgradeinf.lastlen;            
          }            
        }
       else
        {
        // SpiSendCmdDataAtOnce(SPI_COMCMD_UPGRADE_HEAD,upgradeinf.n_periphcnt,upgradeinf.pbuf,NULL,8,0);  

         upgradeinf.pbuf+=8;
        }
        upgradeinf.timescnt++;
      if(((upgradeinf.timescnt*8)%512)==0)
        {
         Ddl_Delay1ms(120);
         DebugInforDisplay(".");    
        }
       if(upgradeinf.timescnt>=upgradeinf.sendtimes)
         upgradeinf.step=4;
       break;
       
       
   case 4://询问升级是否成功
       Ddl_Delay1ms(200);
     //  SpiSendCmdDataAtOnce(SPI_COMCMD_UPGRADE_ISFINISH,upgradeinf.n_periphcnt,buf,data_rx,1,2);
      if((data_rx[0]&0x02)==0x02)
       {
         sprintf(dis,"spi:%d-",sysconfig.periph[upgradeinf.n_periphcnt].n_spi);
         DebugInforDisplay(dis);
         sprintf(dis,"num:%d-",sysconfig.periph[upgradeinf.n_periphcnt].id);
         DebugInforDisplay(dis);
         DebugInforDisplay("升级成功");
         DebugInforDisplay("\r\n");         
       }           
      else
       {
         upgradeinf.sts=0;
         sprintf(dis,"spi:%d-",sysconfig.periph[upgradeinf.n_periphcnt].n_spi);
         DebugInforDisplay(dis);
         sprintf(dis,"num:%d-",sysconfig.periph[upgradeinf.n_periphcnt].id);
         DebugInforDisplay(dis);
         DebugInforDisplay("升级失败");
         DebugInforDisplay("\r\n");
       }   
       upgradeinf.step=5;
      break;

   case 5://下一个
         upgradeinf.n_periphcnt++;
       if(upgradeinf.n_periphcnt>=(sysconfig.sysnum*PERIPHAS_EVERYSYS))
          {
          UpgradeSuccessGoToApp();           
          }
       else
          upgradeinf.step=1;           
      break;
 
    } */ 
}




void machpara_init()
{
 memset(&mach,0,sizeof(mach));
}

int testio=0;
uint8_t Oc_In_Sts()
{
  uint8_t bit_sts=0xFF;

 if((PORT_GetBit(mcu_port[IO_OC_IN1], mcu_pin[IO_OC_IN1])==0)||(PORT_GetBit(mcu_port[IO_OC_IN3], mcu_pin[IO_OC_IN3])==0))
    bit_sts &=0xFE;
 if((PORT_GetBit(mcu_port[IO_OC_IN2], mcu_pin[IO_OC_IN2])==0)||(PORT_GetBit(mcu_port[IO_OC_IN4], mcu_pin[IO_OC_IN4])==0))
    bit_sts &=0xFD;   
  return bit_sts;
}

void ExtInt1R485_1_Callback()//IN1输入中断
{
   Massage_Send_4halfword((0x01<<8)|(0x04),0,0,Oc_In_Sts()); 
    
 // Massage_Send_4halfword((0x03<<8)|(0x04),0,0,0);  //请求停机
}

void ExtInt1R485_2_Callback()//IN2输入中断
{
   Massage_Send_4halfword((0x01<<8)|(0x04),0,0,Oc_In_Sts()); 
 // Massage_Send_4halfword((0x03<<8)|(0x04),0,0,0);  //请求停机
}

void ExtInt2R485_1_Callback()//IN3输入中断
{
   Massage_Send_4halfword((0x01<<8)|(0x04),0,0,Oc_In_Sts()); 
 // Massage_Send_4halfword((0x03<<8)|(0x04),0,0,0);  //请求停机
}

void ExtInt2R485_2_Callback()//IN4输入中断
{
   Massage_Send_4halfword((0x01<<8)|(0x04),0,0,Oc_In_Sts()); 
 // Massage_Send_4halfword((0x03<<8)|(0x04),0,0,0);  //请求停机
}


//////////////////配置ID相关接口////////////////////////
int LookUpHostById(uint8_t id)
{
 uint8_t i;
 for(i=0;i<N_PERIPH;i++)
  {
   if(sysdef.periph[i].id==id)
    {
     return sysdef.periph[i].whathost;
    } 
  }
  //报警  无此id
  return -1;
}


int LookUpNumById(uint8_t id)
{
 uint8_t i;
 for(i=0;i<N_PERIPH;i++)
  {
   if(sysdef.periph[i].id==id)
    {
     return i;
    } 
  }
  //报警  无此id
  return -1;
}

//////////////////配置ID相关接口////////////////////////

void CheckBoardOnline()
{
unsigned char buff_tx[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
 uint8_t i;

 if(configclose_flag)//配置完成后过500ms  才打开通讯
 {
  if(configclose_delay>500)
  {
    configclose_delay=0;
    configclose_flag=0;
    configend_sendonline=1;        
  }
 }

 if(configend_sendonline==0)
 {
   return;
 }

 
 buff_tx[0]=CHECKCMD;
 buff_tx[1]=CHECKCMD_ONLINE;

 for(i=0;i<N_PERIPH;i++)
  {
   if(sysdef.periph[i].id!=0)
    {
      buff_tx[2]=sysdef.periph[i].id;
      UartMessagePushTxlist(UART_LEVEL2,sysdef.periph[i].whathost,sysdef.periph[i].id,FUNC_02,buff_tx,3);      
    } 
  }
}


void SystemClock_DeInit(void)
{
    uint32_t u32Timeout = 0u;

    /* Unlock CMU. */
    ENABLE_CLOCK_REG_WRITE();

    /* Close fcg0~fcg3. */
    M4_MSTP->FCG0 = 0xFFFFFAEE;
    M4_MSTP->FCG1 = 0xFFFFFFFF;
    M4_MSTP->FCG2 = 0xFFFFFFFF;
    M4_MSTP->FCG3 = 0xFFFFFFFF;

    /* Wait stable after close fcg. */
    u32Timeout = CLK_STABLE_TIME;
    while (u32Timeout--);

    M4_SYSREG->CMU_CKSWR = SYSCLK_SEL_MRC;

    u32Timeout = CLK_STABLE_TIME;
    while (u32Timeout--);

    /* Set CMU registers to default value. */
#if (SYSTEM_CLOCK_SOURCE == CLK_SOURCE_XTAL)
    M4_SYSREG->CMU_XTALCFGR = (uint8_t)0x00;
    M4_SYSREG->CMU_XTALCR   = (uint8_t)0x01;
#else
    M4_SYSREG->CMU_HRCCR    = (uint8_t)0x01;
#endif // #if (SYSTEM_CLOCK_SOURCE == CLK_SOURCE_XTAL)
    M4_SYSREG->CMU_PLLCFGR  = (uint32_t)0x11101300;
    M4_SYSREG->CMU_PLLCR    = (uint8_t)0x01;
    M4_SYSREG->CMU_SCFGR    = (uint32_t)0x00;

    u32Timeout = CLK_STABLE_TIME;
    while (u32Timeout--);

 //   EFM_SetWaitCycle(0x0u);

    u32Timeout = CLK_STABLE_TIME;
    while (u32Timeout--);

    /* Lock CMU. */
    DISABLE_CLOCK_REG_WRITE();
}

void Timer_DeInit(void)
{
    stc_tim0_base_init_t stcTimerCfg;
	MEM_ZERO_STRUCT(stcTimerCfg);
    TIMER0_BaseInit(M4_TMR01, Tim0_ChannelB, &stcTimerCfg);
   // enIrqResign(Int002_IRQn);
    TIMER0_Cmd(M4_TMR01, Tim0_ChannelB, Disable);
    M4_MSTP->FCG2_f.TIMER0_1 = Set;   
}



