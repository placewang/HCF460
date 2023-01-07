#include "board_g294.h"


_MAINBOARD_INFO boardinfo;
uint32_t timcan=0;
M4_SPI_TypeDef* whatspi_use[NMAXSPI]={M4_SPI1,M4_SPI2,M4_SPI3};
const en_port_t g294mcu_port[N_USED_MAX]=
{
 PortH,
 PortC,
 PortC,
 PortC,
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

const en_pin_t g294mcu_pin[N_USED_MAX]=
{
 Pin02,
 Pin13,
 Pin14,
 Pin15,
 Pin00, 
 Pin01, 
 Pin05,
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


en_pin_mode_t g294mcu_pinmode[N_USED_MAX]=
{
 Pin_Mode_Out,
 Pin_Mode_In,
 Pin_Mode_In,
 Pin_Mode_In,
 Pin_Mode_Out,
 Pin_Mode_Out,
 Pin_Mode_Out,
 Pin_Mode_In,
 Pin_Mode_In,
 Pin_Mode_In,
 Pin_Mode_Out,
 Pin_Mode_Out,
 Pin_Mode_In,
 Pin_Mode_In,
 Pin_Mode_In,
 Pin_Mode_Out,
 Pin_Mode_Out,
 Pin_Mode_Out,
 Pin_Mode_In,
 Pin_Mode_In,
 Pin_Mode_Out,
 Pin_Mode_Out,   
 Pin_Mode_Out,
 Pin_Mode_Out,   
 Pin_Mode_Out, 
 Pin_Mode_In, 
 Pin_Mode_Out,  
};

const en_port_t SPI_PORT[NMAXSPI][4] = 
 {
  {PortA,PortA,PortA,PortA},
  {PortA,PortA,PortA,PortB},
  {PortA,PortA,PortA,PortA},
 };
 
const uint16_t SPI_PIN[NMAXSPI][4] = 
 {
  {Pin02,Pin00,Pin03,Pin04},
  {Pin06,Pin05,Pin07,Pin00},
  {Pin10,Pin09,Pin11,Pin12},
 }; 
const uint32_t SPI_CLK[NMAXSPI]=
{
  PWC_FCG1_PERIPH_SPI1,
  PWC_FCG1_PERIPH_SPI2,
  PWC_FCG1_PERIPH_SPI3,
};
const en_port_func_t SPI_FUNC[NMAXSPI][4]=
{
 {Func_Spi1_Sck,Func_Spi1_Nss0,Func_Spi1_Mosi,Func_Spi1_Miso},
 {Func_Spi2_Sck,Func_Spi2_Nss0,Func_Spi2_Mosi,Func_Spi2_Miso},
 {Func_Spi3_Sck,Func_Spi3_Nss0,Func_Spi3_Mosi,Func_Spi3_Miso},
};



void G294GpioInit()
{
  uint8_t i;
  stc_port_init_t stcPortInit;  
  MEM_ZERO_STRUCT(stcPortInit);
  stcPortInit.enPinDrv=Pin_Drv_H;  
  stcPortInit.enPinOType=Pin_OType_Cmos;   
  stcPortInit.enExInt = Enable; 
    
  for(i=0;i<N_USED_MAX;i++)
	{           
     stcPortInit.enPinMode = g294mcu_pinmode[i];    
     PORT_Init(g294mcu_port[i], g294mcu_pin[i], &stcPortInit);
	}
  PORT_SetBits(g294mcu_port[M_SPI1_SS1], g294mcu_pin[M_SPI1_SS1]);
  PORT_SetBits(g294mcu_port[M_SPI1_SS2], g294mcu_pin[M_SPI1_SS2]);
  PORT_SetBits(g294mcu_port[M_SPI2_SS0], g294mcu_pin[M_SPI2_SS0]);
  PORT_SetBits(g294mcu_port[M_SPI3_SS0], g294mcu_pin[M_SPI3_SS0]);
  PORT_ResetBits(g294mcu_port[M_SPI3_SCK], g294mcu_pin[M_SPI3_SCK]);
  PORT_ResetBits(g294mcu_port[M_SPI3_MOSI], g294mcu_pin[M_SPI3_MOSI]);
  for(i=0;i<NUM_MOTOR;i++)
     motor_en(i,1);
    
 Drv8803_Reset();
 Drv8803_Enable(ENABLE);
 Drv8803_Out(0,0);
 Drv8803_Out(1,0);
 Drv8803_Out(2,0);
}


void Drv8803_Enable(uint8_t en)
{
 if(en==ENABLE)
  PORT_ResetBits(g294mcu_port[M_8803_EN], g294mcu_pin[M_8803_EN]);
 else
  PORT_SetBits(g294mcu_port[M_8803_EN], g294mcu_pin[M_8803_EN]);
}

void Drv8803_Reset()
{
  PORT_SetBits(g294mcu_port[M_8803_REST], g294mcu_pin[M_8803_REST]);
  Ddl_Delay1ms(1);
  PORT_ResetBits(g294mcu_port[M_8803_REST], g294mcu_pin[M_8803_REST]);
}

void Drv8803_Out(uint8_t num,uint8_t sts)
{
 switch(num)
 {
    case 0:
        sts ?   PORT_SetBits(g294mcu_port[M_8803_OUT1], g294mcu_pin[M_8803_OUT1]): PORT_ResetBits(g294mcu_port[M_8803_OUT1], g294mcu_pin[M_8803_OUT1]); 
     break;
    
    case 1:
        sts ?   PORT_SetBits(g294mcu_port[M_8803_OUT2], g294mcu_pin[M_8803_OUT2]): PORT_ResetBits(g294mcu_port[M_8803_OUT2], g294mcu_pin[M_8803_OUT2]);  
     break;
    
    case 2:
        sts ?   PORT_SetBits(g294mcu_port[M_8803_OUT3], g294mcu_pin[M_8803_OUT3]): PORT_ResetBits(g294mcu_port[M_8803_OUT3], g294mcu_pin[M_8803_OUT3]);  
     break;
 }
}

uint8_t Drv8803_AlarmCheck()
{
 return (PORT_GetBit(g294mcu_port[M_8803_ALARM], g294mcu_pin[M_8803_ALARM]));
}


void Init_Flash()
{  
    EFM_Unlock();

    EFM_FlashCmd(Enable);
    
    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY))
    {
      ;
    }
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







void NewSpi_Config(int nspi)
{

    stc_spi_init_t stcSpiInit;

    MEM_ZERO_STRUCT(stcSpiInit);
    
    PWC_Fcg1PeriphClockCmd(SPI_CLK[nspi], Enable);
    PORT_SetFunc(SPI_PORT[nspi][N_SPI_SCK], SPI_PIN[nspi][N_SPI_SCK], SPI_FUNC[nspi][N_SPI_SCK], Disable);
   // PORT_SetFunc(SPI_PORT[nspi][N_SPI_NSS], SPI_PIN[nspi][N_SPI_NSS], SPI_FUNC[nspi][N_SPI_NSS], Disable);
    PORT_SetFunc(SPI_PORT[nspi][N_SPI_MOSI], SPI_PIN[nspi][N_SPI_MOSI], SPI_FUNC[nspi][N_SPI_MOSI], Disable);
    PORT_SetFunc(SPI_PORT[nspi][N_SPI_MISO], SPI_PIN[nspi][N_SPI_MISO], SPI_FUNC[nspi][N_SPI_MISO], Disable);

    stcSpiInit.enClkDiv = SpiClkDiv16;//21M/16=1.3125M
    stcSpiInit.enFrameNumber = SpiFrameNumber1;
    stcSpiInit.enDataLength = SpiDataLengthBit8;
    stcSpiInit.enFirstBitPosition = SpiFirstBitPositionMSB;
    stcSpiInit.enSckPolarity = SpiSckIdleLevelLow;
    stcSpiInit.enSckPhase = SpiSckOddSampleEvenChange;//第一边沿采样（奇边沿）
    stcSpiInit.enReadBufferObject = SpiReadReceiverBuffer;
    stcSpiInit.enWorkMode = SpiWorkMode4Line;
    stcSpiInit.enTransMode = SpiTransFullDuplex;
    stcSpiInit.enCommAutoSuspendEn = Disable;
    stcSpiInit.enModeFaultErrorDetectEn = Disable;
    stcSpiInit.enParitySelfDetectEn = Disable;
    stcSpiInit.enParityEn = Disable;
    stcSpiInit.enParity = SpiParityEven;
    
    stcSpiInit.enMasterSlaveMode = SpiModeMaster;
    stcSpiInit.stcDelayConfig.enSsSetupDelayOption = SpiSsSetupDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsSetupDelayTime = SpiSsSetupDelaySck3;//SCK延迟设定位
    stcSpiInit.stcDelayConfig.enSsHoldDelayOption = SpiSsHoldDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsHoldDelayTime = SpiSsHoldDelaySck1;//SS无效延迟设定位
    stcSpiInit.stcDelayConfig.enSsIntervalTimeOption = SpiSsIntervalCustomValue;
    stcSpiInit.stcDelayConfig.enSsIntervalTime = SpiSsIntervalSck1PlusPck2;  //SPI下次存取延迟设定位
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;
    SPI_Init(whatspi_use[nspi], &stcSpiInit);
    SPI_Cmd(whatspi_use[nspi], Enable);
}

void spi_cs(uint8_t nspi,uint8_t level)
{
 switch(nspi)
 {
   case 0:
     if(level)
        PORT_SetBits(g294mcu_port[M_SPI1_SS1],g294mcu_pin[M_SPI1_SS1]);
     else
        PORT_ResetBits(g294mcu_port[M_SPI1_SS1],g294mcu_pin[M_SPI1_SS1]);   
     break;
   case 1:
     if(level)
        PORT_SetBits(g294mcu_port[M_SPI2_SS0],g294mcu_pin[M_SPI2_SS0]);
     else
        PORT_ResetBits(g294mcu_port[M_SPI2_SS0],g294mcu_pin[M_SPI2_SS0]);   
     break;  
    case 2:
     if(level)
        PORT_SetBits(g294mcu_port[M_SPI3_SS0],g294mcu_pin[M_SPI3_SS0]);
     else
        PORT_ResetBits(g294mcu_port[M_SPI3_SS0],g294mcu_pin[M_SPI3_SS0]);   
     break;

 }

}


void motor_spi_cs(uint8_t nmot,uint8_t level)
{
 switch(nmot)
 {
   case 0:
     if(level)
        PORT_SetBits(g294mcu_port[M_SPI1_SS1],g294mcu_pin[M_SPI1_SS1]);
     else
        PORT_ResetBits(g294mcu_port[M_SPI1_SS1],g294mcu_pin[M_SPI1_SS1]);   
     break;
   case 1:
     if(level)
        PORT_SetBits(g294mcu_port[M_SPI1_SS2],g294mcu_pin[M_SPI1_SS2]);
     else
        PORT_ResetBits(g294mcu_port[M_SPI1_SS2],g294mcu_pin[M_SPI1_SS2]);   
     break;  
 }
}




void ValveSpi_Config(int nspi)
{

    stc_spi_init_t stcSpiInit;

    MEM_ZERO_STRUCT(stcSpiInit);
    
    PWC_Fcg1PeriphClockCmd(SPI_CLK[nspi], Enable);
    PORT_SetFunc(SPI_PORT[nspi][N_SPI_SCK], SPI_PIN[nspi][N_SPI_SCK], SPI_FUNC[nspi][N_SPI_SCK], Disable);
   // PORT_SetFunc(SPI_PORT[nspi][N_SPI_NSS], SPI_PIN[nspi][N_SPI_NSS], SPI_FUNC[nspi][N_SPI_NSS], Disable);
    PORT_SetFunc(SPI_PORT[nspi][N_SPI_MOSI], SPI_PIN[nspi][N_SPI_MOSI], SPI_FUNC[nspi][N_SPI_MOSI], Disable);
    PORT_SetFunc(SPI_PORT[nspi][N_SPI_MISO], SPI_PIN[nspi][N_SPI_MISO], SPI_FUNC[nspi][N_SPI_MISO], Disable);

    stcSpiInit.enClkDiv = SpiClkDiv16;//21M/16=1.3125M
    stcSpiInit.enFrameNumber = SpiFrameNumber1;
    stcSpiInit.enDataLength = SpiDataLengthBit8;
    stcSpiInit.enFirstBitPosition = SpiFirstBitPositionMSB;
    stcSpiInit.enSckPolarity = SpiSckIdleLevelLow;
    stcSpiInit.enSckPhase = SpiSckOddChangeEvenSample;//SpiSckOddSampleEvenChange;//第一边沿采样（奇边沿）
    stcSpiInit.enReadBufferObject = SpiReadReceiverBuffer;
    stcSpiInit.enWorkMode = SpiWorkMode4Line;
    stcSpiInit.enTransMode = SpiTransFullDuplex;
    stcSpiInit.enCommAutoSuspendEn = Disable;
    stcSpiInit.enModeFaultErrorDetectEn = Disable;
    stcSpiInit.enParitySelfDetectEn = Disable;
    stcSpiInit.enParityEn = Disable;
    stcSpiInit.enParity = SpiParityEven;
    
    stcSpiInit.enMasterSlaveMode = SpiModeMaster;
    stcSpiInit.stcDelayConfig.enSsSetupDelayOption = SpiSsSetupDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsSetupDelayTime = SpiSsSetupDelaySck3;//SCK延迟设定位
    stcSpiInit.stcDelayConfig.enSsHoldDelayOption = SpiSsHoldDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsHoldDelayTime = SpiSsHoldDelaySck1;//SS无效延迟设定位
    stcSpiInit.stcDelayConfig.enSsIntervalTimeOption = SpiSsIntervalCustomValue;
    stcSpiInit.stcDelayConfig.enSsIntervalTime = SpiSsIntervalSck1PlusPck2;  //SPI下次存取延迟设定位
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;
    SPI_Init(whatspi_use[nspi], &stcSpiInit);
    SPI_Cmd(whatspi_use[nspi], Enable);
}

void motor_en(uint8_t nmot,uint8_t en)//0使能  1关断
{
 switch(nmot)
 {
   case 0:
     if(en)
        PORT_SetBits(g294mcu_port[M_MOTER1_ENN],g294mcu_pin[M_MOTER1_ENN]);
     else
        PORT_ResetBits(g294mcu_port[M_MOTER1_ENN],g294mcu_pin[M_MOTER1_ENN]);   
     break;
   case 1:
     if(en)
        PORT_SetBits(g294mcu_port[M_MOTER2_ENN],g294mcu_pin[M_MOTER2_ENN]);
     else
        PORT_ResetBits(g294mcu_port[M_MOTER2_ENN],g294mcu_pin[M_MOTER2_ENN]);   
     break;  

 }	   
}

uint8_t zerosignal(uint8_t nmot)
{
 if(nmot)
  return (PORT_GetBit(g294mcu_port[M_MOTOR2_ZERO], g294mcu_pin[M_MOTOR2_ZERO]));
 else
  return (PORT_GetBit(g294mcu_port[M_MOTOR1_ZERO], g294mcu_pin[M_MOTOR1_ZERO]));

}

uint8_t inputsignal(uint8_t s_num)
{
 if(s_num==0)
  return (PORT_GetBit(g294mcu_port[M_IN1], g294mcu_pin[M_IN1]));
 else if(s_num==1)
  return (PORT_GetBit(g294mcu_port[M_IN2], g294mcu_pin[M_IN2]));
 else
  return (PORT_GetBit(g294mcu_port[M_IN3], g294mcu_pin[M_IN3]));   
}


uint8_t SPI1_ReadWriteByte(uint8_t TxData,uint8_t *iserr)
{
 uint32_t waitcnt=0;
 *iserr=0;
 while(M4_SPI1->SR_f.IDLNF==Set)
  {
  }     
 
  while (M4_SPI1->SR_f.TDEF==Reset)
  {
  }
   SPI_SendData8(M4_SPI1, TxData); 

  waitcnt=0;  
  while (M4_SPI1->SR_f.RDFF==Reset)
   {
    waitcnt++;
    if(waitcnt>16800000) //100ms
    {
     *iserr=1;
    }        
   } 
 
  return SPI_ReceiveData8(M4_SPI1); 	    
}


void SPI2_3_WriteByte(uint8_t nspi,uint8_t TxData)
{
 uint8_t i;    
 if(nspi==1)//硬件SPI 
  {
  while (M4_SPI2->SR_f.TDEF==Reset)
   {
   }
   SPI_SendData8(M4_SPI2, TxData);     
  }
 else if(nspi==2)//模拟SIP
 {
   Ddl_Delay1us(1); 
    
  for(i=0;i<8;i++)
    {
     SPI3_CLK_H; 
   //  Ddl_Delay1us(1);        
     if(TxData&0x80)
      M_SPI3_MOSI_H;
     else
      M_SPI3_MOSI_L;   
     TxData =(TxData<<1);   
    // Ddl_Delay1us(1);
     SPI3_CLK_L; 
    } 
    Ddl_Delay1us(1);    
 }		   
}

uint8_t SpiIsIdel(uint8_t nspi)
{
 if(nspi==1)
    return whatspi_use[nspi]->SR_f.IDLNF;
 else
    return 0;
}

void SPI2_3_WriteLen(uint8_t nspi,uint8_t *TxData,uint8_t len)
{
 uint8_t i;
 spi_cs(nspi,0);
  for(i=0;i<len;i++)
    SPI2_3_WriteByte(nspi,TxData[i]);
    
  while (SpiIsIdel(nspi)==Set)
  {
  }
  spi_cs(nspi,1);
}   


void Init_CanConfig(void (*callback)())
{
    stc_can_init_config_t   stcCanInitCfg;
    stc_can_filter_t        stcFilter;
    stc_irq_regi_conf_t     stcIrqRegiConf;
    stc_pwc_ram_cfg_t       stcRamCfg;
    
    MEM_ZERO_STRUCT(stcRamCfg);
    MEM_ZERO_STRUCT(stcCanInitCfg);
    MEM_ZERO_STRUCT(stcFilter);

    MEM_ZERO_STRUCT(stcIrqRegiConf);

    stcRamCfg.enRamOpMd = HighSpeedMd;
    stcRamCfg.enCan = DynamicCtl;
    PWC_RamCfg(&stcRamCfg);
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_CAN, Enable);

    //<<CAN GPIO config
    PORT_SetFunc(PortB, Pin08, Func_Can1_Rx, Disable);
    PORT_SetFunc(PortB, Pin09, Func_Can1_Tx, Disable);

    //<<Can bit time config
    stcCanInitCfg.stcCanBt.PRESC = 1;
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
    stcFilter.u32CODE     = CAN_RX_ID;
    stcFilter.u32MASK     = (~stcFilter.u32CODE)&0x1FFFFFFFul;
    CAN_FilterConfig(&stcFilter, Enable);
           
    CAN_IrqCmd(CanRxIrqEn, Enable);
     
    stcIrqRegiConf.enIRQn = Int000_IRQn;
    stcIrqRegiConf.enIntSrc = INT_CAN_INT;
    stcIrqRegiConf.pfnCallback = callback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_00);//12-17 DDL_IRQ_PRIORITY_DEFAULT  t0  0
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_EnableIRQ(Int000_IRQn);
}

/*
用于新增需求
*/
int CanMessageSend(int canid,unsigned char *buf)
{
   stc_can_txframe_t       stcTxFrame;
   stcTxFrame.StdID=canid;
   stcTxFrame.Control_f.DLC = 8;
   stcTxFrame.Control_f.IDE =0;
   stcTxFrame.Control_f.RTR=0;
   stcTxFrame.enBufferSel=CanPTBSel;;


   stcTxFrame.Data[0] = buf[0];
   stcTxFrame.Data[1] = buf[1];
   stcTxFrame.Data[2] = buf[2];
   stcTxFrame.Data[3] = buf[3];
   stcTxFrame.Data[4] = buf[4];
   stcTxFrame.Data[5] = buf[5];
   stcTxFrame.Data[6] = buf[6];
   stcTxFrame.Data[7] = buf[7];

 		while(CAN_StatusGet(CanRxActive)){;}
		while(CAN_StatusGet(CanTxActive)){;}    
		CAN_SetFrame(&stcTxFrame);			
    CAN_TransmitCmd(CanPTBTxCmd); 
 		while(CAN_StatusGet(CanRxActive)){;}
		while(CAN_StatusGet(CanTxActive)){;}   
}



int CanMessageSendReal(int canid,unsigned char *buf)
{
   stc_can_txframe_t       stcTxFrame;
   stcTxFrame.StdID=canid;
   stcTxFrame.Control_f.DLC = 8;
   stcTxFrame.Control_f.IDE =0;
   stcTxFrame.Control_f.RTR=0;
   stcTxFrame.enBufferSel=CanSTBSel;


   stcTxFrame.Data[0] = buf[0];
   stcTxFrame.Data[1] = boardinfo.id;
   stcTxFrame.Data[2] = buf[2];
   stcTxFrame.Data[3] = buf[3];
   stcTxFrame.Data[4] = buf[4];
   stcTxFrame.Data[5] = buf[5];
   stcTxFrame.Data[6] = buf[6];
   stcTxFrame.Data[7] = buf[7];
   CAN_SetFrame(&stcTxFrame);
      
   CAN_TransmitCmd(CanSTBTxAllCmd); 
    
    timcan=0;
    
   while(M4_CAN->TCTRL_f.TSSTAT!=0)
   {
    if((timcan)>=5)
    { 
     return -1;
    }
   }  
}


void Init_Timer0(uint16_t time_us,void (*TimeCallback)())
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
    stcTimerCfg.Tim0_CmpValue = (uint16_t)(stcClkTmp.pclk1Freq/4/((1000000ul/(time_us)) - 1ul));
    TIMER0_BaseInit(TMR_UNIT,Tim0_ChannelB,&stcTimerCfg);

    /* Enable channel B interrupt */
    TIMER0_IntCmd(TMR_UNIT,Tim0_ChannelB,Enable);
    /* Register TMR_INI_GCMB Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = Int002_IRQn;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = TMR_INI_GCMB;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = TimeCallback;
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



















































