/*******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co., Ltd. ("HDSC").
 *
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
 * BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
 *
 * This software contains source code for use with HDSC
 * components. This software is licensed by HDSC to be adapted only
 * for use in systems utilizing HDSC components. HDSC shall not be
 * responsible for misuse or illegal use of this software for devices not
 * supported herein. HDSC is providing this software "AS IS" and will
 * not be responsible for issues arising from incorrect user implementation
 * of the software.
 *
 * Disclaimer:
 * HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
 * REGARDING THE SOFTWARE (INCLUDING ANY ACCOMPANYING WRITTEN MATERIALS),
 * ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
 * WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
 * WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
 * WARRANTY OF NONINFRINGEMENT.
 * HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
 * NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
 * LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
 * LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
 * INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
 * SAVINGS OR PROFITS,
 * EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
 * INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
 * FROM, THE SOFTWARE.
 *
 * This software may be replicated in part or whole for the licensed use,
 * with the restriction that this Disclaimer and Copyright notice must be
 * included with each copy of this software, whether used in part or whole,
 * at all times.
 */
/******************************************************************************/
/** \file main.c
 **
 ** \brief The example of SPI four wire interrupt tx and rx function
 **
 **   - 2018-11-08  1.0  Yangjp First version for Device Driver Library of SPI.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"
#include "message.h"
#include "eeprom.h"
#include "hc32f46x_usart.h"
#include "logic_lay.h"
#include "upgrade.h"
#include "board_g294.h"
#include "tmc5610.h"
#include "mot.h"

/*****************************************************/
#include "motordemand.h"
/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* LED0 Port/Pin definition */


/* LED1 Port/Pin definition */
#define LED1_PORT                       (PortA)
#define LED1_PIN                        (Pin07)

#define LED1_ON()                       (PORT_SetBits(LED1_PORT, LED1_PIN))
#define LED1_OFF()                      (PORT_ResetBits(LED1_PORT, LED1_PIN))
#define LED1_TOGGLE()                   (PORT_Toggle(LED1_PORT, LED1_PIN))

/* KEY0 Port/Pin definition */
#define KEY0_PORT                       (PortD)
#define KEY0_PIN                        (Pin03)





/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief ExtInt3 callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/


/**
 *******************************************************************************
 ** \brief SPI3 send callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
#if 0
static void SpiTx_IrqCallback(void)
{
    if (u8TxIndex < u16BufferLen)
    {
        SPI_SendData8(SPI3_UNIT, u8TxBuffer[u8TxIndex++]);
    }
    else
    {
        SPI_IrqCmd(SPI3_UNIT, SpiIrqSend, Disable);
    }

}
#endif
/**
 *******************************************************************************
 ** \brief SPI3 receive callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
#if 0
static void SpiRx_IrqCallback(void)
{
    if (u8RxIndex < u16BufferLen)
    {
        u8RxBuffer[u8RxIndex++] = SPI_ReceiveData8(SPI3_UNIT);
    }
    else
    {
        SPI_IrqCmd(SPI_UNIT3, SpiIrqReceive, Disable);
    }
}
#endif
/**
 *******************************************************************************
 ** \brief KEY0(SW2) init function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
#if 0
static void Sw2_Init(void)
{
    stc_port_init_t stcPortInit;
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Set PD03 as External Int Ch.3 input */
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY0_PORT, KEY0_PIN, &stcPortInit);

    stcExtiConfig.enExitCh = ExtiCh03;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Disable;
    stcExtiConfig.enFltClk = Pclk3Div1;
    /* Both edge */
    stcExtiConfig.enExtiLvl = ExIntFallingEdge;
    EXINT_Init(&stcExtiConfig);

    /* Select External Int Ch.3 */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ3;
    /* Register External Int to Vect.No.007 */
    stcIrqRegiConf.enIRQn = Int007_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &ExtInt03_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}
#endif
/**
 *******************************************************************************
 ** \brief System clock init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void SystemClk_Init(void)
{  
    stc_clk_sysclk_cfg_t    stcSysClkCfg;
    stc_clk_xtal_cfg_t      stcXtalCfg;
    stc_clk_mpll_cfg_t      stcMpllCfg;

    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv = ClkSysclkDiv1;   // Max 168MHz
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  // Max 168MHz
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv8;  // Max 21MHz
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  // Max 42MHz
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  // Max 42MHz
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  // Max 84MHz
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Switch system clock source to MPLL. */
    /* Use Xtal32 as MPLL source. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* MPLL config (XTAL / pllmDiv * plln / PllpDiv = 128Mhz). */
    stcMpllCfg.pllmDiv = 1u;
    stcMpllCfg.plln = 42u;//42->32   168Mhz
    stcMpllCfg.PllpDiv = 2u;
    
    stcMpllCfg.PllqDiv = 2u;
    stcMpllCfg.PllrDiv = 2u;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_5);
    EFM_Lock();

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while (Set != CLK_GetFlagStatus(ClkFlagMPLLRdy))
    {
    }

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);
}

uint8_t iodat1[8]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
uint8_t iodat2[8]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
static void LedPro()
{ 
 static uint8_t level=0;   
  
 if(time_led>=(500000/TIME0_LEVEL))//500ms
 {
   time_led=0;
   LED0_TOGGLE; 
  }
 if(time_valve>(2000000/TIME0_LEVEL))
 {
  time_valve=0;
    if(level==0)
    {
     level=1;
  //   ValveDriverChangeBit(1,iodat1,8);
     //SPI2_3_WriteLen(2,iodat1,8); 
    }
   else
   {
     level=0;
   //  ValveDriverChangeBit(1,iodat2,8);
    // SPI2_3_WriteLen(2,iodat2,8);   
   }
 
 }
 if(read_status_cnt > (5000/TIME0_LEVEL))
 {
 		mot_r_limit =  mot_rightlimit_get(0) ;
	  read_status_cnt = 0 ;
 }
  
}
int test1=0,test2=0;
unsigned char bufrx[8]={0};
signed int valseep=0,valseep2=0;

int32_t main(void)
{  
   uint32_t checkcnt=0;    
    __set_PRIMASK(0);//开总中断
    
    PORT_DebugPortSetting(1<<2, Disable);//
    
    PORT_DebugPortSetting(1<<3, Disable);//
    
    PORT_DebugPortSetting(1<<4, Disable);//NRST关闭
  
    SystemClk_Init();
       
    Ddl_Delay1ms(100);//给子板上电留时间  

    InitList(); 

    G294GpioInit(); 
    
    Init_Flash();
    
    Init_GetBoardInfo();
    
    AlarmInfoInit();
    
    Init_RegisterSpiErr();

    Init_Timer0(TIME0_LEVEL,Tim0_Callback);    

    NewSpi_Config(0);  

    ValveSpi_Config(1);
		
    
    Init_CanConfig(CAN_RxIrqCallBack);
    
    Ddl_Delay1ms(10);
    
    InitDefaultValveConfig();
     
    mot_init();
		mot_enable(0,0);
		mot_runbysped_mode(0,0,0);	
		mot_curpos_set(0,0);
		
    while(1)
    {         
      LedPro();
      UartListPop();
      CanSendSignal();
      mot[0].pos=mot_curpos_get(0);
      mot[1].pos=mot_curpos_get(1);
			mot[0].limit_pos = mot_limit_pos_get(0);
      MotResetLoop();
      CanMessagePoplist(&cantx_cmdlist,NULL);
      Alarmlist_Pop(&alarmlist);
    if(CanMessagePoplist(&canrx_cmdlist,bufrx)>0)
      CanReciveDataDeal(bufrx);
   
		
		 valseep=mot_curpos_get(0);
    lift_mot_run_loop();
		CanRaedMassgaAnalysis(bufrx);
    if(Drv8803_AlarmCheck()==0)
     {
      checkcnt++;
      if(checkcnt>=50000)
       {
         checkcnt=0;
         Alarmlist_Push(&alarmlist,DEVID_INOUT,ALARM_ALARM8803_ERR,0);
       }
     }
    else
      checkcnt=0;        
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
