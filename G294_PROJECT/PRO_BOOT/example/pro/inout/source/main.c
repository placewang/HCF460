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
#include "bsp.h"
#include "message.h"
#include "eeprom.h"
#include "hc32f46x_usart.h"
#include "mb_host.h"
#include "mb_port.h"
#include "upgrade.h"

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



void TestConfigId()
{
 uint8_t buff_tx[20],arry_uid[12]={0x2A,0x0D,0xFF,0xFF,0x00,0x00,0x0A,0x1B,0xB7,0x3E,0x74,0x00};
 config_failflag=0;
 buff_tx[0]=CONFIGCMD;
 buff_tx[1]=CONFIGCMD_SETID;
 buff_tx[2]=0x01;
 buff_tx[3]=0x00;
 memcpy(&buff_tx[4],arry_uid,12);             
 UartMessagePushTxlist(UART_LEVEL1,HOST1,0xFF,FUNC_01_ONLYSEND,buff_tx,16); 
 
 
 buff_tx[0]=CONFIGCMD;
 buff_tx[1]=CONFIGCMD_SETID;
 buff_tx[2]=0x01;
 buff_tx[3]=0x00;
 memcpy(&buff_tx[4],arry_uid,12);             
 UartMessagePushTxlist(UART_LEVEL1,HOST2,0xFF,FUNC_01_ONLYSEND,buff_tx,16); 
 
 buff_tx[0]=CONFIGCMD;
 buff_tx[1]=CONFIGCMD_ASK; 
 buff_tx[2]=0x01; 
 UartMessagePushTxlist(UART_LEVEL1,HOST1,0x01,FUNC_02,buff_tx,3); 
 
 buff_tx[0]=CONFIGCMD;
 buff_tx[1]=CONFIGCMD_ASK; 
 buff_tx[2]=0x01; 
 UartMessagePushTxlist(UART_LEVEL1,HOST2,0x01,FUNC_02,buff_tx,3); 
}


unsigned char bufrx[8];
int32_t main(void)
{    
    __set_PRIMASK(0);//开总中断
    
    PORT_DebugPortSetting(1<<2, Disable);//
    
    PORT_DebugPortSetting(1<<3, Disable);//
    
    PORT_DebugPortSetting(1<<4, Disable);//NRST关闭
  
    SystemClk_Init();
       
    Ddl_Delay1ms(2);  

    InitList();    
    
    InitGpioDrive();
    
    InitConfigDrive();
    
    UpgradeLoop();
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
