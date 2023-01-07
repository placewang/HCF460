#include "mb_include.h"



#define TIMERA_UNIT1                    (M4_TMRA1)
#define TIMERA_UNIT1_CLOCK              (PWC_FCG2_PERIPH_TIMA1)
#define TIMERA_UNIT1_OVERFLOW_INT       (INT_TMRA1_OVF)

#define TIMERA_UNIT2                    (M4_TMRA2)
#define TIMERA_UNIT2_CLOCK              (PWC_FCG2_PERIPH_TIMA2)
#define TIMERA_UNIT2_OVERFLOW_INT       (INT_TMRA2_OVF)



M4_USART_TypeDef* uarthost[2]={M4_USART3,M4_USART4}; 

 const stc_usart_uart_init_t stcInitCfg = {
        UsartIntClkCkNoOutput,
        UsartClkDiv_1,
        UsartDataBits8,
        UsartDataLsbFirst,
        UsartOneStopBit,
        UsartParityNone,
        UsartSamleBit8,
        UsartStartBitFallEdge,
        UsartRtsEnable,
    };
 
    
uint8_t buff3,buff4;
void Usart3RxIrqCallback()
{
 mbh_uartRxIsr(HOST1);
}

void Usart3TxIrqCallback()
{
 mbh_uartTxIsr(HOST1);
}

void Usart4RxIrqCallback()
{
 mbh_uartRxIsr(HOST2);
}

void Usart4TxIrqCallback()
{
 mbh_uartTxIsr(HOST2);
}

void mb_port_uartInitByidx(uint8_t idx,uint32_t baud)
{
   stc_irq_regi_conf_t stcIrqRegiCfg;
 // PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART3 | PWC_FCG1_PERIPH_USART4, Enable);
    
  USART_UART_Init(uarthost[idx], &stcInitCfg);
    
  USART_SetBaudrate(uarthost[idx], baud);
    
  USART_FuncCmd(uarthost[idx], UsartRx, Enable);
  USART_FuncCmd(uarthost[idx], UsartTx, Enable);
 
 if(idx==HOST1) 
 {     
  stcIrqRegiCfg.enIRQn = Int003_IRQn;
  stcIrqRegiCfg.pfnCallback = &Usart3RxIrqCallback;
  stcIrqRegiCfg.enIntSrc = INT_USART3_RI;
  enIrqRegistration(&stcIrqRegiCfg);
  NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_10);
  NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
  NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
  USART_FuncCmd(uarthost[idx], UsartRxInt, Enable);
  
  
  stcIrqRegiCfg.enIRQn = Int004_IRQn;
  stcIrqRegiCfg.pfnCallback = &Usart3TxIrqCallback;
  stcIrqRegiCfg.enIntSrc = INT_USART3_TI;
  enIrqRegistration(&stcIrqRegiCfg);
  NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
  NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
  NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
 }
else
 {

  stcIrqRegiCfg.enIRQn = Int005_IRQn;
  stcIrqRegiCfg.pfnCallback = &Usart4RxIrqCallback;
  stcIrqRegiCfg.enIntSrc = INT_USART4_RI;
  enIrqRegistration(&stcIrqRegiCfg);
  NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_10);
  NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
  NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
  USART_FuncCmd(uarthost[idx], UsartRxInt, Enable);
  
  stcIrqRegiCfg.enIRQn = Int006_IRQn;
  stcIrqRegiCfg.pfnCallback = &Usart4TxIrqCallback;
  stcIrqRegiCfg.enIntSrc = INT_USART4_TI;
  enIrqRegistration(&stcIrqRegiCfg);
  NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
  NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
  NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
 }
}

void mb_port_uartInit(uint32_t baud)
{
  stc_irq_regi_conf_t stcIrqRegiCfg;
  PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART3 | PWC_FCG1_PERIPH_USART4, Enable);

  PORT_SetFunc(mcu_port[IO_UART3_RX], mcu_pin[IO_UART3_RX], Func_Usart3_Rx, Disable);
  PORT_SetFunc(mcu_port[IO_UART3_TX], mcu_pin[IO_UART3_TX], Func_Usart3_Tx, Disable);

  PORT_SetFunc(mcu_port[IO_UART4_RX], mcu_pin[IO_UART4_RX], Func_Usart4_Rx, Disable);
  PORT_SetFunc(mcu_port[IO_UART4_TX], mcu_pin[IO_UART4_TX], Func_Usart4_Tx, Disable);
    
  USART_UART_Init(uarthost[HOST1], &stcInitCfg);
  USART_UART_Init(uarthost[HOST2], &stcInitCfg); 
    
  USART_SetBaudrate(uarthost[HOST1], baud);
  USART_SetBaudrate(uarthost[HOST2], baud);
    
  USART_FuncCmd(uarthost[HOST1], UsartRx, Enable);
  USART_FuncCmd(uarthost[HOST1], UsartTx, Enable);
    
  USART_FuncCmd(uarthost[HOST2], UsartRx, Enable);
  USART_FuncCmd(uarthost[HOST2], UsartTx, Enable);
    
  stcIrqRegiCfg.enIRQn = Int003_IRQn;
  stcIrqRegiCfg.pfnCallback = &Usart3RxIrqCallback;
  stcIrqRegiCfg.enIntSrc = INT_USART3_RI;
  enIrqRegistration(&stcIrqRegiCfg);
  NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_10);
  NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
  NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
  USART_FuncCmd(uarthost[HOST1], UsartRxInt, Enable);
  
  
  stcIrqRegiCfg.enIRQn = Int004_IRQn;
  stcIrqRegiCfg.pfnCallback = &Usart3TxIrqCallback;
  stcIrqRegiCfg.enIntSrc = INT_USART3_TI;
  enIrqRegistration(&stcIrqRegiCfg);
  NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
  NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
  NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
  
  
  stcIrqRegiCfg.enIRQn = Int005_IRQn;
  stcIrqRegiCfg.pfnCallback = &Usart4RxIrqCallback;
  stcIrqRegiCfg.enIntSrc = INT_USART4_RI;
  enIrqRegistration(&stcIrqRegiCfg);
  NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_10);
  NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
  NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
  USART_FuncCmd(uarthost[HOST2], UsartRxInt, Enable);
  
  stcIrqRegiCfg.enIRQn = Int006_IRQn;
  stcIrqRegiCfg.pfnCallback = &Usart4TxIrqCallback;
  stcIrqRegiCfg.enIntSrc = INT_USART4_TI;
  enIrqRegistration(&stcIrqRegiCfg);
  NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
  NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
  NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
  
}

void mb_port_uartInitTest()
{
//stc_irq_regi_conf_t stcIrqRegiCfg;
 // PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART3 | PWC_FCG1_PERIPH_USART4, Enable);


    
 // USART_FuncCmd(uarthost[HOST1], UsartRx, Enable);
//  USART_FuncCmd(uarthost[HOST1], UsartTx, Enable);
    
//  USART_FuncCmd(uarthost[HOST2], UsartRx, Enable);
//  USART_FuncCmd(uarthost[HOST2], UsartTx, Enable);
    
/* stcIrqRegiCfg.enIRQn = Int003_IRQn;
  stcIrqRegiCfg.pfnCallback = &Usart3RxIrqCallback;
  stcIrqRegiCfg.enIntSrc = INT_USART3_RI;
  enIrqRegistration(&stcIrqRegiCfg);
  NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_10);
  NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
  NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);*/
  USART_FuncCmd(uarthost[HOST1], UsartRxInt, Enable);
  
  
 /* stcIrqRegiCfg.enIRQn = Int004_IRQn;
  stcIrqRegiCfg.pfnCallback = &Usart3TxIrqCallback;
  stcIrqRegiCfg.enIntSrc = INT_USART3_TI;
  enIrqRegistration(&stcIrqRegiCfg);
  NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
  NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
  NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
  
  
  stcIrqRegiCfg.enIRQn = Int005_IRQn;
  stcIrqRegiCfg.pfnCallback = &Usart4RxIrqCallback;
  stcIrqRegiCfg.enIntSrc = INT_USART4_RI;
  enIrqRegistration(&stcIrqRegiCfg);
  NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_10);
  NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
  NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);*/
  USART_FuncCmd(uarthost[HOST2], UsartRxInt, Enable);
  
 /* stcIrqRegiCfg.enIRQn = Int006_IRQn;
  stcIrqRegiCfg.pfnCallback = &Usart4TxIrqCallback;
  stcIrqRegiCfg.enIntSrc = INT_USART4_TI;
  enIrqRegistration(&stcIrqRegiCfg);
  NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
  NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
  NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);*/

}

void mb_port_putchar(uint8_t idx,uint8_t ch)
{
  uarthost[idx]->DR_f.TDR = (uint32_t)ch;
}

void mb_port_getchar(uint8_t idx,uint8_t *ch)
{
  *ch=USART_RecData(uarthost[idx]);
}


void TimeraUnit1_IrqCallback()
{
 mbh_timer3T5Isr(HOST1);
}

void TimeraUnit2_IrqCallback()
{
 mbh_timer3T5Isr(HOST2);
}
//定时器的两个通道分别作为UART3--CHAN_A    UART4--CHAN_B     3.5T
void mb_port_timerInit(uint32_t baud)
{
    stc_timera_base_init_t stcTimeraInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_timera_orthogonal_coding_init_t stcTimeraCondingInit;
    int usTimerT35_50us=0;
   

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimeraCondingInit);
    
   // CLK_GetClockFreq(&stcClkTmp); 
    
     if( baud > 19200 )
        {
            usTimerT35_50us = 45;//       /* 2250us. */
        }
        else
        {
            /* The timer reload value for a character is given by:
             *
             * ChTimeValue = Ticks_per_1s / ( Baudrate / 11 )
             *             = 11 * Ticks_per_1s / Baudrate
             *             = 220000 / Baudrate
             * The reload for t3.5 is 1.5 times this value and similary
             * for t3.5.
             */
            //usTimerT35_50us = ( 7UL * 220000UL ) / ( 2UL * ulBaudRate );
					  usTimerT35_50us = 150;//150;
        }
    


    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIMA1 | PWC_FCG2_PERIPH_TIMA2, Enable);

  
    /* Configuration timera unit 1 structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv4;//21M/4  PCLK1上
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal =(21*50*usTimerT35_50us)/4;       //3.5T
    TIMERA_BaseInit(TIMERA_UNIT1, &stcTimeraInit);
    TIMERA_IrqCmd(TIMERA_UNIT1, TimeraIrqOverflow, Enable);
        

    /* Configure interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT1_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = Int007_IRQn;
    stcIrqRegiConf.pfnCallback = &TimeraUnit1_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_14);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Configuration timera unit 2 structure */
    stcTimeraInit.enSyncStartupEn = Disable;
    TIMERA_BaseInit(TIMERA_UNIT2, &stcTimeraInit);
    TIMERA_IrqCmd(TIMERA_UNIT2, TimeraIrqOverflow, Enable);

    /* Configure interrupt of timera unit 2 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT2_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = Int008_IRQn;
    stcIrqRegiConf.pfnCallback = &TimeraUnit2_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_14);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    TIMERA_SetCurrCount(TIMERA_UNIT1, 0);
    TIMERA_SetCurrCount(TIMERA_UNIT2, 0);

    /* Sync startup timera unit 2 when timera unit 1 startup */
    TIMERA_Cmd(TIMERA_UNIT1, Disable);
    TIMERA_Cmd(TIMERA_UNIT2, Disable);
}


void mb_port_timercnt_clear(uint8_t idx)
{
  if(idx==HOST1)
    {
      TIMERA_SetCurrCount(TIMERA_UNIT1, 0);
    }
  else if(idx==HOST2)
    {     
      TIMERA_SetCurrCount(TIMERA_UNIT2, 0);  
    }                       
}

void mb_port_timerEnable(uint8_t idx)
{
  if(idx==HOST1)
    {
     // NVIC_ClearPendingIRQ(Int007_IRQn);
      TIMERA_Cmd(TIMERA_UNIT1, Enable);  
    }
  else if(idx==HOST2)
    {
     // NVIC_ClearPendingIRQ(Int008_IRQn);
      TIMERA_Cmd(TIMERA_UNIT2, Enable);  
    }                         
}

void mb_port_timerDisable(uint8_t idx)
{
 if(idx==HOST1)
    {
      TIMERA_Cmd(TIMERA_UNIT1, Disable);  
    }
  else if(idx==HOST2)
    {
      TIMERA_Cmd(TIMERA_UNIT2, Disable);  
    }      
}



