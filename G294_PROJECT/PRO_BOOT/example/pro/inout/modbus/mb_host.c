#include "mb_include.h"
#include "string.h"
#include "bsp.h"
#include "message.h"



MBHOST mbHost[MBH_MASTER_NUM];


//modbus初始化
void mbh_init(uint32_t baud)
{
	mb_port_uartInit(baud);
	mb_port_timerInit(baud);
}
void mbHost_Init()
{
 memset(&mbHost[0],0,sizeof(mbHost[0]));
 memset(&mbHost[1],0,sizeof(mbHost[1]));
}
uint8_t mbh_getState(uint8_t idx)
{
	return mbHost[idx].state;
}
int testcnt=0;
//发送一帧命令
int8_t mbh_send(uint8_t idx,uint8_t add,uint8_t cmd,uint8_t *data,uint8_t data_len)
{
    int i;
	uint16_t crc;
    MBHOST *phost=&mbHost[idx];
    memset(&mbHost[idx],0,sizeof(mbHost[idx]));
	phost->txBuf[0]=add;
	phost->txBuf[1]=cmd;
    phost->txBuf[2]=data_len;
    for(i=0;i<data_len;i++)
     phost->txBuf[3+i]=data[i];
	phost->txLen=data_len+3; //data(n)+add(1)+cmd(1)+len(1)
	crc=mb_crc16(phost->txBuf,phost->txLen);
	phost->txBuf[phost->txLen++]=(uint8_t)(crc&0xff);
	phost->txBuf[phost->txLen++]=(uint8_t)(crc>>8);
    USART_FuncCmd(uarthost[idx], UsartTxAndTxEmptyInt,(en_functional_state_t)0);
    USART_FuncCmd(uarthost[idx], UsartTxAndTxEmptyInt,(en_functional_state_t)1);
    Rs485_DirContrl(idx,SENDSTS);
	mb_port_putchar(idx,phost->txBuf[phost->txCounter]);
    phost->txCounter++;
    phost->state=MBH_STATE_TX;
	return 0;
}

void mbh_poll(uint8_t idx)
{
  MBHOST *phost=&mbHost[idx];
    
	switch(phost->state)
	{
		/*接收完一帧数据,开始进行校验*/
		case MBH_STATE_RX_CHECK:
            if(phost->rxCounter>=MBH_RTU_MIN_SIZE)//&&(mb_crc16(phost->rxBuf,phost->rxCounter)==0)) 	
			{
				if(phost->txBuf[0]==phost->rxBuf[0])		
				{
                  if(phost->rxCounter>=(phost->rxBuf[2]+5))//5 地址码+功能码+字节长度+两字节校验
                    {
                     if(mb_crc16(phost->rxBuf,phost->rxBuf[2]+3)==((phost->rxBuf[phost->rxCounter-1]<<8)|(phost->rxBuf[phost->rxCounter-2])))
                       phost->state=MBH_STATE_EXEC;
                     else
                       phost->state=MBH_STATE_REC_ERR;  
                    }                      
				}
				else
                 phost->state=MBH_STATE_REC_ERR;
				
			}
           else
             phost->state=MBH_STATE_REC_ERR;
                
			break;
		/*接收一帧数据出错*/	
		case MBH_STATE_REC_ERR:
 
			phost->errTimes++;			
			if(phost->errTimes>=MBH_ERR_MAX_TIMES)  
			{
                if(configend_sendonline==0)//配置id阶段
                 {
                  if((phost->txBuf[3]==CONFIGCMD)&&(phost->txBuf[4]==CONFIGCMD_ASK))//查询ID配置成功的命令
                   {
                     config_failflag++;
                     if(config_failflag>=2)
                        Massage_Send_4halfword((0x06<<8)|(0x01),phost->txBuf[0],0,0);  //配置id失败
                   }
                   
                 }               
                 phost->errTimes=0;
				 phost->state=MBH_STATE_TIMES_ERR;
       
			}
			else  //重新再启动一次传输
			{
				phost->txCounter=0;
                USART_FuncCmd(uarthost[idx], UsartTxAndTxEmptyInt,(en_functional_state_t)0);
                USART_FuncCmd(uarthost[idx], UsartTxAndTxEmptyInt,(en_functional_state_t)1);
                Rs485_DirContrl(idx,SENDSTS);
	            mb_port_putchar(idx,phost->txBuf[phost->txCounter]);
                phost->txCounter++;
                phost->state=MBH_STATE_TX;
			}
                
			break;
		/*超过最大错误传输次数*/
		case MBH_STATE_TIMES_ERR:
         if(((phost->txBuf[3]==CONFIGCMD)&&(phost->txBuf[4]==CONFIGCMD_ASK))||(phost->txBuf[3]==GETCMD)) 
            {
            }
         else              
            Alarmlist_Push(&alarmlist,phost->txBuf[0],bit_alarminfor|(1<<BIT_UNLINK_SONBOARD));
         
            memset(&mbHost[idx],0,sizeof(mbHost[idx]));
            mb_port_uartInitByidx(idx,115200);           
			break;
         
		/*确定接收正确执行回调*/
		case MBH_STATE_EXEC: 
            memset(phost->txBuf,0,MBH_RTU_MAX_SIZE);            
			mbh_exec(idx,phost->rxBuf[0],&phost->rxBuf[3],phost->rxBuf[2]);
			phost->state=MBH_STATE_IDLE;
			break;
		
	}
}


void mbh_timer3T5Isr(uint8_t idx)
{
  MBHOST *phost=&mbHost[idx];
	switch(phost->state)
	{
		/*发送完但没有接收到数据*/
		case MBH_STATE_TX_END:
           if(phost->txBuf[1]==FUNC_01_ONLYSEND)//仅发送
             {
              mb_port_timerDisable(idx); 
              phost->state=MBH_STATE_IDLE;                 
              break;                 
             }
			phost->rxTimeOut++;
			if(phost->rxTimeOut>=MBH_REC_TIMEOUT) //接收超时
			{
				phost->rxTimeOut=0;
				phost->state=MBH_STATE_REC_ERR;
				mb_port_timerDisable(idx);		//关闭定时器
                Rs485_DirContrl(idx,SENDSTS);
                USART_FuncCmd(uarthost[idx], UsartRxInt,(en_functional_state_t)0); 
			}
			break;
		case MBH_STATE_RX:     	//3.5T到,接收一帧完成			
			phost->state=MBH_STATE_RX_CHECK;
			mb_port_timerDisable(idx);		//关闭定时器
            Rs485_DirContrl(idx,SENDSTS);
            USART_FuncCmd(uarthost[idx], UsartRxInt, (en_functional_state_t)0); 
			break;
	}	
}
void mbh_uartRxIsr(uint8_t idx)
{
    MBHOST *phost=&mbHost[idx];
	switch(phost->state)
	{
		case MBH_STATE_TX_END:
			phost->rxCounter=0;
			phost->rxBuf[phost->rxCounter++]=uarthost[idx]->DR_f.RDR;
         
			phost->state=MBH_STATE_RX;
			mb_port_timerEnable(idx);
			break;
		case MBH_STATE_RX:
			if(phost->rxCounter<MBH_RTU_MAX_SIZE)
			{
			 phost->rxBuf[phost->rxCounter++]=uarthost[idx]->DR_f.RDR;
			}
			mb_port_timerEnable(idx);
			break;
		default:
			mb_port_timerEnable(idx);
			break;		
	}
}
void mbh_uartTxIsr(uint8_t idx)
{
  MBHOST *phost=&mbHost[idx];
	switch (phost->state)
	{
       
		case MBH_STATE_TX:
			if(phost->txCounter>phost->txLen) //全部发送完
			{                 
                mb_port_timercnt_clear(idx);
                phost->rxCounter=0;
                if(phost->txBuf[1]!=FUNC_01_ONLYSEND)
                {
                 Rs485_DirContrl(idx,RECIVESTS);
                 USART_FuncCmd(uarthost[idx], UsartRxInt,(en_functional_state_t)1); 
                }
                    
				phost->rxTimeOut=0;		  //清除接收超时计数     
				mb_port_timerEnable(idx);    //open time  
                phost->state=MBH_STATE_TX_END;  
			}
			else
			{
				mb_port_putchar(idx,phost->txBuf[phost->txCounter++]);
			}
			break;		
	}	
}



