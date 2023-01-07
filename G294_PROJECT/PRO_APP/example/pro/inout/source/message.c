#include "message.h"
#include "hc32f46x_efm.h"
#include "board_g294.h"
#include "logic_lay.h"

UART_TX_LIST uart_tx_list0[2];
UART_TX_LIST uart_tx_list1[2];
UART_TX_LIST uart_tx_list2[2];
UART_TX_LIST *curlist[2];

CAN_CMD_LIST canrx_cmdlist;
CAN_CMD_LIST cantx_cmdlist;
//CAN_CMD_LIST canrx_cmdlist;

AlarmList_TypeDef alarmlist;
NEWCMDMASSAGE cmdmessage;
AlarmList_TypeDef alarmlist;

unsigned char setnumsts=0,clearalarm=0;

void alarmlist_init()
{
  memset(&alarmlist,0,sizeof(alarmlist));
}

void Alarmlist_Push(AlarmList_TypeDef *alarml,uint8_t dev_id,uint8_t alarm_idx,int value)
{
 if(alarml->alarmflag)
   return;
 alarml->alarmflag=1;
 alarml->relate[alarml->writecnt].dev_id=dev_id;  
 alarml->relate[alarml->writecnt].index=alarm_idx; 
 alarml->relate[alarml->writecnt].value=value;      
 alarml->writecnt=(alarml->writecnt+1)%ALARMLEN;
}

void Alarmlist_Pop(AlarmList_TypeDef *alarml)
{
 if(alarml->readcnt!=alarml->writecnt)
 {     
  Massage_Send_4halfword(ALARM_HEAD|(alarml->relate[alarml->readcnt].dev_id<<8),alarml->relate[alarml->readcnt].index<<8,alarml->relate[alarml->readcnt].value,0);     
  alarml->readcnt=(alarml->readcnt+1)%ALARMLEN;
 }
}


void CanMessageInitlist()
{
 memset(&canrx_cmdlist,0,sizeof(canrx_cmdlist));
 memset(&cantx_cmdlist,0,sizeof(cantx_cmdlist));    
}
void CanMessagePushlist(CAN_CMD_LIST *list,unsigned short id,unsigned char *dat)
{
 int i;  
 
 list->canid[list->writecnt]=id;     
 for(i=0;i<8;i++)
  { 
   list->buf[list->writecnt][i]=dat[i];
  }
  list->writecnt=(list->writecnt+1)%CAN_RX_LEN;
    
}
int CanMessagePoplist(CAN_CMD_LIST *list,unsigned char *dat)
{
  int i;
  stc_can_txframe_t       stcTxFrame;
    
  if(list->writecnt!=list->readcnt)
	{
      for(i=0;i<8;i++)
      { 
       dat[i]=list->buf[list->readcnt][i];
      } 
      if(list==&cantx_cmdlist)
      {
        stcTxFrame.StdID=list->canid[list->readcnt];
        stcTxFrame.Control_f.DLC = 8;
        stcTxFrame.Control_f.IDE =0;
        stcTxFrame.Control_f.RTR=0;
        stcTxFrame.enBufferSel=CanSTBSel;

        for(i=0; i<stcTxFrame.Control_f.DLC; i++)
        {
           stcTxFrame.Data[i] = list->buf[list->readcnt][i];//dat[i];
        }

       CAN_SetFrame(&stcTxFrame);
      
       CAN_TransmitCmd(CanSTBTxAllCmd);

       timcan=0;           
    
       while(M4_CAN->TCTRL_f.TSSTAT!=0)
       {
         if((timcan)>=5)
         {
           //增加CAN通讯报警
           list->readcnt =(list->readcnt+1)%CAN_RX_LEN; 
            return -2;           
         }
       
       }
      
      }
          
      list->readcnt =(list->readcnt+1)%CAN_RX_LEN; 
      return 1;
    }
  return -1;
}



//与主控的通道
void Massage_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4)
{
    uint32_t canid; 
	  unsigned char buff[8]={0,0,0,0,0,0,0,0};
	  buff[0] = Msg1&0xFF;
    buff[1] = (Msg1>>8)&0xFF;
    buff[2] = Msg2&0xFF;
    buff[3] = (Msg2>>8)&0xFF;
    buff[4] = Msg3&0xFF;
    buff[5] = (Msg3>>8)&0xFF;
    buff[6] = Msg4&0xFF;
    buff[7] = (Msg4>>8)&0xFF;
    canid=CAN_TX_ID+boardinfo.id;   
		
    CanMessagePushlist(&cantx_cmdlist,canid,buff);
}





void Massage_Send_Alert(unsigned char whatsys,unsigned int Msg, unsigned int arg1)
{
   
}






void Command_exec_alarm(NEWCMDMASSAGE *cmd)
{

}


void UartMessageInitlist()
{
 int i;
 memset(&uart_tx_list0,0,sizeof(uart_tx_list0)); 
 memset(&uart_tx_list1,0,sizeof(uart_tx_list1)); 
 memset(&uart_tx_list2,0,sizeof(uart_tx_list2));
 for(i=0;i<HOSTMAX;i++)
   {
    curlist[i]=&uart_tx_list0[i]; 
   }       
}

void UartMessagePushTxlist(uint8_t level,uint8_t host,uint8_t addr,uint8_t cmd,uint8_t *data,uint8_t len)
{
  int i;
  UART_TX_LIST *puart_txlist=NULL;
    
  if(level>=UART_LEVELMAX)
    return;
  if(host>=HOSTMAX)
    return;
  
  if(((puart_txlist->write+1)%CMDLEN)==puart_txlist->read)
    return;
  
  if(level==UART_LEVEL0)  
    puart_txlist=&uart_tx_list0[host];
  else if(level==UART_LEVEL1)  
    puart_txlist=&uart_tx_list1[host];
  else if(level==UART_LEVEL2)  
    puart_txlist=&uart_tx_list2[host];
    
  puart_txlist->addr[puart_txlist->write]=addr;  
  puart_txlist->cmd[puart_txlist->write]=cmd;  
  puart_txlist->datlen[puart_txlist->write]=len;
  
  for(i=0;i<len;i++)  
  puart_txlist->data[puart_txlist->write][i]=data[i];  
 
  puart_txlist->write=(puart_txlist->write+1)%CMDLEN;
}

static void UartMessagePopTxlist(int nhost,UART_TX_LIST *list)
{
 
 
}


static void UartListCheck(int nhost)
{

  
 if(uart_tx_list0[nhost].write!=uart_tx_list0[nhost].read)
  {
     curlist[nhost]=&uart_tx_list0[nhost];
     return;
  }
  
 if(uart_tx_list1[nhost].write!=uart_tx_list1[nhost].read)
  {
     curlist[nhost]=&uart_tx_list1[nhost];
     return;
  }
  
 if(uart_tx_list2[nhost].write!=uart_tx_list2[nhost].read)
  {
     curlist[nhost]=&uart_tx_list2[nhost];
     return;
  }
}

void UartListPop()
{
  int i;
    
  for(i=0;i<HOSTMAX;i++)
    {
     UartListCheck(i);
    
     UartMessagePopTxlist(i,curlist[i]);
    }
}

static void UartCheckBoardOnline()
{
 

}


void SysConfigInit()
{
 int i;
 memset(&sysdef,0,sizeof(sysdef));
 for(i=0;i<N_PERIPH;i++)
 sysdef.periph[i].type=-1;

}

void DataInit()
{
 SysConfigInit();
}

void InitList()
{
  CanMessageInitlist();
    
  UartMessageInitlist();
    
  UartCheckBoardOnline();
    
  alarmlist_init();
       
  DataInit();
    
}














