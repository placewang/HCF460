#include "message.h"
#include "hc32f46x_efm.h"
#include "mb_host.h"

UART_TX_LIST uart_tx_list0[HOSTMAX];
UART_TX_LIST uart_tx_list1[HOSTMAX];
UART_TX_LIST uart_tx_list2[HOSTMAX];
UART_TX_LIST *curlist[HOSTMAX];

CAN_CMD_LIST canrx_cmdlist;
CAN_CMD_LIST cantx_cmdlist;
CAN_CMD_LIST canrx_cmdlist;
AlarmList_TypeDef alarmlist;
NEWCMDMASSAGE cmdmessage;
AlarmList_TypeDef alarmlist;
uint16_t bit_alarminfor;

unsigned char setnumsts=0,clearalarm=0;

void alarmlist_init()
{
  memset(&alarmlist,0,sizeof(alarmlist));
}

void Alarmlist_Push(AlarmList_TypeDef *alarml,uint8_t id,int infor)
{
 if(alarml->alarmflag)
   return;
 alarml->alarmflag=1;
 alarml->val[alarml->writecnt].id=id; 
 alarml->val[alarml->writecnt].infor=infor;      
 alarml->writecnt=(alarml->writecnt+1)%ALARMLEN;
}

void Alarmlist_Pop(AlarmList_TypeDef *alarml)
{
 if(alarml->readcnt!=alarml->writecnt)
 {   
  Massage_Send_4halfword((CMD_ALARM_SON_INFOR<<8)|CMDTYPE_ALARM,alarml->val[alarml->readcnt].id,alarml->val[alarml->readcnt].infor,0);     
  alarml->readcnt=(alarml->readcnt+1)%ALARMLEN;
 }
}







void CanMessageInitlist()
{
 memset(&canrx_cmdlist,0,sizeof(canrx_cmdlist));
 memset(&cantx_cmdlist,0,sizeof(cantx_cmdlist));    
}
void CanMessagePushlist(CAN_CMD_LIST *list,unsigned char *dat)
{
 int i;  
    
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
        stcTxFrame.StdID=NEW_CANID_COM_TX;
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





//调试通道
void DebugInforByCan(char arg1,int arg2)
{
 unsigned char buf[8]={0,0,0,0,0,0,0,0};
 buf[0]=0x30;
 buf[1]=arg1;
 buf[2]=0x3A;
 buf[3]=0x3A;
 buf[4]=arg2/1000+0x30;
 buf[5]=arg2%1000/100+0x30;
 buf[6]=arg2%1000%100/10+0x30;
 buf[7]=arg2%10+0x30;
 
 CanMessagePushlist(&cantx_cmdlist,buf);
}


void DebugInforDisplay(char *arg1)
{
 CanMessagePushlist(&cantx_cmdlist,(unsigned char *)arg1);
}

//与主控的通道
void Massage_Send_4halfword(unsigned int Msg1,unsigned int Msg2,unsigned int Msg3,unsigned int Msg4)
{
	unsigned char buff[8]={0,0,0,0,0,0,0,0};
	buff[0] = Msg1&0xFF;
    buff[1] = (Msg1>>8)&0xFF;
    buff[2] = Msg2&0xFF;
    buff[3] = (Msg2>>8)&0xFF;
    buff[4] = Msg3&0xFF;
    buff[5] = (Msg3>>8)&0xFF;
    buff[6] = Msg4&0xFF;
    buff[7] = (Msg4>>8)&0xFF;
   
  CanMessagePushlist(&cantx_cmdlist,buff);
}



void Massage_Send_Alert(unsigned char whatsys,unsigned int Msg, unsigned int arg1)
{
   
}



int CanMessageSendReal22(int canid,unsigned int buf)
{
   stcTxFrame.StdID=canid;
   stcTxFrame.Control_f.DLC = 8;
   stcTxFrame.Control_f.IDE =0;
   stcTxFrame.Control_f.RTR=0;
   stcTxFrame.enBufferSel=CanSTBSel;


   stcTxFrame.Data[0] = buf&0xFF;
   stcTxFrame.Data[1] = (buf>>8)&0xFF;
  
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
return 0;
  
}

void SonBoardAllCmdBackDeal(NEWCMDMASSAGE *cmd)
{
 uint8_t buftx[20];
 
  switch(cmd->cmd_data)
  { 
      case CMD_ALLBACK_GETBOARDTYPE:
         buftx[0]=GETCMD;   
         buftx[1]=GETBOARDTYPE;
         buftx[2]=cmd->arg1_l; 
         UartMessagePushTxlist(UART_LEVEL0,HOST1,cmd->arg1_l,FUNC_02,buftx,3);   
         UartMessagePushTxlist(UART_LEVEL0,HOST2,cmd->arg1_l,FUNC_02,buftx,3);   
        break;
      
      case CMD_ALLBACK_GETUID:
         buftx[0]=GETCMD;   
         buftx[1]=GETBOARDUID;
         buftx[2]=cmd->arg1_l; 
         buftx[3]=0; 
         UartMessagePushTxlist(UART_LEVEL0,HOST1,cmd->arg1_l,FUNC_02,buftx,4);  
         buftx[3]=1; 
         UartMessagePushTxlist(UART_LEVEL0,HOST1,cmd->arg1_l,FUNC_02,buftx,4);   
         buftx[3]=2; 
         UartMessagePushTxlist(UART_LEVEL0,HOST1,cmd->arg1_l,FUNC_02,buftx,4); 

         buftx[0]=GETCMD;   
         buftx[1]=GETBOARDUID;
         buftx[2]=cmd->arg1_l; 
         buftx[3]=0; 
         UartMessagePushTxlist(UART_LEVEL0,HOST2,cmd->arg1_l,FUNC_02,buftx,4);  
         buftx[3]=1; 
         UartMessagePushTxlist(UART_LEVEL0,HOST2,cmd->arg1_l,FUNC_02,buftx,4);   
         buftx[3]=2; 
         UartMessagePushTxlist(UART_LEVEL0,HOST2,cmd->arg1_l,FUNC_02,buftx,4);       
          
        break;

  }

}

void MainBoardCmdDeal(NEWCMDMASSAGE *cmd)
{
 static uint8_t setid=0,setboardtype=0,arry_uid[12]={0,0,0,0,0,0,0,0,0,0,0,0};
 uint8_t buff_tx[20];
 
  switch(cmd->cmd_data)
  {
    case CMD_MAIN_GET_UID:
        
        break;
    
    case CMD_MAIN_GET_HARDWARE:
        
        break;
    
    case CMD_MAIN_GET_SOFTVER:
        
        break;
    
    case CMD_MAIN_GET_VI1:
        
        break;
    
    case CMD_MAIN_GET_VI2:
        
        break;
    
    case CMD_MAIN_GET_INPUT:
        
        break;
    
    case CMD_BOARD_CONFIG_ID:
        if(cmd->arg1_l==0)
         {
          setid=cmd->arg1_h;
          setboardtype=cmd->arg2_l;
          arry_uid[0]=cmd->arg2_h;
          arry_uid[1]=cmd->arg3_l;
          arry_uid[2]=cmd->arg3_h;
         }
        else if(cmd->arg1_l==1)
         {
          arry_uid[3]=cmd->arg1_h;
          arry_uid[4]=cmd->arg2_l;
          arry_uid[5]=cmd->arg2_h;
          arry_uid[6]=cmd->arg3_l;  
          arry_uid[7]=cmd->arg3_h;                
         }
        else if(cmd->arg1_l==2)
         {
          arry_uid[8]=cmd->arg1_h;
          arry_uid[9]=cmd->arg2_l;
          arry_uid[10]=cmd->arg2_h;
          arry_uid[11]=cmd->arg3_l;            
          config_failflag=0;      
          memcpy(&buff_tx[4],arry_uid,12);   
                        
          buff_tx[0]=CONFIGCMD;
          buff_tx[1]=CONFIGCMD_SETID;
          buff_tx[2]=setid;
          buff_tx[3]=setboardtype;
          memcpy(&buff_tx[4],arry_uid,12);             
          UartMessagePushTxlist(UART_LEVEL0,HOST1,0xFF,FUNC_01_ONLYSEND,buff_tx,16); 
 
 
          buff_tx[0]=CONFIGCMD;
          buff_tx[1]=CONFIGCMD_SETID;
          buff_tx[2]=setid;
          buff_tx[3]=setboardtype;
          memcpy(&buff_tx[4],arry_uid,12);             
          UartMessagePushTxlist(UART_LEVEL0,HOST2,0xFF,FUNC_01_ONLYSEND,buff_tx,16); 
 
          buff_tx[0]=CONFIGCMD;
          buff_tx[1]=CONFIGCMD_ASK; 
          buff_tx[2]=setid; 
          UartMessagePushTxlist(UART_LEVEL0,HOST1,setid,FUNC_02,buff_tx,3); 
 
          buff_tx[0]=CONFIGCMD;
          buff_tx[1]=CONFIGCMD_ASK; 
          buff_tx[2]=setid; 
          UartMessagePushTxlist(UART_LEVEL0,HOST2,setid,FUNC_02,buff_tx,3);        
         }
        break;
         
    case CMD_BOARD_GOIN_CONFIG:
         if(cmd->arg1_l)
         {
          configend_sendonline=0;             
          memset(&sysdef,0,sizeof(sysdef));
          UartMessageInitlist(); 
          Massage_Send_4halfword((0x07<<8)|(0x01),0x01,0,0);              
         }
        else
        {

          Massage_Send_4halfword((0x07<<8)|(0x01),0x00,0,0); 
          configclose_delay=0;
          configclose_flag=1;           
        }
        break;
  }

}



void SonBoardCmdDeal(NEWCMDMASSAGE *cmd)
{
 uint8_t buftx[8]={0,0,0,0,0,0,0,0},i;
  switch(cmd->cmd_data)
  {
    case CMD_SON_GET_UID://子板链路上只能插一个模块
         /*buftx[0]=CONFIGCMD;   
         buftx[1]=CONFIGCMD_ID_0XFF;
         buftx[2]=cmd->arg1_l; 
         UartMessagePushTxlist(UART_LEVEL0,HOST1,cmd->arg1_l,FUNC_01_ONLYSEND,buftx,3);  */
    
         buftx[0]=CONFIGCMD;   
         buftx[1]=CONFIGCMD_GETUID;
         buftx[2]=cmd->arg1_l; 
         buftx[3]=0; 
         UartMessagePushTxlist(UART_LEVEL0,HOST1,cmd->arg1_l,FUNC_02,buftx,4);  
         buftx[3]=1; 
         UartMessagePushTxlist(UART_LEVEL0,HOST1,cmd->arg1_l,FUNC_02,buftx,4);   
         buftx[3]=2; 
         UartMessagePushTxlist(UART_LEVEL0,HOST1,cmd->arg1_l,FUNC_02,buftx,4);      
        break;    
    
    case CMD_SON_GET_HARDWARE:
        
        break;
    
    case CMD_SON_GET_SOFTVER:
        
        break;
    
    case CMD_SON_GET_VI1:
        
        break;
    
    case CMD_SON_GET_VCOLLECT:
        
        break;
    
    case CMD_SON_GET_INPUT:
        for(i=0;i<N_PERIPH;i++) 
          {
           if(sysdef.periph[i].id!=0)
            Massage_Send_4halfword((0x02<<8)|(0x04),sysdef.periph[i].id,sysdef.periph[i].input_sts,0); 
          }              
        break;
    
    case CMD_SON_SET_OUTPUT:
        buftx[0]=CHECKCMD;//cmd
        buftx[1]=SETCMD_OUTPUT;
        buftx[2]=cmd->arg1_l; 
        buftx[3]=cmd->arg2_l;
        buftx[4]=cmd->arg1_h;               
        UartMessagePushTxlist(UART_LEVEL0,LookUpHostById(cmd->arg1_l),cmd->arg1_l,FUNC_02,buftx,5);     
        break;
    
    case CMD_SON_SET_VHL:
        buftx[0]=CONFIGCMD;//cmd
        buftx[1]=CONFIGCMD_SETVAL;
        buftx[2]=cmd->arg1_l; //id
        buftx[3]=cmd->arg1_h;//通道
        buftx[4]=cmd->arg2_l;
        buftx[5]=cmd->arg2_h;      
        UartMessagePushTxlist(UART_LEVEL0,LookUpHostById(cmd->arg1_l),cmd->arg1_l,FUNC_02,buftx,6);     
    
        break;
    
    case CMD_SON_SET_ALARM_IRQLINE1:
        buftx[0]=CONFIGCMD;//cmd
        buftx[1]=CONFIGCMD_PIN_PURPOSE; 
        buftx[2]=cmd->arg1_l;//id
        buftx[3]=cmd->arg2_l;//引脚号
        buftx[4]=cmd->arg2_h;//用途 
        buftx[5]=cmd->arg3_l;//边沿触发  0：下降沿触发   1：上升沿触发  2：双边沿触发   
        UartMessagePushTxlist(UART_LEVEL0,LookUpHostById(cmd->arg1_l),cmd->arg1_l,FUNC_02,buftx,6);     
        break;
    case CMD_SON_SET_ALARM_REMAP://
        buftx[0]=CONFIGCMD;//cmd
        buftx[1]=CONFIGCMD_PIN_REMAP;
        buftx[2]=cmd->arg1_l;//id
        buftx[3]=cmd->arg1_h;//中断线
        buftx[4]=cmd->arg2_l;//引脚号
        buftx[5]=cmd->arg2_h;//是否印射
        UartMessagePushTxlist(UART_LEVEL0,LookUpHostById(cmd->arg1_l),cmd->arg1_l,FUNC_02,buftx,6);     
        break;
    case CMD_SON_GET_BOARDTYPE:
        buftx[0]=CONFIGCMD;//cmd
        buftx[1]=CONFIGCMD_GETBOARDTYPE;
        buftx[2]=cmd->arg1_l; //id   
        UartMessagePushTxlist(UART_LEVEL0,HOST1,cmd->arg1_l,FUNC_02,buftx,3); 

        buftx[0]=CONFIGCMD;//cmd
        buftx[1]=CONFIGCMD_GETBOARDTYPE;
        buftx[2]=cmd->arg1_l; //id   
        UartMessagePushTxlist(UART_LEVEL0,HOST2,cmd->arg1_l,FUNC_02,buftx,3);      
        break;
    
   
  }
}


void SonMotBoardCmdDeal(NEWCMDMASSAGE *cmd)
{
   switch(cmd->cmd_data)
  {
    case CMD_MOT_SET_RESETSPEED:
        
        break;
    
    case CMD_MOT_SET_RUNSPEED:
        
        break;
    
    case CMD_MOT_SET_RESETDIR:
        
        break;
    
    case CMD_MOT_SET_RESETMODE:
        
        break;
    
    case CMD_MOT_SET_SENSORWIDTH:
        
        break;
    
    case CMD_MOT_SET_RESET:
        
        break;
    
    case CMD_MOT_SET_GOPSO:
        
        break;
    
    case CMD_MOT_GET_CURPOS:
        
        break;
  }
}



void AlarmDataCmdDeal(NEWCMDMASSAGE *cmd)
{ 
  uint8_t buftx[8]={0,0,0,0,0,0,0,0};
 if(cmd->cmd_data==CMD_ALARM_MAIN_PINREPORT) 
  {
  
  }
 else if(cmd->cmd_data==CMD_ALARM_SON_PINREPORT)
 {     
  // Massage_Send_4halfword((0x02<<8)|(0x04),(cmd->arg1_h<<8)| cmd->arg1_l ,sysdef[cmd->arg1_l].periph[cmd->arg1_h].input_sts,0);  
 }
 else if(cmd->cmd_data==CMD_ALARM_CLEAN)
 {
    bit_alarminfor=0;
    alarmlist.alarmflag=0;
    buftx[0]=CHECKCMD;//cmd
    buftx[1]=CLEAN_ALARM;
    UartMessagePushTxlist(UART_LEVEL0,HOST1,0xFF,FUNC_01_ONLYSEND,buftx,2);     
    UartMessagePushTxlist(UART_LEVEL0,HOST2,0xFF,FUNC_01_ONLYSEND,buftx,2); 
 }     
 

}

void CanReciveDataDeal(unsigned char *dat)
{
NEWCMDMASSAGE newcmd; 
 newcmd.cmd_type = dat[0];			
 newcmd.cmd_data = dat[1];
 newcmd.arg1=      (dat[3]<<8)|dat[2];
 newcmd.arg2 =     (dat[5]<<8)|dat[4];
 newcmd.arg3 =     (dat[7]<<8)|dat[6];
 newcmd.arg1_l =   dat[2];
 newcmd.arg1_h =   dat[3];
 newcmd.arg2_l =   dat[4];
 newcmd.arg2_h =   dat[5];
 newcmd.arg3_l =   dat[6];
 newcmd.arg3_h =   dat[7]; 

  switch (newcmd.cmd_type)
   {
      case 0x00:
         SonBoardAllCmdBackDeal(&newcmd);
          break;
      case 0x01://主板相关
          MainBoardCmdDeal(&newcmd);
          break; 
      
      case 0x02://子板相关
          SonBoardCmdDeal(&newcmd);
          break;
      
      case 0x03://电机相关
          SonMotBoardCmdDeal(&newcmd);
          break;
       
      case CMDTYPE_ALARM://报警相关
          AlarmDataCmdDeal(&newcmd);
          break;          
   }        
}




void CanReciveUpgradeDeal(unsigned char whatsys,unsigned char *dat)
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
 
    
  if(list->write==list->read)
    return; 
  
  if(mbh_getState(nhost)!=MBH_STATE_IDLE)
    return;
  if(USART_GetStatus(uarthost[nhost],UsartTxEmpty)==0)
    return ;
  if(USART_GetStatus(uarthost[nhost],UsartTxComplete)==0)
    return ;
  mbh_send(nhost,list->addr[list->read],list->cmd[list->read],list->data[list->read],list->datlen[list->read]); 
  
  list->read=(list->read+1)%CMDLEN;
}


static void UartListCheck(int nhost)
{
  if(mbh_getState(nhost)!=MBH_STATE_IDLE)
    return;
  
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


void DataInit()
{
 memset(&sysdef,0,sizeof(sysdef));
}

void InitList()
{
  CanMessageInitlist();
    
  UartMessageInitlist();
    
  UartCheckBoardOnline();
    
  alarmlist_init();
    
  DataInit();
    
}


void TestConfigPinRemap()
{
  uint8_t buftx[20];  
  buftx[0]=CONFIGCMD;//cmd
  buftx[1]=CONFIGCMD_PIN_REMAP;
  buftx[2]=0x01;//id
  buftx[3]=1;//中断线
  buftx[4]=1;//引脚号
  buftx[5]=1;//是否印射
  UartMessagePushTxlist(UART_LEVEL1,LookUpHostById(0x01),0x01,FUNC_02,buftx,6);     
}

//接收正确,进行解析处理
void mbh_exec(uint8_t idx,uint8_t addr,uint8_t *pframe,uint8_t len)
{
    if(idx>=MBH_MASTER_NUM)
      return;
    switch(pframe[0])
    {
      case GETCMD:    
         if(pframe[1]==GETBOARDTYPE)
          {
           Massage_Send_4halfword((CMD_ALLBACK_GETBOARDTYPE<<8)|(0x00),addr,pframe[3],0);
          } 
         else if(pframe[1]==GETBOARDUID)
         {
           Massage_Send_4halfword((CMD_ALLBACK_GETUID<<8)|(0x00),(pframe[3]<<8)|addr,pframe[4]|(pframe[5]<<8),pframe[6]|(pframe[7]<<8));  //获取uid         
         }
         break;
        
      case CONFIGCMD:
        if(pframe[1]==CONFIGCMD_ASK)
          {         
           Massage_Send_4halfword((0x06<<8)|(0x01),(1<<8)|addr,0,0);  //配置id成功 
           sysdef.periph[sysdef.cnt].whathost=idx;
           sysdef.periph[sysdef.cnt].id=addr;
           sysdef.cnt++;               
           }
        else if(pframe[1]==CONFIGCMD_GETUID)
          {
           Massage_Send_4halfword((0x00<<8)|(0x02),(pframe[3]<<8)|addr,pframe[4]|(pframe[5]<<8),pframe[6]|(pframe[7]<<8));  //获取uid
          }
        else if(pframe[1]==CONFIGCMD_GETBOARDTYPE)
         {
           Massage_Send_4halfword((0x0A<<8)|(0x02),addr,pframe[3],0);  //获子板类型
         }       
          break;
           
      case CHECKCMD:
          if(pframe[1]==CHECKCMD_ONLINE)
          {         
           if(pframe[3])
            {
             Massage_Send_4halfword((CMD_ALARM_SON_PINREPORT<<8)|CMDTYPE_ALARM,pframe[2],pframe[4],0);      
             }
            sysdef.periph[LookUpNumById(pframe[2])].input_sts=(pframe[4]);                      
           }     
          break;     
    }  
}


int CanMessageSendReal(int canid,unsigned char *buf)
{
   int i;
   int flag=0;
   stc_can_txframe_t       stcTxFrame;
   int timover=0;
   stcTxFrame.StdID=canid+mainboardid;
   stcTxFrame.Control_f.DLC = 8;
   stcTxFrame.Control_f.IDE =0;
   stcTxFrame.Control_f.RTR=0;
   stcTxFrame.enBufferSel=CanSTBSel;


   stcTxFrame.Data[0] = buf[0];
   stcTxFrame.Data[1] = buf[1];
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





