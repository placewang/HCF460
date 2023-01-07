#include "upgrade.h"
#include "hc32_ddl.h"
#include "message.h"
#include "logic_lay.h"

unsigned short canid_upgrade_rx=0;
unsigned short canid_upgrade_tx=0;
unsigned char upgradetype=0;
int timdelay_upgrade=0;

void IAP_ResetConfig()
{
/* int i;
 SystemClock_DeInit();
 DISABLE_TMR0();
 Timer_DeInit();
 for(i=0;i<143;i++)
 enIrqResign(i);
    
  PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_CAN, Disable);
  CAN_IrqCmd(CanTxPrimaryIrqEn, 0);
  CAN_IrqCmd(CanRxIrqEn, 0);*/
}

uint32_t JumpAddress;
func_ptr_t JumpToApplication;




void EFM_Program_AllSector(uint32_t u32Addr,uint32_t *dat,uint32_t len)
{
 int i,j;
   __set_PRIMASK(1);
    EFM_Unlock();
    
   if(len/SECTOR_SIZE==0)
    {
    
    for(j=0;j<(len/SECTOR_SIZE);j++)
       {
         EFM_SectorErase(u32Addr+j*SECTOR_SIZE);  
       }
    }
   else
     {
    
    for(j=0;j<((len/SECTOR_SIZE)+1);j++)
       {
         EFM_SectorErase(u32Addr+j*SECTOR_SIZE);  
       }
    }  
    
    for(i=0;i<len;)
     {
      EFM_SingleProgram(u32Addr+i,*dat);
      dat++;
      i+=4;
     }
    EFM_Lock();
    __set_PRIMASK(0);
}

#define POLY16 0x1021
unsigned short CRC16(unsigned char *buf,unsigned long dlen, int poly, int CRCinit)
{
	unsigned char ch;
	int i;
	unsigned short CRC_loc = (unsigned short)CRCinit;
	
	while (dlen--)
	{
		ch = *buf++;
		CRC_loc ^= (((unsigned short) ch) << 8);
		for (i = 0; i < 8; i++)
		{
			if (CRC_loc & 0x8000)
				CRC_loc = (CRC_loc << 1) ^ poly;
			else
				CRC_loc <<= 1;
		}

	}
	return CRC_loc;
}


unsigned short arch_crc_ok(unsigned char *buff,unsigned short CRCdata,unsigned long datelen,unsigned short *crcd)
{
	unsigned short crc_temp;
	crc_temp = CRC16((unsigned char *)buff,datelen,POLY16,0);
	if(crcd)
		*crcd = crc_temp;
	if (crc_temp==CRCdata)
	{
		return 1;
	}
	else
		return 0;

}



#define SLEN  (SONBOARD_SECTOR_SIZE)
unsigned char recivebuf[SLEN];//预留8K缓存  一个扇区


static void SonBoardSendUpGradeSucc()
{
 /* unsigned char sendbuf[8]={0,0,0,0,0,0,0,0};
  uint8_t i;

  for(i=0;i<N_PERIPH;i++)
    {
     if((sysdef.periph[i].has==1)&&(sysdef.periph[i].type==boardupgrade.curtype))
      {
        sysdef.periph[i].upgradesuccess_flag=0;
        sendbuf[0]=CMDUART_COM_UPGCHECK;          
        UartMessagePushTxlist(UART_LEVEL0,LookUpHostById(sysdef.periph[i].id),sysdef.periph[i].id,FUNC_02,sendbuf,2);          
      }   
    }*/

}



void NewSonBoardUpGradeLoop()
{
 /* static uint16_t looptik=0;
  uint8_t sendbuf[8]={0,0,0,0,0,0,0,0},buff_rx[8]={0,0,0,0,0,0,0,0},uartdat[8]={0,0,0,0,0,0,0,0},sData[8]={0,0,0,0,0,0,0,0},upgade_err=0;
 _UPGRADE *upg=&boardupgrade;
  uint16_t reciveid=0;
  static uint8_t recordcnt=0;
  static uint32_t recivecnt=0;
  switch(upg->step)
  {
      case UPGRADE_PHASE_PREPARE:
           if(boardupgrade.delays>100)//10ms
           {
            upg->step=UPGRADE_PHASE_GOBOOT;
            boardupgrade.delays=0;
           }
           break;
           
      case UPGRADE_PHASE_GOBOOT:
          if(boardupgrade.app_or_boot==0)//升级APP
           {
            uartdat[0]=CMDUART_COM_JUMTOAPPBY_BOAEDTYPE;
            uartdat[1]=boardupgrade.curtype&0xFF;
            uartdat[2]=(boardupgrade.curtype>>8);
            UartMessagePushTxlist(UART_LEVEL0,HOST1,0xFF,FUNC_01_ONLYSEND,uartdat,3);
            UartMessagePushTxlist(UART_LEVEL0,HOST2,0xFF,FUNC_01_ONLYSEND,uartdat,3);                  
           }
          else
            InitAllSonBoardGoApp();
          
           upg->step=UPGRADE_PHASE_WAITAPPBOOT;
           upg->delays=0;           
           break;
            
      case UPGRADE_PHASE_WAITAPPBOOT:
          if(upg->delays>50)//10ms
          {
           uartdat[0]=CMDUART_COM_CLEANAPP_CODE;
           uartdat[1]=boardupgrade.curtype&0xFF;
           uartdat[2]=(boardupgrade.curtype>>8);
           uartdat[3]=boardupgrade.app_or_boot;
           UartMessagePushTxlist(UART_LEVEL0,HOST1,0xFF,FUNC_01_ONLYSEND,uartdat,4);
           UartMessagePushTxlist(UART_LEVEL0,HOST2,0xFF,FUNC_01_ONLYSEND,uartdat,4);                    
           upg->step=UPGRADE_PHASE_ERASE;
           upg->delays=0;  
           }   
           break;
        
      case UPGRADE_PHASE_ERASE:
          if(upg->delays>50)//2000ms擦除  20k空间
            {
             upg->step=UPGRADE_PHASE_STARTUPG;  
             upg->delays=0;
             recivecnt=0;                
            }     
           break;
            
      case UPGRADE_PHASE_STARTUPG:
           while(1)
           {               
             if(CanMessagePoplist(&canrx_cmdlist,buff_rx)<=0) 
               continue; 
             if(buff_rx[0]==UPGRADE_REQUEST)
               {                 
               sendbuf[0]=UPGRADE_REQUEST;
               sendbuf[3]=1;
               sendbuf[4]=NO_ERROR_2;
               CanMessageSendReal(CAN_TX_ID+boardinfo.id,sendbuf);  
               continue;
               }                   
             if(buff_rx[0]==UPGRADE_ENDDATA)
               {
                uartdat[0]=CMDUART_COM_UPGEND;
                uartdat[1]=buff_rx[2]; 
                uartdat[2]=buff_rx[3]; 
                uartdat[3]=buff_rx[4]; 
                uartdat[4]=buff_rx[5];                    
                UartMessagePushTxlist(UART_LEVEL0,HOST1,0xFF,FUNC_01_ONLYSEND,&buff_rx[2],5);
                UartMessagePushTxlist(UART_LEVEL0,HOST2,0xFF,FUNC_01_ONLYSEND,&buff_rx[2],5);     
                //SpiSendToAllSpi(0xFF,CMDSPI_COM_UPGEND,&buff_rx[2],4,0); 
                upg->step=UPGRADE_PHASE_ENDUPG;  
                upg->delays=0;
                recordcnt=buff_rx[6];
                break;                   
               }
             else
              {
                uartdat[0]=CMDUART_COM_UPGDATA;
                uartdat[1]=buff_rx[2]; 
                uartdat[2]=buff_rx[3]; 
                uartdat[3]=buff_rx[4]; 
                uartdat[4]=buff_rx[5];                    
                UartMessagePushTxlist(UART_LEVEL0,HOST1,0xFF,FUNC_01_ONLYSEND,&buff_rx[2],5);
                UartMessagePushTxlist(UART_LEVEL0,HOST2,0xFF,FUNC_01_ONLYSEND,&buff_rx[2],5); 
             //  SpiSendToAllSpi(0xFF,CMDSPI_COM_UPGDATA,&buff_rx[2],4,0);  
               sendbuf[0]=buff_rx[0];
               sendbuf[3]=buff_rx[6]+1;
               sendbuf[4]=upgade_err;
               CanMessageSendReal(CAN_TX_ID+boardinfo.id,sendbuf);  
                recivecnt+=4;
                             
              }
               looptik=10;
               while(looptik--)
               {
                UartListPop();
               }
                if(recivecnt>=512)
                {
                 recivecnt=0;
                 Ddl_Delay1ms(120); 
                }   
           }
          break;
      case UPGRADE_PHASE_ENDUPG:
             if(upg->delays>2500)//500ms
             {
               SonBoardSendUpGradeSucc();
               upg->delays=0;
               upg->step=UPGRADE_PHASE_CHECKSUCCESS;
             }
          break;
           
      case UPGRADE_PHASE_CHECKSUCCESS:
           if(upg->delays>500)//100ms
            {
             SonBoardCheckUpGradeSucc(recordcnt);
             upg->step=0;
          //  configend_sendonline=1;
            }
          break;                     
  }  */   
}




void UpgradeInitDeal(uint16_t boardtype)
{
  int i;
  uint8_t sendbuf[8]={0,0,0,0,0,0,0,0};
  boardupgrade.curtype=boardtype&0x0FFF;
   
  for(i=0;i<BOARDTYPE_N;i++)
    {
     if(boarddef[i].hardware==boardupgrade.curtype)
        break;
    }
  if(i==BOARDTYPE_N)
   {
    sendbuf[0]=0xE1;
    sendbuf[3]=1;
    sendbuf[4]=NO_UPGRADE;
     CanMessageSendReal(CAN_TX_ID+boardinfo.id,sendbuf); 
    return;
   }
  
  if((boardtype>>12)==0x08)//升级APP
   {       
       if(boardupgrade.curtype==(boardinfo.hardware[0]|(boardinfo.hardware[1]<<8)))//升级主板
          {
            EFM_EraseProgram(ADDR_UPGRADE_START,CMD_UPGRADE_START);
            Ddl_Delay1ms(10);
            __NVIC_SystemReset();  
          }
         else
          {        
            boardupgrade.step=UPGRADE_PHASE_PREPARE;
            boardupgrade.delays=0;
            boardupgrade.app_or_boot=0;
           // configend_sendonline=0;//关闭子板在线检测
          }          
    }
   
 if((boardtype>>12)==0x07)
   {
        if(boardupgrade.curtype==(boardinfo.hardware[0]|(boardinfo.hardware[1]<<8)))//升级主板
          {
           //主板暂不支持升级BOOT
          }
         else
          {        
            boardupgrade.step=UPGRADE_PHASE_PREPARE;
            boardupgrade.delays=0;
            boardupgrade.app_or_boot=1;
          //  configend_sendonline=0;//关闭子板在线检测
          }      
   }
}






















