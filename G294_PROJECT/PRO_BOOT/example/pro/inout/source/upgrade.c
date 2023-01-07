#include "upgrade.h"
#include "hc32_ddl.h"
#include "message.h"

unsigned short canid_upgrade_rx=0;
unsigned short canid_upgrade_tx=0;
int timdelay_upgrade=0;

void IAP_ResetConfig()
{
 int i;
 SystemClock_DeInit();
 PORT_DeInit();
 DISABLE_TMR0();
 Timer_DeInit();
 for(i=0;i<143;i++)
 enIrqResign((IRQn_Type)i);
    
  PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_CAN, Disable);
  CAN_IrqCmd(CanTxPrimaryIrqEn,(en_functional_state_t)0);
  CAN_IrqCmd(CanRxIrqEn, (en_functional_state_t)0);
}

uint32_t JumpAddress;
func_ptr_t JumpToApplication;

static en_result_t IAP_JumpToApp(uint32_t u32Addr)
{
    uint32_t u32StackTop = *((__IO uint32_t *)u32Addr);

    /* Check if user code is programmed starting from address "u32Addr" */
    /* Check stack top pointer. */
    if ((u32StackTop > SRAM_BASE) && (u32StackTop <= (SRAM_BASE + RAM_SIZE)))
    {
        IAP_ResetConfig();
        /* Jump to user application */
        JumpAddress = *(__IO uint32_t *)(u32Addr + 4);
        JumpToApplication = (func_ptr_t)JumpAddress;
        /* Initialize user application's Stack Pointer */
        __set_MSP(*(__IO uint32_t *)u32Addr);
        SCB->VTOR = ((uint32_t) u32Addr & SCB_VTOR_TBLOFF_Msk);
        __set_PRIMASK(1);
        JumpToApplication();
    }

    return Error;
}


int EFM_Program_AllSector(uint32_t u32Addr,uint32_t *dat,uint32_t len)
{
 int i,j;
 int ret=0;
   __set_PRIMASK(1);
    EFM_Unlock();
    
   if(len/SECTOR_SIZE==0)
    {
    
    for(j=0;j<(len/SECTOR_SIZE);j++)
       {
         ret=EFM_SectorErase(u32Addr+j*SECTOR_SIZE);  
       }
    }
   else
     {
    
    for(j=0;j<((len/SECTOR_SIZE)+1);j++)
       {
         ret=EFM_SectorErase(u32Addr+j*SECTOR_SIZE);  
       }
    }  
    
    for(i=0;i<len;)
     {
      ret=EFM_SingleProgram(u32Addr+i,*dat);
      dat++;
      i+=4;
     }
    EFM_Lock();
    __set_PRIMASK(0);
   return ret;
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



#define SLEN  (SECTOR_SIZE)
unsigned char recivebuf[SLEN];//预留8K缓存  一个扇区
unsigned char sendbuf[8]={0,0,0,0,0,0,0,0};
void UpgradeLoop()
{
 int allbytes_cnt=0,Count=0,cmd_len=0,ledcnt=0;
 unsigned short reciveid,writesector_cnt=0,crcval=0,crc_recive=0;
 unsigned char num_sys,buff_rx[8]={0,0,0,0,0,0,0,0},lastlen=0;
 unsigned char *pData;
 int ret=0;
 unsigned char upgade_err=NO_ERROR_2;
 
 if(EFM_ReadByAddr(ADDR_UPGRADE_START)==CMD_UPGRADE_START)
    {
   Upgrade:
     sendbuf[0]=UPGRADE_REQUEST;
     sendbuf[1]=9;//buff_rx[1];
     CanMessageSendReal(canid_upgrade_tx,sendbuf);
    
     EFM_EraseProgram(ADDR_UPGRADE_START,CMD_UPGRADE_RCVDATA);    
     
     LED0_ON();  
         
     writesector_cnt=0;  
     Count=0;  
     allbytes_cnt=0;        
     while(1)
     {        
      if(CanMessagePoplist(&canrx_cmdlist,buff_rx)<0) 
       {
        continue;
       }
       

      if(buff_rx[0]==UPGRADE_REQUEST) 
       {
         sendbuf[0]=buff_rx[0];
         sendbuf[1]=buff_rx[1];
         sendbuf[3]=buff_rx[6]+1;
         CanMessageSendReal(canid_upgrade_tx,sendbuf);  
         continue; 
       }           
       
       ledcnt++;
       if(ledcnt>5)
       {
         LED0_TOGGLE();
         ledcnt=0;
       }
       
     if(buff_rx[0]!=UPGRADE_ENDDATA)
      {
         cmd_len = 4;   
         pData=&buff_rx[1];
      do {   
          pData++; 
          
          recivebuf[Count++]=*pData;
         
          allbytes_cnt++;

          if(Count == SLEN)
          {
            LED0_ON();
            ret=EFM_Program_AllSector(ADDR_APP_BACKUP+SECTOR_SIZE*writesector_cnt,(uint32_t *) &recivebuf,SECTOR_SIZE);
            if(ret)
            {
             upgade_err=FLASH_ERROR;
             break;
            }
            writesector_cnt++;
            LED0_OFF();
            Count=0;
          } 
          
         }while(--cmd_len);
      }
     else//带校验下来的最后一帧
         {
          if(Count)
              {
                 ret=EFM_Program_AllSector(ADDR_APP_BACKUP+SECTOR_SIZE*writesector_cnt,(uint32_t *) &recivebuf,Count); 
                  if(ret)
                   {
                   upgade_err=FLASH_ERROR;
                   }                  
                 crc_recive=buff_rx[2]|(buff_rx[3]<<8);
              }
              
           if(!arch_crc_ok((unsigned char *)ADDR_APP_BACKUP,crc_recive,allbytes_cnt-8,&crcval))
               { 
                 upgade_err=CRC_ERROR_2;  
                 LED0_OFF();
                 break;
               }
              
               LED0_ON();
               
               EFM_EraseProgram(ADDR_UPGRADE_START,CMD_UPGRADE_BURN);

              ret=EFM_Program_AllSector(APP_RUN_ADDRESS,(uint32_t *)ADDR_APP_BACKUP,allbytes_cnt-8); 
                 if(ret)
                   {
                    upgade_err=FLASH_ERROR;
                   } 

               EFM_EraseProgram(ADDR_UPGRADE_START,CMD_UPGRADE_SUCCESS);                          
               LED0_OFF();  
              sendbuf[0]=buff_rx[0];
              sendbuf[1]=buff_rx[1];
              sendbuf[2]=upgade_err;
              sendbuf[3]=buff_rx[6]+1;

              CanMessageSendReal(canid_upgrade_tx,sendbuf);  
              break;                              
        }
         
         sendbuf[0]=buff_rx[0];
         sendbuf[1]=buff_rx[1];
         sendbuf[2]=upgade_err;
         sendbuf[3]=buff_rx[6]+1;      
         CanMessageSendReal(canid_upgrade_tx,sendbuf);  
        if(upgade_err)
           break;        
      }
     
   }
  else if(EFM_ReadByAddr(ADDR_UPGRADE_START)==CMD_UPGRADE_BURN)
   {
     
        EFM_Program_AllSector(APP_RUN_ADDRESS,(uint32_t *)ADDR_APP_BACKUP,allbytes_cnt-2);                
        
       EFM_EraseProgram(ADDR_UPGRADE_START,CMD_UPGRADE_SUCCESS);               
                  
      LED0_OFF();
      Ddl_Delay1ms(10);
   }
   	else {
		int delay = 10000;
       // sendbuf[0]=UPGRADE_FORCE_REQ;
       // sendbuf[3]=buff_rx[6]+1;
       // CanMessageSendReal(canid_upgrade_tx,sendbuf);  
		LED0_ON();
        timdelay_upgrade=0;
		while(1) 
            {
			   if(CanMessagePoplist(&canrx_cmdlist,buff_rx)<0) 
                 {
                  if(timdelay_upgrade>=10) {//10ms
					break;
				    }                
                  continue;
                 }   
	           /*else
               {
                if(buff_rx[0] == UPGRADE_FORCE_REQ) 
                 break;
               }*/
			}

			/*if(buff_rx[0] == UPGRADE_FORCE_REQ) 
			{
				goto Upgrade;
			}*/
		}
    
        
     LED0_OFF();


     IAP_JumpToApp(APP_RUN_ADDRESS); 

}
























