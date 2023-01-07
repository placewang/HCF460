#include "logic_lay.h"
#include "upgrade.h"
#include "mot.h"
#include "stdlib.h"
#include "motordemand.h"
P_PERPHA perpha_para[N_PARA];
_VALVE_CONFIG valveconfig;
uint8_t isconfiged=0;
uint32_t time_led=0;
uint32_t time_valve=0;
uint32_t read_status_cnt =0 ;	
uint32_t time_cnt =0 ;
uint8_t valvests[N_MAX_VALVE];
const uint8_t app_ver[] __attribute__((at(BOARD_VER_APP))) ={APP_VER_L,APP_VER_H};
uint8_t  mainarry_uid[12] __attribute__((at(ADDR_UID))); 
NEWSYSCONFIG sysdef;
_UPGRADE boardupgrade;
uint8_t lift_mot_state = LIFT_STOP ;
uint8_t litf_mot_run_dir_flag = LIFT_RUN_IDLE_FLAG ;
uint8_t mot_r_limit = 0 ;
uint8_t auto_send = 0 ;
uint32_t acc_dis_time_ms = 26 ; //10ms
_ALARMCONTENT alarmcontent[ALARM_MAX]=
{
 {ALARM_SPI1_ERR,0,0,"电机1SPI通讯报警","motor 1 spi err"},
 {ALARM_SPI2_ERR,0,0,"电机2SPI通讯报警","motor 2 spi err"}, 
 {ALARM_ALARM8803_ERR,0,0,"8803输出过流报警","chip8803 output over current"},
 {ALARM_GOIN_ERR,0,0,"电机复位走完未找到传感器","no sensor check at mot reset in"},
 {0,0,0,"",""},
 {0,0,0,"",""},
 {0,0,0,"",""},
 {0,0,0,"",""},
 {0,0,0,"",""},
 {0,0,0,"",""},
 {0,0,0,"",""},
 {0,0,0,"",""},
 {0,0,0,"",""},
 {0,0,0,"",""},
 {0,0,0,"",""},
 {0,0,0,"",""},
 {0,0,0,"",""},
};

extern unsigned short motorstatetime;

//mdbus接收正确,进行解析处理
void mbh_exec(uint8_t idx,uint8_t addr,uint8_t *pframe,uint8_t len)
{
	
}

void SonBoardAllCmdBackDeal(NEWCMDMASSAGE *cmd)
{

}


static void MainBoardCmdDeal(NEWCMDMASSAGE *cmd)
{
 
  switch(cmd->cmd_data)
  {
    
    case CMD_BOARD_GOIN_CONFIG:
         if(cmd->arg1_l)
         {
       //   configend_sendonline=0;             
          SysConfigInit();
          UartMessageInitlist(); 
          Massage_Send_4halfword((0x07<<8)|(0x01),0x01,0,0);              
         }
        else
        {

          Massage_Send_4halfword((0x07<<8)|(0x01),0x00,0,0); 
         // configclose_delay=0;
         // configclose_flag=1;           
        }
        break;
        
    case CMD_SONBOARD_GOTOAPP:
       
        break;
  }

}



static void SonMotBoardCmdDeal(NEWCMDMASSAGE *cmd)
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



static void AlarmDataCmdDeal(NEWCMDMASSAGE *cmd)
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
    int i;
   // poweroff_flag=0;
    alarmlist.alarmflag=0;
     
   for(i=0;i<N_PERIPH;i++) 
    {
      if(sysdef.periph[i].id!=0)
        Massage_Send_4halfword((0x02<<8)|(0x04),sysdef.periph[i].id,sysdef.periph[i].input_sts,0); 
    }              
  }     
}

#include "hc32f46x_utility.h"
static void OtherCmdDeal(NEWCMDMASSAGE *cmd)
{
  switch(cmd->cmd_data)
  {
    case CMD_OTHERSET_REQUEST:
          boardupgrade.curtype=cmd->arg2;
         if(boardupgrade.curtype==UPGRADE_BOARDTYPE_E785)
          {
            EFM_EraseProgram(ADDR_UPGRADE_START,CMD_UPGRADE_START);
            Ddl_Delay1ms(10);
            __NVIC_SystemReset();  
          }
         else
         {        
            boardupgrade.step=UPGRADE_PHASE_PREPARE;
            boardupgrade.delays=0;
         }                 
         break;
  }
}


static void MainBoardComParaGet(uint8_t p_num)
{
  switch(p_num)
  {
      case P_MAIN_BOARDTYPE:
           Massage_Send_4halfword(PARA_SETORGET|((0x80|0)<<8),p_num|(P_CANNOTSET<<7),boardinfo.hardware[0]|((boardinfo.hardware[1]&0x0F)<<8),0);   
           break;
      case P_MAIN_UID1:
           Massage_Send_4halfword(PARA_SETORGET|((0x80|0)<<8),p_num|(P_CANNOTSET<<7),boardinfo.uid[0]|(boardinfo.uid[1]<<8),boardinfo.uid[2]|(boardinfo.uid[3]<<8));
           break;
      case P_MAIN_UID2:
           Massage_Send_4halfword(PARA_SETORGET|((0x80|0)<<8),p_num|(P_CANNOTSET<<7),boardinfo.uid[4]|(boardinfo.uid[5]<<8),boardinfo.uid[6]|(boardinfo.uid[7]<<8));  
           break;
      case P_MAIN_UID3:
           Massage_Send_4halfword(PARA_SETORGET|((0x80|0)<<8),p_num|(P_CANNOTSET<<7),boardinfo.uid[8]|(boardinfo.uid[9]<<8),boardinfo.uid[10]|(boardinfo.uid[11]<<8));
           break;
      case P_MAIN_VHARDWARE:
           Massage_Send_4halfword(PARA_SETORGET|((0x80|0)<<8),p_num|(P_CANNOTSET<<7),boardinfo.hardware[2]|(boardinfo.hardware[3]<<8),0);
           break;
      case P_MAIN_VSOFTWARE_BOOT:
           Massage_Send_4halfword(PARA_SETORGET|((0x80|0)<<8),p_num|(P_CANNOTSET<<7),boardinfo.boot_ver[0]|(boardinfo.boot_ver[1]<<8),0);
           break;
      case P_MAIN_VSOFTWARE_APP:
           Massage_Send_4halfword(PARA_SETORGET|((0x80|0)<<8),p_num|(P_CANNOTSET<<7),boardinfo.app_ver[0]|(boardinfo.app_ver[1]<<8),0);
           break; 
  }
}


static void MainBoardMotParaGet(uint8_t p_num,uint8_t devid)
{
 uint8_t motid=0,i;
 uint8_t sensorsts=0;
 if((devid==DEVID_MOT1)||(devid==DEVID_MOT2))
  {
   motid=devid-DEVID_MOT1;
  switch(p_num)
  {      
     case  P_MOT_SUB_1:
           Massage_Send_4halfword(PARA_SETORGET|((0x00|devid)<<8),p_num|(P_CANSET<<7),(mot[motid].xf&0xFF)|((mot[motid].xf>>8)<<8),0);        
          break;
     case  P_MOT_POS_1:
           Massage_Send_4halfword(PARA_SETORGET|((0x00|devid)<<8),p_num|(P_CANSET<<7),(mot[motid].pos&0xFF)|((mot[motid].pos>>8)<<8),0);        
          break;
     case  P_MOT_ZWREMAP_1:
                   
          break;
     case  P_MOT_SENSOR_1:
           if(zerosignal(motid))
             sensorsts|=(1<<0);
           else
             sensorsts &=~(1<<0);
           
           if(mot_leftlimit_get(motid))
              sensorsts|=(1<<1);
           else
             sensorsts &=~(1<<1);
           
           if(mot_rightlimit_get(motid))
              sensorsts|=(1<<2);
           else
             sensorsts &=~(1<<2);
           
           Massage_Send_4halfword(PARA_SETORGET|((0x00|devid)<<8),p_num|(P_CANNOTSET<<7),sensorsts,0);                    
          break;
     case  P_MOT_ZWIDTH_1:
           
          break;
     case  P_MOT_WWIDTH_1:
               
          break;
     case  P_MOT_VRUN_1:
          Massage_Send_4halfword(PARA_SETORGET|((0x00|devid)<<8),p_num|(P_CANSET<<7),(mot[motid].run_speed&0xFF)|((mot[motid].run_speed>>8)<<8),0);           
          break;
     case  P_MOT_VRESET_1:
          Massage_Send_4halfword(PARA_SETORGET|((0x00|devid)<<8),p_num|(P_CANSET<<7),(mot[motid].reset_speed&0xFF)|((mot[motid].reset_speed>>8)<<8),0);     
          break;
     case  P_MOT_SRESET_1:
          Massage_Send_4halfword(PARA_SETORGET|((0x00|devid)<<8),p_num|(P_CANSET<<7),(mot[motid].reset_maxstep&0xFF)|((mot[motid].reset_maxstep>>8)<<8),0); 
          break;
     case P_MOT_RUNMODE_1:
                   
          break;
     case P_MOT_RUN_GOZEROMODE_1:
                 
          break;
     case P_MOT_RUN_SENSORMODE_1:
                  
          break;
     case P_MOT_RESETMODE_1:
                 
          break;
     case P_MOT_ISFAST_1:
                   
          break;
     case P_MOT_SRESETIN_MAX_1:
                   
          break;
     case P_MOT_SRESETOUT_MAX_1:
                   
          break; 
     case P_MOT_RESETOUT_CHECKEN_1:
               
          break; 
    }
  }
 else if(devid==DEVID_VALVE)
 {
    switch(p_num)
      {
     case  P_VALVE_LEFTCONFIG:
           Massage_Send_4halfword(PARA_SETORGET|((0x00|(devid))<<8),p_num|(P_CANSET<<7),(valveconfig.left_ngroup&0xFF)|((valveconfig.left_ngroup>>8)<<8),0); 
          break;
     
     case  P_VALVE_RIGHTCONFIG:
           Massage_Send_4halfword(PARA_SETORGET|((0x00|(devid))<<8),p_num|(P_CANSET<<7),(valveconfig.right_ngroup&0xFF)|((valveconfig.right_ngroup>>8)<<8),0); 
          break;
     }
 }
 else if(devid==DEVID_INOUT)
 {
  switch(p_num)
      {
     case  P_INPUT_STS:
         for(i=0;i<INPUT_N;i++)
           {
          if(inputsignal(i))
             sensorsts|=(1<<i);
           else
             sensorsts &=~(1<<i);
           }
           Massage_Send_4halfword(PARA_SETORGET|((0x00|(devid))<<8),p_num|(P_CANSET<<7),sensorsts,0); 
          break;
     }
 }
}

static void MainBoardParaSet(uint8_t p_num,uint8_t devid,uint16_t value)
{
 uint8_t motid=0;
  _MOTDEF *smot=NULL;
   if((devid==DEVID_MOT1)||(devid==DEVID_MOT2))
    {
     motid=devid-DEVID_MOT1;
     smot=&mot[motid];
     switch(p_num)
      {
    //电机1参数设置
		 case P_MOT_D_SPEED:
			    mot_d_spd_config(motid,value); 
				 break;
     case  P_MOT_DIR_1:
          mot_dir_config(motid,value ? 1:0);
          break;
     case  P_MOT_CUR_1:
          mot_current_set(motid,value);        
          break;
     case  P_MOT_SUB_1:
          mot_xf_config(motid,value);         
          break;
     case  P_MOT_POS_1:
          mot_curpos_set(motid,value);        
          break;
		 case p_MOT_V1:
			    mot_V1_set(motid,value); 
			    break;
		 case p_MOT_A1:
			    mot_A1_set(motid,value); 
			    break;
		 case p_MOT_AMAX:
			     mot_AMAX_set(motid,value); 
			    break;
		 
		 case p_MOT_DMAX:
			     mot_DMAX_set(motid,value); 
			    break;
		 case p_MOT_D1:
			     mot_D1_set(motid,value); 
			    break;
		 case p_MOT_VSTOP:
			     mot_VStop_set(motid,value); 
			    break;
		 case p_MOT_ACC_TIME:
					acc_dis_time_ms = value;
					break;
     case  P_MOT_ZWREMAP_1:
                   
          break;
     case  P_MOT_SENSOR_1:
           
          break;
     case  P_MOT_ZWIDTH_1:
           
          break;
     case  P_MOT_WWIDTH_1:
               
          break;
     case  P_MOT_VRUN_1:
          smot->run_speed=value;        
          break;
     case  P_MOT_VRESET_1:
          smot->reset_speed=value;        
          break;
     case  P_MOT_SRESET_1:
          smot->reset_maxstep=value; //      
          break;
     case P_MOT_RUNMODE_1:
                   
          break;
     case P_MOT_RUN_GOZEROMODE_1:
                 
          break;
     case P_MOT_RUN_SENSORMODE_1:
                  
          break;
     case P_MOT_RESETMODE_1:
                 
          break;
     case P_MOT_ISFAST_1:
                   
          break;
     case P_MOT_SRESETIN_MAX_1:
                   
          break;
     case P_MOT_SRESETOUT_MAX_1:
                   
          break; 
     case P_MOT_RESETOUT_CHECKEN_1:
               
          break;
         }       
  }
 else if(devid==DEVID_VALVE)
 {
    switch(p_num)
      {
     case  P_VALVE_LEFTCONFIG:
          valveconfig.left_ngroup=value;
          break;
     
     case  P_VALVE_RIGHTCONFIG:
          valveconfig.right_ngroup=value;
          break;
    }
 }
}


void CanSendAlarmInfoByPack(uint8_t id,uint8_t *packdat,uint8_t datlen)
{
  uint8_t i,packlen=0,index_len=0;
  datlen=datlen+1;//加个\0
    
 if((datlen%5)==0)
   packlen=datlen/5;
 else
   packlen=datlen/5+1;  
  for(i=0;i<packlen;i++)
  {
		if(i<packlen-1)      
			index_len=(i<<4)|5;
		else
		{
			if((datlen%5)==0)
				index_len=(i<<4)|5;
		else
      index_len=(i<<4)|(datlen%5);  
   } 
   Massage_Send_4halfword(ALARM_INFO_CAN_CMD|(id<<8),index_len|(packdat[i*5+0]<<8),packdat[i*5+1]|(packdat[i*5+2]<<8),packdat[i*5+3]|(packdat[i*5+4]<<8));
  }
}


void AlarmInfoInit()
{
   uint8_t i;
    
   for(i=0;i<ALARM_MAX;i++)
    {
     if(alarmcontent[i].index==0)
        break;
    }
      
   for(i=0;i<ALARM_MAX;i++)
    {
     alarmcontent[i].bytes_ch=strlen((char *)alarmcontent[i].info_ch);
     alarmcontent[i].bytes_en=strlen((char *)alarmcontent[i].info_en);
    }
}





void CanReciveDataDeal(unsigned char *dat)
{
 NEWCMDMASSAGE newcmd; 
 static uint8_t boardid_get=0;
 uint16_t para_charact_id;
 uint8_t dir =0 ;
 int32_t spd =0 ;
 uint8_t alarmbytelen=0,arry_uid[5]={0,0,0,0,0},rlen=0,buftx[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
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
 para_charact_id= newcmd.arg1;
     
  if((newcmd.cmd_data<DEVID_MOT2)&&(newcmd.cmd_data>DEVID_INOUT))
   return;
     
  switch (newcmd.cmd_type)
   {
      case 0x01://电机复位
         if(newcmd.cmd_data==DEVID_MOT1)
           mot_runreset(0);
         else
           mot_runreset(1);                           
          break;
         
      case 0x07://电机使能 *
          if(newcmd.cmd_data==DEVID_MOT1)
              mot_enable(0,(newcmd.arg1_l ? 0:1));
          else if(newcmd.cmd_data==DEVID_MOT2)
              mot_enable(1,(newcmd.arg1_l ? 0:1));              
          break;
      
      case 0x08://清除报警 *
          alarmlist.alarmflag=0;     
          break;     
      case 0x0A://电机跑位置*
         if(newcmd.cmd_data==DEVID_MOT1)
           mot_runpos_mode(0,newcmd.arg2|(newcmd.arg3<<16));
         else if(newcmd.cmd_data==DEVID_MOT2)
           mot_runpos_mode(1,newcmd.arg2|(newcmd.arg3<<16));  
          break; 
      case 0x0e://相对位置
					if(newcmd.cmd_data==DEVID_MOT1)
           mot_run_relative_pos_mode(0,newcmd.arg1,newcmd.arg2|(newcmd.arg3<<16));
         else if(newcmd.cmd_data==DEVID_MOT2)
           mot_run_relative_pos_mode(1,newcmd.arg1,newcmd.arg2|(newcmd.arg3<<16)); 
					break;
      case 0x0B ://电机速度模式*
				  spd = newcmd.arg2|(newcmd.arg3<<16) ;
					if(newcmd.cmd_data==DEVID_MOT1)
						mot_runbysped_mode(0,1,spd) ;
					else if(newcmd.cmd_data==DEVID_MOT2)
						mot_runbysped_mode(0,1,spd) ;
				  break;
			case 0x11: //电机停止运行
					mot_runbysped_mode(0,1,0) ;
				  break;
			case 0x71:
//				  auto_send = 1 ;
          break;		
      case 0xD3://**气阀动作--扩展口输出
         if(newcmd.cmd_data==DEVID_VALVE)
				 {
					 ValveDriverByNum(newcmd.arg1_l,newcmd.arg1_h);
					 if(newcmd.arg1_l ==141) //下降
					 {
						   if(newcmd.arg1_h==1)
							 litf_mot_run_dir_flag = LIFT_RUN_DOWN_FLAG;
					 }
					 else if(newcmd.arg1_l ==140) //上升
					 {
						   if(newcmd.arg1_h==1)
							 litf_mot_run_dir_flag = LIFT_RUN_UP_FLAG;
					 }					  
				 }
         else if(newcmd.cmd_data==DEVID_INOUT)
          {
           if(newcmd.arg1_l<OUTPUT_N)              
           Drv8803_Out(newcmd.arg1_l,newcmd.arg1_h);            
          }             
          break;
      
      case ALARM_INFO_CAN_CMD:
          if(alarmcontent[newcmd.arg1_l].index!=0)
           {
            if(newcmd.arg1_h==0)//中文
             CanSendAlarmInfoByPack(newcmd.cmd_data,&alarmcontent[newcmd.arg1_l].bytes_ch,strlen((char *)&alarmcontent[newcmd.arg1_l].bytes_ch));
            else
             CanSendAlarmInfoByPack(newcmd.cmd_data,&alarmcontent[newcmd.arg1_l].bytes_en,strlen((char *)&alarmcontent[newcmd.arg1_l].bytes_en));
           }
          break;

      case PARA_SETORGET:
         if(para_charact_id>>15)//获取参数
         {
           if((para_charact_id&0x7F)<0x10)
             MainBoardComParaGet(para_charact_id&0x7F);
           else
             MainBoardMotParaGet(para_charact_id&0x7F,newcmd.cmd_data); 
         }             
         else
           MainBoardParaSet(para_charact_id&0x7F,newcmd.cmd_data,newcmd.arg2);               
          break;      
         
      case 0xE1://进入升级模式
          UpgradeInitDeal(newcmd.arg2);   
         break;         
   }        
}


//////////////////配置ID相关接口////////////////////////

void CheckBoardOnline()
{

}




void Init_ParaList()
{
 int i;
 for(i=0;i<N_PARA;i++)
  {
   perpha_para[i].character_cmdtype |=CMDUART_COM_CONFIGPARA;
   perpha_para[i].boardid=boardinfo.id;
   memset(&perpha_para[i].parameter,0,4);
  } 

 perpha_para[P_BOARD_NUM].character_cmdtype |=(P_CANNOTSET<<7);
 perpha_para[P_BOARD_NUM].cmdnum=P_BOARDTYPE;
 perpha_para[P_BOARD_NUM].parameter[0]=PARA_CNT;  
   
 perpha_para[P_BOARDTYPE].character_cmdtype |=(P_CANNOTSET<<7);
 perpha_para[P_BOARDTYPE].cmdnum=P_BOARDTYPE;
 perpha_para[P_BOARDTYPE].parameter[0]=boardinfo.type;
    
 perpha_para[P_UID1].character_cmdtype|=(P_CANNOTSET<<7);
 perpha_para[P_UID1].cmdnum=P_UID1;
 memcpy(perpha_para[P_UID1].parameter,&boardinfo.uid[0],4);
 
 perpha_para[P_UID2].character_cmdtype|=(P_CANNOTSET<<7);
 perpha_para[P_UID2].cmdnum=P_UID2;
 memcpy(perpha_para[P_UID2].parameter,&boardinfo.uid[4],4);
 
 perpha_para[P_UID3].character_cmdtype|=(P_CANNOTSET<<7);
 perpha_para[P_UID3].cmdnum=P_UID3;
 memcpy(perpha_para[P_UID3].parameter,&boardinfo.uid[8],4);
    
 perpha_para[P_VHARDWARE].character_cmdtype|=(P_CANNOTSET<<7);
 perpha_para[P_VHARDWARE].cmdnum=P_VHARDWARE;
 memcpy(perpha_para[P_VHARDWARE].parameter,boardinfo.hardware,4);
 
 perpha_para[P_VSOFTWARE_BOOT].character_cmdtype|=(P_CANNOTSET<<7);
 perpha_para[P_VSOFTWARE_BOOT].cmdnum=P_VSOFTWARE_BOOT;
 perpha_para[P_VSOFTWARE_BOOT].parameter[0]=boardinfo.boot_ver[0];
 perpha_para[P_VSOFTWARE_BOOT].parameter[1]=boardinfo.boot_ver[1];
 
 perpha_para[P_VSOFTWARE_APP].character_cmdtype|=(P_CANNOTSET<<7);
 perpha_para[P_VSOFTWARE_APP].cmdnum=P_VSOFTWARE_APP;
 perpha_para[P_VSOFTWARE_APP].parameter[0]=boardinfo.app_ver[0];
 perpha_para[P_VSOFTWARE_APP].parameter[1]=boardinfo.app_ver[1]; 
}


void CAN_RxIrqCallBack(void)
{
  stc_can_rxframe_t stcRxFrame; 
  if(true == CAN_IrqFlgGet(CanRxIrqFlg))
  {
      CAN_IrqFlgClr(CanRxIrqFlg);
        
      CAN_Receive(&stcRxFrame);
     
      if(stcRxFrame.StdID==CAN_RX_ID)
        CanMessagePushlist(&canrx_cmdlist,stcRxFrame.StdID,stcRxFrame.Data);             
  }
}

void Tim0_Callback(void)
{
 timcan++; 
 time_led++;
 boardupgrade.delays++;  
 time_valve++; 
 mot[0].delayset++;
 mot[1].delayset++;  
 read_status_cnt++ ;  
 time_cnt ++ ;
 MotMove.MotorMoveCountTime++;
 MotMove.SpeedDwonCountTime++;
 MotMove.SpeedUpCountTime++;
 MotorDataSendTimeCount++;	

}

int  test=0;
void MotResetLoop()
{
 uint32_t i;
 uint32_t checkcnt=0;
 static uint8_t state[2] ={0 ,0};
 _MOTDEF *smot=NULL;
 for(i=0;i<NUM_MOTOR;i++)
  {
     smot=&mot[i];
		 
   if(smot->reset_sts==MOT_RESETSTS_IDLE) 
   {
		 state[i] = 0 ;
		 continue;  
	 }
   switch(state[i])
	 {
		 case 0 :
				if(zerosignal(i)==0)//感应到零位
				{
						mot_runbysped_mode(i,0,0);
						smot->reset_sts=MOT_RESETSTS_REACHZERO;
						smot->delayset=0;  
					  state[i] = 1 ;
				}
			 break;
		 case 1:
				if((smot->reset_sts==MOT_RESETSTS_REACHZERO)&&(smot->delayset>=50))//50ms
				{
					mot_curpos_set(i,0);
					smot->pos=mot_curpos_get(i);
					smot->reset_sts=MOT_RESETSTS_IDLE;
					smot->isrunning=0;  
					smot->reset_flag =1 ;		      			
				}
				break;
	  }		 
    
   
		#if 1
		if((smot->reset_sts==MOT_RESETSTS_GOIN)&&(smot->pos==0))
    {
      mot_runbysped_mode(i,0,0);
      smot->reset_sts=MOT_RESETSTS_IDLE;
      smot->isrunning=0; 
      Alarmlist_Push(&alarmlist,i+DEVID_MOT2,ALARM_GOIN_ERR,0);      
    }  
    #endif		
  }
}

void InitDefaultValveConfig()
{
 valveconfig.left_ngroup=N_VALVEGROUP_DEF;
 valveconfig.right_ngroup=N_VALVEGROUP_DEF;
 memset(valvests,0,N_MAX_VALVE);
}


void ValveDriverChangeBit(uint8_t nspi,uint8_t *TxData,uint8_t len)
{
  uint8_t i;
  uint8_t temp[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  for(i=0;i<len;i++)
  {
		 if(TxData[i]&0x01)//第一个    
				temp[len-i-1] |=0x10;
		 if(TxData[i]&0x02)  
				temp[len-i-1] |=0x01;
		 if(TxData[i]&0x04)  
				temp[len-i-1] |=0x20;
		 if(TxData[i]&0x08)  
				temp[len-i-1] |=0x02;  
		 if(TxData[i]&0x10)  
				temp[len-i-1] |=0x40; 
		 if(TxData[i]&0x20)  
				temp[len-i-1] |=0x04; 
		 if(TxData[i]&0x40)  
				temp[len-i-1] |=0x80; 
		 if(TxData[i]&0x80)  
				temp[len-i-1] |=0x08;       
  }         
  SPI2_3_WriteLen(nspi,temp,len);
}

void ValveDriverByNum(uint16_t num,uint8_t sts)
{
  uint8_t n_whatgroup=0,n_whatnum=0;
  static uint8_t sts_every_leftgroup[8]={0,0,0,0,0,0,0,0},sts_every_rightgroup[8]={0,0,0,0,0,0,0,0};
  if(num>=N_MAX_VALVE)
    return;
  if(valvests[num]==sts)
    return;      
   valvests[num]=sts;
  
 if(num<(8*valveconfig.left_ngroup))
  {
   n_whatgroup=num/N_VALVE_EVERYGROUP;
   n_whatnum=num%N_VALVE_EVERYGROUP;
   if(sts)
    sts_every_leftgroup[n_whatgroup] |=(1<<n_whatnum);
   else
    sts_every_leftgroup[n_whatgroup] &=(~(1<<n_whatnum));   
      
   ValveDriverChangeBit(1,sts_every_leftgroup,valveconfig.left_ngroup);  
  }
 else
  {
   n_whatgroup=(num-(8*valveconfig.left_ngroup))/N_VALVE_EVERYGROUP;
   n_whatnum=(num-(8*valveconfig.left_ngroup))%N_VALVE_EVERYGROUP;
   if(sts)
    sts_every_rightgroup[n_whatgroup] |=(1<<n_whatnum);
   else
    sts_every_rightgroup[n_whatgroup] &=(~(1<<n_whatnum)); 
   
   ValveDriverChangeBit(2,sts_every_rightgroup,valveconfig.right_ngroup);  
  }
}
void CanSendSignal(void)
{
	static uint8_t old_sg = 0xff ;
	uint16_t i,sensorsts=0;
	if(auto_send)
	{
		for(i=0;i<2;i++)
		{
			if(inputsignal(i))
				sensorsts|=(1<<i);
			else
				sensorsts &=~(1<<i);
		}
		if(sensorsts!=old_sg)
		{
				Massage_Send_4halfword((0x71|( boardinfo.id)<<8),sensorsts<<8,0,0); 
				old_sg = sensorsts;
		}
	}	
}


void Init_GetBoardInfo()
{
  int i;
  EFM_EraseProgram(ADDR_BOARD_ID,INIT_BOARDID);
  boardinfo.id=DEVID_INOUT;
  boardinfo.type=*(__IO uint32_t *)ADDR_BOARDTYPE;
  boardinfo.private_canid=0;
  for(i=0;i<12;i++)
    boardinfo.uid[i]=mainarry_uid[i];
  for(i=0;i<4;i++)
    boardinfo.hardware[i] =*(__IO uint8_t *)(MAINBOARD_HARDWARE_ADDR+4*6+i);    
  for(i=0;i<2;i++)
   boardinfo.app_ver[i]=app_ver[i];
  boardinfo.boot_ver[0]=*(__IO uint8_t *)BOARD_VER_ADDR;
  boardinfo.boot_ver[1]=*(__IO uint8_t *)(BOARD_VER_ADDR+1);   
}


void Spi_ErrDeal(uint8_t motid)
{
 Alarmlist_Push(&alarmlist,motid+DEVID_MOT2,ALARM_SPI1_ERR+motid,0);
}

void Init_RegisterSpiErr()
{
 mot_errdeal_register(Spi_ErrDeal);     
}

uint8_t expireTicks(uint32_t oldTicks, uint32_t period)
{
  uint32_t curTicks = time_cnt;
  uint32_t delayTicks;
  
  delayTicks = ((curTicks >= oldTicks) ? (curTicks-oldTicks) : (0xffffffff-oldTicks+curTicks));
  if(delayTicks >= period)
    return 1;

  return 0;
}
void check_rst(void)
{
	  if(abs(mot[0].pos - mot[0].beg_reset_pos) >mot[0].beg_reset_pos && mot[0].beg_reset_pos >0) //检测复位运动最大行程
		{
				 lift_mot_state = LIFT_PAUSE ;
		}
}
void lift_mot_run_loop(void)
{
	  static uint32_t old_tick =0 ;
		switch(lift_mot_state)
		{
			case LIFT_PAUSE:
				mot_runbysped_mode(0,0,0);
			  lift_mot_state = LIFT_STOP;
				break;
			case LIFT_STOP:
				if(litf_mot_run_dir_flag ==LIFT_RUN_UP_FLAG)
				{
					  mot[0].run_speed = mot[0].up_run_spd;
					  if(inputsignal(UP_LIMIT_SIGNAL) ==1)
						{
							  old_tick = time_cnt ;
								lift_mot_state = LIFT_RUN_A ;
					      mot[0].cur_speed =0 ;
								//mot_runbysped_mode(0,0,mot[0].run_speed) ;
						}
						mot[0].beg_reset_pos = mot[0].pos ;
					  litf_mot_run_dir_flag =LIFT_RUN_IDLE_FLAG ;
				}
				else if(litf_mot_run_dir_flag ==LIFT_RUN_DOWN_FLAG)
				{
					  mot[0].run_speed = mot[0].up_run_spd;
					  mot[0].beg_reset_pos = mot[0].pos ;
					  if(inputsignal(DOWN_LIMIT_SIGNAL) ==1)
						{
								//lift_mot_state = LIFT_RUN_DOWN ;
								//mot_runbysped_mode(0,1,mot[0].run_speed) ;
							  old_tick = time_cnt ;
								lift_mot_state = LIFT_RUN_A_DOWN ;
					      mot[0].cur_speed =0 ;
						}
					  litf_mot_run_dir_flag =LIFT_RUN_IDLE_FLAG ;
				}
				break;
			case LIFT_RUN_A:
				check_rst();
				if(expireTicks(old_tick,acc_dis_time_ms))
				{
						mot[0].cur_speed += 5 ;
					  if(mot[0].cur_speed >=mot[0].run_speed)
						{
								lift_mot_state = LIFT_RUN_UP ;
						}
						mot_runbysped_mode(0,0,mot[0].cur_speed);
						old_tick = time_cnt ;
				}
				if(inputsignal(UP_LIMIT_SIGNAL) ==0)
				{
						mot_runbysped_mode(0,0,0);
					  lift_mot_state = LIFT_STOP ;
				}
				break;
			case LIFT_RUN_A_DOWN:
				check_rst();
				if(expireTicks(old_tick,acc_dis_time_ms))
				{
						mot[0].cur_speed += 5 ;
					  if(mot[0].cur_speed >=mot[0].run_speed )
						{
								lift_mot_state = LIFT_RUN_DOWN ;
						}
						mot_runbysped_mode(0,1,mot[0].cur_speed) ;
						old_tick = time_cnt ;						
				}
				if(inputsignal(DOWN_LIMIT_SIGNAL) ==0)
				{
						mot_runbysped_mode(0,0,0);
					  lift_mot_state = LIFT_STOP ;
				}
				break;
		  case LIFT_RUN_UP:
				check_rst();
				if(inputsignal(UP_LIMIT_SIGNAL) ==0)
				{
						mot_runbysped_mode(0,0,0);
					  lift_mot_state = LIFT_STOP ;
				}
				break;
			case LIFT_RUN_DOWN :
				check_rst();
				if(inputsignal(DOWN_LIMIT_SIGNAL) ==0)
				{
						mot_runbysped_mode(0,0,0);
					  lift_mot_state = LIFT_STOP ;
				}
				else  
				{
					 if(mot_r_limit ==0) //减速点
					 {
							lift_mot_state = LIFT_RUN_D ;
						  mot_runbysped_mode(0,1,mot[0].d_speed) ;
					 } 
				}
				break;
			case LIFT_RUN_D:
				check_rst();
				if(inputsignal(DOWN_LIMIT_SIGNAL) ==0)
				{
						mot_runbysped_mode(0,0,0);
					  lift_mot_state = LIFT_STOP ;
				}
				break;
			default:
				break;
		}
}




















