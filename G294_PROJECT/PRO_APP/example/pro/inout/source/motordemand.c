/***********************************************************************
G294�ڻƸԴ����������������
�㽭��ǿ�Ƽ�
���ƽ̨��:F11500
2022/11/22
************************************************************************/

#include "motordemand.h"
#include "message.h"
#include "logic_lay.h"
#include "mot.h"
#include "tmc5610.h"

MotorParameter MrPr={1,1,4,19,\
										400,400,100,5000 
										};
								
/*
�˶�����
*/
void MotRunCurrentSet(uint8_t idx,unsigned char* ru)
{
	int cur_reg=0;   
	unsigned short curvalue=0;
	curvalue =ru[3];
	curvalue|=ru[2]<<8;
  
	MrPr.CurrentRun=(32*curvalue)/6600;//�����Ч����Ӳ������Ϊ4.7A   ���ҷ�ֵ��������Ϊ6.6A
  cur_reg=((0x60000|(MrPr.CurrentRun<<8))|MrPr.Currentlock);
	tmc5160_writeInt(idx,TMC5160_IHOLD_IRUN,cur_reg);//0x60803     
}

/*
��������
	�����ܵ�������1-15����λ
*/
void MotClockCurrentSet(uint8_t idx,unsigned char* ck)
{
	unsigned short curvalue=0;
	int cur_reg=0; 
  curvalue=ck[3];
	curvalue|=ck[2]<<8;
	if(curvalue>=1&&curvalue<=15)
	{	
		MrPr.Currentlock=	MrPr.CurrentRun/curvalue;
		cur_reg=((0x60000|(MrPr.CurrentRun<<8))|MrPr.Currentlock);	
		tmc5160_writeInt(idx,TMC5160_IHOLD_IRUN,cur_reg);
	}
}

/*
�������г�
*/			
signed SetMaxLeadScrewStroke(unsigned char* Vl)
{
	MrPr.LeadScrewStroke_Max = Vl[3];
  MrPr.LeadScrewStroke_Max|= Vl[4]<<8;
	return 0;
}	

/*
���ʹ��
*/
void SetMotorEn(unsigned char *en)
{
	if(en[2]==1)
	{
		mot_enable(0,0);
	}
	else if(en[2]==0)
	{
		mot_enable(0,1);
	}
}

/*
����ƫ����
offset:Э������
*/
signed char MoveOffsetUp(unsigned char* offset)
{
	unsigned short moveoffset=0;
	MotMove.MotorTimeOutFlagUP=0;
	MotMove.MotorTimeOutFlagDwon=0;
	
	moveoffset =offset[4];
	moveoffset|=offset[3]<<8;
	mot_run_relative_pos_mode(0,MrPr.UpsSpeed,moveoffset);
	return 0;
}
/*
����ƫ����
offset:Э������
*/
signed char MoveOffsetDwon(unsigned char* offset)
{
	signed short moveoffset=0;
	
	MotMove.MotorTimeOutFlagUP=0;
	MotMove.MotorTimeOutFlagDwon=0;
	
	moveoffset =offset[4];
	moveoffset|=offset[3]<<8;
	moveoffset=0-moveoffset;
	mot_run_relative_pos_mode(0,MrPr.DwonSpeed,moveoffset);
	return 0;
}


/*
Can��Ϣ���ܽ���
dat:�����׵�ַ
*/
signed char CanRaedMassgaAnalysis(unsigned char * dat)
{
	SensorGetMotorStatePoll();
	
	MoveUpTask();
	//ֹͣ
	if(dat[0]==0xD4&&dat[1]==0x01&&dat[2]==0x00)
	{			
		mot_run_relative_pos_mode(0,MrPr.UpsSpeed,0);
		MotMove.StartUpFlag=0;
		MotMove.StartDwonFlag=0;
//		ClearMotorState(0);
//		ClearMotorState(1);
	}
	//��ƫ��	
	else if(dat[0]==0xD4&&dat[1]==0x02&&dat[2]==0x01)
	{
		MoveOffsetUp(dat);
	}
	//��ƫ��
	else if(dat[0]==0xD4&&dat[1]==0x02&&dat[2]==0x02)
	{
		MoveOffsetDwon(dat);
	}
	//�źŷ���
	else if(dat[0]==0x71&&dat[1]==0x0c)
	{
		 requestflag=1;
		 StartActiveReporting=1;	
		 signalreturnflag=2;	
	}
	//��������
	else if(dat[0]==0xD4&&dat[1]==0x10&&(dat[2]==0||dat[2]==1))
	{
			MrPr.Mtdir=dat[2];
			mot_dir_config(0,MrPr.Mtdir);
	}		
	//ʹ��
	else if(dat[0]==0xD4&&dat[1]==0x11)
	{
		SetMotorEn(dat);
	}	
	//�������ٶ�����
	else if(dat[0]==0xD4&&dat[1]==0x12)
	{
		MrPr.UpsSpeed=dat[3];
		MrPr.UpsSpeed |=dat[2]<<8;
//		tmc5160_writeInt(0,TMC5160_VMAX,MrPr.UpsSpeed*MT_SPD_ADJUST);
	}	
	//�������ٶ�����
	else if(dat[0]==0xD4&&dat[1]==0x13)
	{
		MrPr.DwonSpeed =dat[3];
		MrPr.DwonSpeed |=dat[2]<<8;
//		tmc5160_writeInt(0,TMC5160_VMAX,MrPr.DwonSpeed*MT_SPD_ADJUST);
	}
	//���ٶ�
	else if(dat[0]==0xD4&&dat[1]==0x14)
	{
		MrPr.StopSpeed =dat[3];
		MrPr.StopSpeed |=dat[2]<<8;
		mot_D1_set(0,MrPr.StopSpeed);
	}
	//���е���	
	else if(dat[0]==0xD4&&dat[1]==0x15)
	{
		MotRunCurrentSet(0,dat);
	}
	//��������
	else if(dat[0]==0xD4&&dat[1]==0x16)
	{
		MotClockCurrentSet(0,dat);
	}
	//������λ
	else if(dat[0]==0xD4&&dat[1]==0x01&&dat[2]==0x01)
	{
		ClearMotorState(0);
		MotMove.OriginPosUp=mot_curpos_get(0);
		MotMove.StartUpFlag=1;	
	}
	//������λ	
	else if(dat[0]==0xD4&&dat[1]==0x01&&dat[2]==0x02)
	{
		ClearMotorState(1);
		MotMove.OriginPosDwon=mot_curpos_get(0);
		MotMove.StartDwonFlag=1;
	}	
	//�˶�ģʽ0
	else if(dat[0]==0xD4&&dat[1]==0x01&&dat[2]==0x04)
	{
		MotMove.MotorRunMode=0;
	}		
	//�˶�ģʽ1
	else if(dat[0]==0xD4&&dat[1]==0x01&&dat[2]==0x03)
	{
		MotMove.MotorRunMode=1;
	}		
	//�����г�
	else if(dat[0]==0xD4&&dat[1]==0x17)
	{
		SetMaxLeadScrewStroke(dat);
	}		
	memset(dat,0,8);
	return 0;
}






