#ifndef __UPGRADE_H
#define __UPGRADE_H
#include "machconfig.h"

#define SECTOR_SIZE    0x2000

#define SONBOARD_SECTOR_SIZE 0x200


#define ADDR_APP_BACKUP         0x50000  //APP������  320K��ʼ

#define CMD_UPGRADE_START	0xA050AA00
#define CMD_UPGRADE_RCVDATA	0xA050AA55
#define CMD_UPGRADE_BURN	0xA0505A5A
#define CMD_UPGRADE_SUCCESS	0xA05055AA


#define UPGRADE_BOARDTYPE   0x30    //���յ���Ҫ�������Ӱ�����   new add
#define UPGRADE_REQUEST		0x3A	/* �������� */ 
#define UPGRADE_DATA		0x3B	/* �������� */ 
#define UPGRADE_ENDDATA		0x3C	/* �������� */ 
#define UPGRADE_DATA_RECEIVED	0x3D	/* �������� */ 
#define UPGRADE_SUCCESS		0x3E	/* �������� */ 
#define UPGRADE_FORCE_REQ	0x3F	/* �������� */ 

enum
{
  NO_ERROR_2,
  FLASH_ERROR,
  PACK_ERROR,
  NO_UPGRADE = 6,
  CRC_ERROR_2,
  CRC_FLAG_ERROR
};

enum
{   
 UPGRADE_PHASE_PREPARE=1,
 UPGRADE_PHASE_GOBOOT=2,
 UPGRADE_PHASE_WAITAPPBOOT=3,
 UPGRADE_PHASE_ERASE=4,
 UPGRADE_PHASE_STARTUPG=5,
 UPGRADE_PHASE_ENDUPG=6,
 UPGRADE_PHASE_CHECKSUCCESS=7,
};








#define UPGRADE_DATA_RECEIVEDERR	0x013D	/* ��������--�������ݳ��� */ 
#define UPGRADE_DATA_RECEIVEDAPPVER_LOW	0x023D	/* ��������--�������ݳ��� */ 
#define UPGRADE_DATA_RECEIVEDAPPCRC_ERROR	0x033D	/* ��������--CRCУ����� */ 

extern unsigned short canid_upgrade;
extern unsigned short canid_upgrade_rx;
extern unsigned short canid_upgrade_tx;
extern int timdelay_upgrade;
extern void UpgradeSonBoardLoop(void);
extern void UpgradeInitDeal(uint16_t boardtype);
extern void NewSonBoardUpGradeLoop(void);
#endif









