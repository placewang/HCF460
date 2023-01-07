#ifndef __UPGRADE_H
#define __UPGRADE_H
#include "bsp.h"


enum
{
  NO_ERROR_2,
  FLASH_ERROR,
  PACK_ERROR,
  NO_UPGRADE = 6,
  CRC_ERROR_2,
  CRC_FLAG_ERROR
};


#define SECTOR_SIZE    0x2000
#define ADDR_APP_BACKUP         0x50000  //APP������  320K��ʼ

#define CMD_UPGRADE_START	0xA050AA00
#define CMD_UPGRADE_RCVDATA	0xA050AA55
#define CMD_UPGRADE_BURN	0xA0505A5A
#define CMD_UPGRADE_SUCCESS	0xA05055AA



#define UPGRADE_REQUEST		    0xE1	/* �������� */ 
#define UPGRADE_ENDDATA		    0xE5	/* �������� */ 
#define UPGRADE_DATA_RECEIVED	0xE3	/* �������� */ 
#define UPGRADE_DATA_RECEIVED_2	0xE4	/* �������� */ 




#define UPGRADE_DATA_RECEIVEDERR	0x013D	/* ��������--�������ݳ��� */ 
#define UPGRADE_DATA_RECEIVEDAPPVER_LOW	0x023D	/* ��������--�������ݳ��� */ 
#define UPGRADE_DATA_RECEIVEDAPPCRC_ERROR	0x033D	/* ��������--CRCУ����� */ 

extern unsigned short canid_upgrade;
extern unsigned short canid_upgrade_rx;
extern unsigned short canid_upgrade_tx;
extern int timdelay_upgrade;

extern void UpgradeLoop(void);
#endif









