#ifndef _EEPROM_H
#define _EEPROM_H

typedef unsigned short u16;
typedef unsigned char u8;
typedef unsigned int  u32;
typedef signed short s16;
typedef u16 vu16;
typedef u32 vu32;

typedef enum
{
  FLASH_BUSY = 1,
  FLASH_ERROR_WRP,
  FLASH_ERROR_PROGRAM,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;

/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : eeprom.h
* Author             : MCD Application Team
* Version            : V2.0.0
* Date               : 06/16/2008
* Description        : This file contains all the functions prototypes for the
*                      EEPROM emulation firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/

/* Includes ------------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* EEPROM start address in Flash */  //from  29K
#define EEPROM_START_ADDRESS    ((u32)0x78000) /* EEPROM emulation start address: from 10K satrt
                                                  after 64KByte of used Flash memory */
#define PAGE_SIZE  (u16)0x2000  /* Page size = 1KByte */

/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS      ((u32)(EEPROM_START_ADDRESS + 0x000))
#define PAGE0_END_ADDRESS       ((u32)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))

#define PAGE1_BASE_ADDRESS      ((u32)(EEPROM_START_ADDRESS + PAGE_SIZE))
#define PAGE1_END_ADDRESS       ((u32)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))

/* Used Flash pages for EEPROM emulation */
#define PAGE0                   ((u16)0x0000)
#define PAGE1                   ((u16)0x0001)

/* No valid page define */
#define NO_VALID_PAGE           ((u16)0x00AB)
#define NO_VALID_ADDR           ((u16)0x00AF)

/* Page status definitions */
#define ERASED                  ((u16)0xFFFF)     /* PAGE is empty */
#define RECEIVE_DATA            ((u16)0xEEEE)     /* PAGE is marked to receive data */
#define VALID_PAGE              ((u16)0x0000)     /* PAGE containing valid data */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE    ((u8)0x00)
#define WRITE_IN_VALID_PAGE     ((u8)0x01)

/* Page full define */
#define PAGE_FULL               ((u8)0x80)

/* Variables' number */
#define NumbOfVar               ((u8)0x64)

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
u16 EE_Init(void);
u16 EE_ReadVariable(u32 VirtAddress, u32* Data);
u16 EE_WriteVariable(u32 VirtAddress, u32 Data);
int EE_Read(int addr, u32 *Data, int len);
int EE_Write(int addr, u32 *Data, int len);

#endif /* __EEPROM_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




