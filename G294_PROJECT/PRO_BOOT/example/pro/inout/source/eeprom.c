#include "eeprom.h"
#include "hc32_ddl.h"
#include "hc32f46x_efm.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Global variable used to store variable value in read sequence */
u32 DataVar = 0;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
/*extern*/ u16 VirtAddVarTab[NumbOfVar];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static FLASH_Status EE_Format(void);
static u16 EE_FindValidPage(u8 Operation);
static u16 EE_VerifyPageFullWriteVariable(u32 VirtAddress, u32 Data);
static u16 EE_PageTransfer(u32 VirtAddress, u32 Data);

/*******************************************************************************
 * Function Name  : EE_Init
 * Description    : Restore the pages to a known good state in case of pages'
 *                  status corruption after a power loss.
 * Input          : None.
 * Output         : None.
 * Return         : - Flash error code: on write Flash error
 *                  - FLASH_COMPLETE: on success
 *******************************************************************************/
u16 EE_Init(void)
{
	u16 PageStatus0 = 6, PageStatus1 = 6;
	u16 VarIdx = 0;
	u16 EepromStatus = 0, ReadStatus = 0;
	s16 x = -1;
	u16 FlashStatus;
	u16 i;

	for(i = 0; i < NumbOfVar; i ++) {
		VirtAddVarTab[i] = 5 + i;
	}
	
	/* Get Page0 status */
	PageStatus0 = (*(vu16*)PAGE0_BASE_ADDRESS);
	/* Get Page1 status */
	PageStatus1 = (*(vu16*)PAGE1_BASE_ADDRESS);

	/* Check for invalid header states and repair if necessary */
	switch (PageStatus0)
	{
		case ERASED:
			if (PageStatus1 == VALID_PAGE) /* Page0 erased, Page1 valid */
			{
				/* Erase Page0 */
				 EFM_SectorErase(PAGE0_BASE_ADDRESS);//  EFM_SectorErase(ADDR_BOARDTYPE);
				/* If erase operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
			else if (PageStatus1 == RECEIVE_DATA) /* Page0 erased, Page1 receive */
			{
				/* Erase Page0 */
				EFM_SectorErase(PAGE0_BASE_ADDRESS);
				/* If erase operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
				/* Mark Page1 as valid */
				//FlashStatus = FLASH_ProgramHalfWord(PAGE1_BASE_ADDRESS, VALID_PAGE);
                EFM_SingleProgram(PAGE1_BASE_ADDRESS, VALID_PAGE);
				/* If program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
			else /* First EEPROM access (Page0&1 are erased) or invalid state -> format EEPROM */
			{
				/* Erase both Page0 and Page1 and set Page0 as valid page */
				FlashStatus = EE_Format();
				/* If erase/program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
			break;

		case RECEIVE_DATA:
			if (PageStatus1 == VALID_PAGE) /* Page0 receive, Page1 valid */
			{
				/* Transfer data from Page1 to Page0 */
				for (VarIdx = 0; VarIdx < NumbOfVar; VarIdx++)
				{
					if (( *(vu16*)(PAGE0_BASE_ADDRESS + 6)) == VirtAddVarTab[VarIdx])
					{
						x = VarIdx;
					}
					if (VarIdx != x)
					{
						/* Read the last variables' updates */
						ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
						/* In case variable corresponding to the virtual address was found */
						if (ReadStatus != 0x1)
						{
							/* Transfer the variable to the Page0 */
							EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx], DataVar);
							/* If program operation was failed, a Flash error code is returned */
							if (EepromStatus != FLASH_COMPLETE)
							{
								return EepromStatus;
							}
						}
					}
				}
				/* Mark Page0 as valid */
				//FlashStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
                  EFM_SingleProgram(PAGE0_BASE_ADDRESS, VALID_PAGE);
				/* If program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
				/* Erase Page1 */
				 EFM_SectorErase(PAGE1_BASE_ADDRESS);
				/* If erase operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
			else if (PageStatus1 == ERASED) /* Page0 receive, Page1 erased */
			{
				/* Erase Page1 */
				 EFM_SectorErase(PAGE1_BASE_ADDRESS);
				/* If erase operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
				/* Mark Page0 as valid */
				//FlashStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
                  EFM_SingleProgram(PAGE0_BASE_ADDRESS, VALID_PAGE);
				/* If program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
			else /* Invalid state -> format eeprom */
			{
				/* Erase both Page0 and Page1 and set Page0 as valid page */
				FlashStatus = EE_Format();
				/* If erase/program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
			break;

		case VALID_PAGE:
			if (PageStatus1 == VALID_PAGE) /* Invalid state -> format eeprom */
			{
				/* Erase both Page0 and Page1 and set Page0 as valid page */
				FlashStatus = EE_Format();
				/* If erase/program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
            #if 0
			else if (PageStatus1 == ERASED) /* Page0 valid, Page1 erased */
			{
				/* Erase Page1 */
				FlashStatus = FLASH_ErasePage(PAGE1_BASE_ADDRESS);
				/* If erase operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
            #endif
			else /* Page0 valid, Page1 receive */
			{
				/* Transfer data from Page0 to Page1 */
				for (VarIdx = 0; VarIdx < NumbOfVar; VarIdx++)
				{
					if ((*(vu16*)(PAGE1_BASE_ADDRESS + 6)) == VirtAddVarTab[VarIdx])
					{
						x = VarIdx;
					}
					if (VarIdx != x)
					{
						/* Read the last variables' updates */
						ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
						/* In case variable corresponding to the virtual address was found */
						if (ReadStatus != 0x1)
						{
							/* Transfer the variable to the Page1 */
							EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx], DataVar);
							/* If program operation was failed, a Flash error code is returned */
							if (EepromStatus != FLASH_COMPLETE)
							{
								return EepromStatus;
							}
						}
					}
				}
				/* Mark Page1 as valid */
				//FlashStatus = FLASH_ProgramHalfWord(PAGE1_BASE_ADDRESS, VALID_PAGE);
                  EFM_SingleProgram(PAGE1_BASE_ADDRESS, VALID_PAGE);
				/* If program operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
				/* Erase Page0 */
				FlashStatus = EFM_SectorErase(PAGE0_BASE_ADDRESS);
				/* If erase operation was failed, a Flash error code is returned */
				if (FlashStatus != FLASH_COMPLETE)
				{
					return FlashStatus;
				}
			}
			break;

		default:  /* Any other state -> format eeprom */
			/* Erase both Page0 and Page1 and set Page0 as valid page */
			FlashStatus = EE_Format();
			/* If erase/program operation was failed, a Flash error code is returned */
			if (FlashStatus != FLASH_COMPLETE)
			{
				return FlashStatus;
			}
			break;
	}

	return FLASH_COMPLETE;
}

/*******************************************************************************
 * Function Name  : EE_ReadVariable
 * Description    : Returns the last stored variable data, if found, which
 *                  correspond to the passed virtual address
 * Input          : - VirtAddress: Variable virtual address
 *                  - Data: Global variable contains the read variable value
 * Output         : None
 * Return         : - Success or error status:
 *                      - 0: if variable was found
 *                      - 1: if the variable was not found
 *                      - NO_VALID_PAGE: if no valid page was found.
 *******************************************************************************/
u16 EE_ReadVariable(u32 VirtAddress, u32* Data)
{
	u16 ValidPage = PAGE0;
	u16 AddressValue = 0x5555, ReadStatus = 1;
	u32 Address = 0x08007000, PageStartAddress = 0x08007000;

	/* Get active Page for read operation */
	ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

	/* Check if there is no valid page */
	if (ValidPage == NO_VALID_PAGE)
	{
		return  NO_VALID_PAGE;
	}

	/* Get the valid Page start Address */
	PageStartAddress = (u32)(EEPROM_START_ADDRESS + (u32)(ValidPage * PAGE_SIZE));

	/* Get the valid Page end Address */
	Address = (u32)((EEPROM_START_ADDRESS - 2) + (u32)((1 + ValidPage) * PAGE_SIZE));

	/* Check each active page address starting from end */
	while (Address > (PageStartAddress + 2))
	{
		/* Get the current location content to be compared with virtual address */
		AddressValue = (*(vu16*)Address);

		/* Compare the read address with the virtual address */
		if (AddressValue == VirtAddress)
		{
			/* Get content of Address-2 which is variable value */
			*Data = (*(vu16*)(Address - 2));

			/* In case variable value is read, reset ReadStatus flag */
			ReadStatus = FLASH_COMPLETE;

			break;
		}
		else
		{
			/* Next address location */
			Address = Address - 4;
		}
	}

	/* Return ReadStatus value: (0: variable exist, 1: variable doesn't exist) */
	return ReadStatus;
}

/*******************************************************************************
 * Function Name  : EE_WriteVariable
 * Description    : Writes/upadtes variable data in EEPROM.
 * Input          : - VirtAddress: Variable virtual address
 *                  - Data: 16 bit data to be written
 * Output         : None
 * Return         : - Success or error status:
 *                      - FLASH_COMPLETE: on success,
 *                      - PAGE_FULL: if valid page is full
 *                      - NO_VALID_PAGE: if no valid page was found
 *                      - Flash error code: on write Flash error
 *******************************************************************************/
u16 EE_WriteVariable(u32 VirtAddress, u32 Data)
{
	u16 Status = 0;

	/* Write the variable virtual address and value in the EEPROM */
	Status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);

	/* In case the EEPROM active page is full */
	if (Status == PAGE_FULL)
	{
		/* Perform Page transfer */
		Status = EE_PageTransfer(VirtAddress, Data);
	}

	/* Return last operation status */
	return Status;
}

/*******************************************************************************
 * Function Name  : EE_Format
 * Description    : Erases PAGE0 and PAGE1 and writes VALID_PAGE header to PAGE0
 * Input          : None
 * Output         : None
 * Return         : Status of the last operation (Flash write or erase) done during
 *                  EEPROM formating
 *******************************************************************************/
static FLASH_Status EE_Format(void)
{
	FLASH_Status FlashStatus = FLASH_COMPLETE;

	/* Erase Page0 */
	 EFM_SectorErase(PAGE0_BASE_ADDRESS);

	/* If erase operation was failed, a Flash error code is returned */
	if (FlashStatus != FLASH_COMPLETE)
	{
		return FlashStatus;
	}

	/* Set Page0 as valid page: Write VALID_PAGE at Page0 base address */
    EFM_SingleProgram(PAGE0_BASE_ADDRESS, VALID_PAGE);

	/* If program operation was failed, a Flash error code is returned */
	if (FlashStatus != FLASH_COMPLETE)
	{
		return FlashStatus;
	}

	/* Erase Page1 */
	 EFM_SectorErase(PAGE1_BASE_ADDRESS);

	/* Return Page1 erase operation status */
	return FlashStatus;
}

/*******************************************************************************
 * Function Name  : EE_FindValidPage
 * Description    : Find valid Page for write or read operation
 * Input          : - Operation: operation to achieve on the valid page:
 *                      - READ_FROM_VALID_PAGE: read operation from valid page
 *                      - WRITE_IN_VALID_PAGE: write operation from valid page
 * Output         : None
 * Return         : Valid page number (PAGE0 or PAGE1) or NO_VALID_PAGE in case
 *                  of no valid page was found
 *******************************************************************************/
static u16 EE_FindValidPage(u8 Operation)
{
	u16 PageStatus0 = 6, PageStatus1 = 6;

	/* Get Page0 actual status */
	PageStatus0 = (*(vu16*)PAGE0_BASE_ADDRESS);

	/* Get Page1 actual status */
	PageStatus1 = (*(vu16*)PAGE1_BASE_ADDRESS);

	/* Write or read operation */
	switch (Operation)
	{
		case WRITE_IN_VALID_PAGE:   /* ---- Write operation ---- */
			if (PageStatus1 == VALID_PAGE)
			{
				/* Page0 receiving data */
				if (PageStatus0 == RECEIVE_DATA)
				{
					return PAGE0;         /* Page0 valid */
				}
				else
				{
					return PAGE1;         /* Page1 valid */
				}
			}
			else if (PageStatus0 == VALID_PAGE)
			{
				/* Page1 receiving data */
				if (PageStatus1 == RECEIVE_DATA)
				{
					return PAGE1;         /* Page1 valid */
				}
				else
				{
					return PAGE0;         /* Page0 valid */
				}
			}
			else
			{
				return NO_VALID_PAGE;   /* No valid Page */
			}

		case READ_FROM_VALID_PAGE:  /* ---- Read operation ---- */
			if (PageStatus0 == VALID_PAGE)
			{
				return PAGE0;           /* Page0 valid */
			}
			else if (PageStatus1 == VALID_PAGE)
			{
				return PAGE1;           /* Page1 valid */
			}
			else
			{
				return NO_VALID_PAGE ;  /* No valid Page */
			}

		default:
			return PAGE0;             /* Page0 valid */
	}
}

/*******************************************************************************
 * Function Name  : EE_VerifyPageFullWriteVariable
 * Description    : Verify if active page is full and Writes variable in EEPROM.
 * Input          : - VirtAddress: 16 bit virtual address of the variable
 *                  - Data: 16 bit data to be written as variable value
 * Output         : None
 * Return         : - Success or error status:
 *                      - FLASH_COMPLETE: on success
 *                      - PAGE_FULL: if valid page is full
 *                      - NO_VALID_PAGE: if no valid page was found
 *                      - Flash error code: on write Flash error
 *******************************************************************************/
static u16 EE_VerifyPageFullWriteVariable(u32 VirtAddress, u32 Data)
{
	FLASH_Status FlashStatus = FLASH_COMPLETE;
	u16 ValidPage = PAGE0;
	u32 Address = 0x08007400, PageEndAddress = 0x080107FF;

	/* Get valid Page for write operation */
	ValidPage = EE_FindValidPage(WRITE_IN_VALID_PAGE);

	/* Check if there is no valid page */
	if (ValidPage == NO_VALID_PAGE)
	{
		return  NO_VALID_PAGE;
	}

	/* Get the valid Page start Address */
	Address = (u32)(EEPROM_START_ADDRESS + (u32)(ValidPage * PAGE_SIZE));

	/* Get the valid Page end Address */
	PageEndAddress = (u32)((EEPROM_START_ADDRESS - 2) + (u32)((1 + ValidPage) * PAGE_SIZE));

	/* Check each active page address starting from begining */
	while (Address < PageEndAddress)
	{
		/* Verify if Address and Address+2 contents are 0xFFFFFFFF */
		if ((*(vu32*)Address) == 0xFFFFFFFF)
		{
			/* Set variable data */
			EFM_SingleProgram(Address, Data);
			/* If program operation was failed, a Flash error code is returned */
			if (FlashStatus != FLASH_COMPLETE)
			{
				return FlashStatus;
			}
			/* Set variable virtual address */
			 EFM_SingleProgram(Address + 4, VirtAddress);
			/* Return program operation status */
			return FlashStatus;
		}
		else
		{
			/* Next address location */
			Address = Address + 4;
		}
	}

	/* Return PAGE_FULL in case the valid page is full */
	return PAGE_FULL;
}

/*******************************************************************************
 * Function Name  : EE_PageTransfer
 * Description    : Transfers last updated variables data from the full Page to
 *                  an empty one.
 * Input          : - VirtAddress: 16 bit virtual address of the variable
 *                  - Data: 16 bit data to be written as variable value
 * Output         : None
 * Return         : - Success or error status:
 *                      - FLASH_COMPLETE: on success,
 *                      - PAGE_FULL: if valid page is full
 *                      - NO_VALID_PAGE: if no valid page was found
 *                      - Flash error code: on write Flash error
 *******************************************************************************/
static u16 EE_PageTransfer(u32 VirtAddress, u32 Data)
{
	FLASH_Status FlashStatus = FLASH_COMPLETE;
	u32 NewPageAddress = 0x080073FF, OldPageAddress = 0x08007000;
	u16 ValidPage = PAGE0, VarIdx = 0;
	u16 EepromStatus = 0, ReadStatus = 0;

	/* Get active Page for read operation */
	ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

	if (ValidPage == PAGE1)       /* Page1 valid */
	{
		/* New page address where variable will be moved to */
		NewPageAddress = PAGE0_BASE_ADDRESS;

		/* Old page address where variable will be taken from */
		OldPageAddress = PAGE1_BASE_ADDRESS;
	}
	else if (ValidPage == PAGE0)  /* Page0 valid */
	{
		/* New page address where variable will be moved to */
		NewPageAddress = PAGE1_BASE_ADDRESS;

		/* Old page address where variable will be taken from */
		OldPageAddress = PAGE0_BASE_ADDRESS;
	}
	else
	{
		return NO_VALID_PAGE;       /* No valid Page */
	}

	/* Set the new Page status to RECEIVE_DATA status */
	EFM_SingleProgram(NewPageAddress, RECEIVE_DATA);
	/* If program operation was failed, a Flash error code is returned */
	if (FlashStatus != FLASH_COMPLETE)
	{
		return FlashStatus;
	}

	/* Write the variable passed as parameter in the new active page */
	EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
	/* If program operation was failed, a Flash error code is returned */
	if (EepromStatus != FLASH_COMPLETE)
	{
		return EepromStatus;
	}

	/* Transfer process: transfer variables from old to the new active page */
	for (VarIdx = 0; VarIdx < NumbOfVar; VarIdx++)
	{
		if (VirtAddVarTab[VarIdx] != VirtAddress)  /* Check each variable except the one passed as parameter */
		{
			/* Read the other last variable updates */
			ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
			/* In case variable corresponding to the virtual address was found */
			if (ReadStatus != 0x1)
			{
				/* Transfer the variable to the new active page */
				EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx], DataVar);
				/* If program operation was failed, a Flash error code is returned */
				if (EepromStatus != FLASH_COMPLETE)
				{
					return EepromStatus;
				}
			}
		}
	}

	/* Erase the old Page: Set old Page status to ERASED status */
	EFM_SectorErase(OldPageAddress);
	/* If erase operation was failed, a Flash error code is returned */
	if (FlashStatus != FLASH_COMPLETE)
	{
		return FlashStatus;
	}

	/* Set new Page status to VALID_PAGE status */
	EFM_SingleProgram(NewPageAddress, VALID_PAGE);
	/* If program operation was failed, a Flash error code is returned */
	if (FlashStatus != FLASH_COMPLETE)
	{
		return FlashStatus;
	}

	/* Return last operation flash status */
	return FlashStatus;
}

int EE_Read(int addr, u32 *Data, int len)
{
	int i;

	if(addr + len > NumbOfVar)
		return NO_VALID_ADDR;
	  
	for(i = 0; i < len; i ++) {
		if(EE_ReadVariable(VirtAddVarTab[addr + i], &Data[i]) != FLASH_COMPLETE) 		return -1;
	}

	return 0;
}

int EE_Write(int addr, u32 *Data, int len)
{	
	int i;
	EFM_Unlock();
	__disable_irq();
	if(addr + len > NumbOfVar)
	{
		__enable_irq();
			EFM_Lock();
		return NO_VALID_ADDR;
	}
	for(i = 0; i < len; i ++) {
		if(EE_WriteVariable(VirtAddVarTab[addr + i], Data[i]) != FLASH_COMPLETE) 
		{
			__enable_irq();
			EFM_Lock();
			return -1;
		}
	}	
	__enable_irq();
	EFM_Lock();
	return 0;	
}