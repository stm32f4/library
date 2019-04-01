/* Prevent recursive inclusion */
#ifndef __NORFLASH_H
#define __NORFLASH_H

//******************************************************
//             Includes
//******************************************************
#include "stm32f4xx.h"
//******************************************************

//******************************************************
//             Defines
//******************************************************
#define Bank1_NORSRAM3_ADDR  	((uint32_t)0x68000000)

//  Timeout values defined for 168Mhz operation without optimization
#define BlockErase_Timeout    	((uint32_t)0x00A00000)
#define ChipErase_Timeout     	((uint32_t)0x30000000)
#define Program_Timeout       	((uint32_t)0x00001400)

#define NORFLASH_SECTOR_SIZE 0x10000	//  64K words 128K bytes
#define NORFLASH_SECTOR_NUMBER 0x80 	//  128 sectors
#define NORFLASH_WRITE(Address, Data)  (*(__IO uint16_t *)(Bank1_NORSRAM3_ADDR + (Address)*2) = (Data))
#define NORFLASH_READ(Address)  (*(__IO uint16_t *)(Bank1_NORSRAM3_ADDR + (Address)*2))
//******************************************************

//******************************************************
//             Structures
//******************************************************

/* Chip query def */
typedef struct {
	char QRY[3];
	uint16_t Primary_OEM_Command_Set;
	uint16_t Primary_OEM_Table_Address;
} NORFLASH_CFI_Query;

/* Chip Interface */
typedef struct {
	uint16_t VCC_MIN;
	uint16_t VCC_MAX;
	uint16_t VPP_MIN;
	uint16_t VPP_MAX;
	uint16_t TimeOut_Single;
	uint16_t TimeOut_Buffer;
	uint16_t TimeOut_Block;
	uint16_t TimeOut_Chip;
	uint16_t Max_TimeOut_Single;
	uint16_t Max_TimeOut_Buffer;
	uint16_t Max_TimeOut_Block;
	uint16_t Max_TimeOut_Chip;
} NORFLASH_CFI_System_Interface_String;

/* Memory Info */
typedef struct {
	uint16_t Manufacturer_Code;
	uint16_t Device_Code1;
	uint16_t Device_Code2;
	uint16_t Device_Code3;
} NORFLASH_Id;

/* NOR Status */
typedef enum {
	NOR_SUCCESS = 0, NOR_ONGOING, NOR_ERROR, NOR_TIMEOUT
} NORFLASH_Status;
//******************************************************

//******************************************************
//             Exported functions
//******************************************************
void NORFLASH_Init();
void NORFLASH_ReadID(NORFLASH_Id* NOR_ID);
NORFLASH_Status NORFLASH_EraseBlock(uint32_t BlockAddr);
NORFLASH_Status NORFLASH_EraseBlockNum(uint32_t BlockNum);
NORFLASH_Status NORFLASH_EraseChip(void);
NORFLASH_Status NORFLASH_WriteWord(uint32_t WriteAddr, uint16_t Data);
NORFLASH_Status NORFLASH_WriteBuffer(uint16_t* pBuffer, uint32_t WriteAddr,
		uint32_t NumWordToWrite);
NORFLASH_Status NORFLASH_ProgramBuffer(uint16_t* pBuffer, uint32_t WriteAddr,
		uint32_t NumWordToWrite);
uint16_t NORFLASH_ReadWord(uint32_t ReadAddr);
void NORFLASH_ReadBuffer(uint16_t* pBuffer, uint32_t ReadAddr,
		uint32_t NumWordToRead);
NORFLASH_Status NORFLASH_ReturnToReadMode(void);
NORFLASH_Status NORFLASH_Reset(void);
NORFLASH_Status NORFLASH_GetStatus(uint32_t Timeout);
//******************************************************

#endif
