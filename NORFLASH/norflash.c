/**
 ***************************************************************************
 * Sample library to use nor flash memories.
 *
 * Tested on S29GL128N memory.
 *
 * Code is for NE3 bank
 *
 *
 ***************************************************************************
 */

/* Includes */
#include "norflash.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_fsmc.h"

/**
 * Initialize NORFLASH access though FSMC bus
 *
 */
void NORFLASH_Init() {

	GPIO_InitTypeDef GPIO_InitStruct;
	FSMC_NORSRAMInitTypeDef FSMC_NORSRAM_InitStructure;
	FSMC_NORSRAMTimingInitTypeDef readTimingStructure;
	FSMC_NORSRAMTimingInitTypeDef writeTimingStructure;

	/* FSMC_NOR GPIO Configuration

	 PD0	 ------> FSMC_NOR_RAM_D2
	 PD1	 ------> FSMC_NOR_RAM_D3
	 PD4	 ------> FSMC_NOR_RAM_NOE
	 PD5	 ------> FSMC_NOR_RAM_NWE
	 PD6	 ------> READY/BUSY (Pull Up Floating INPUT)
	 PD8	 ------> FSMC_NOR_RAM_D13
	 PD9	 ------> FSMC_NOR_RAM_D14
	 PD10	 ------> FSMC_NOR_RAM_D15
	 PD11	 ------> FSMC_NOR_RAM_A16
	 PD12	 ------> FSMC_NOR_RAM_A17
	 PD13	 ------> FSMC_NOR_RAM_A18
	 PD14	 ------> FSMC_NOR_RAM_D0
	 PD15	 ------> FSMC_NOR_RAM_D1

	 PE3	 ------> FSMC_NOR_RAM_A19
	 PE4	 ------> FSMC_NOR_RAM_A20
	 PE5	 ------> FSMC_NOR_RAM_A21
	 PE6	 ------> FSMC_NOR_RAM_A22
	 PE7	 ------> FSMC_NOR_RAM_D4
	 PE8	 ------> FSMC_NOR_RAM_D5
	 PE9	 ------> FSMC_NOR_RAM_D6
	 PE10	 ------> FSMC_NOR_RAM_D7
	 PE11	 ------> FSMC_NOR_RAM_D8
	 PE12	 ------> FSMC_NOR_RAM_D9
	 PE13	 ------> FSMC_NOR_RAM_D10
	 PE14	 ------> FSMC_NOR_RAM_D11
	 PE15	 ------> FSMC_NOR_RAM_D12

	 PF0	 ------> FSMC_NOR_RAM_A0
	 PF1	 ------> FSMC_NOR_RAM_A1
	 PF2	 ------> FSMC_NOR_RAM_A2
	 PF3	 ------> FSMC_NOR_RAM_A3
	 PF4	 ------> FSMC_NOR_RAM_A4
	 PF5	 ------> FSMC_NOR_RAM_A5
	 PF12	 ------> FSMC_NOR_RAM_A6
	 PF13	 ------> FSMC_NOR_RAM_A7
	 PF14	 ------> FSMC_NOR_RAM_A8
	 PF15	 ------> FSMC_NOR_RAM_A9

	 PG0	 ------> FSMC_NOR_RAM_A10
	 PG1	 ------> FSMC_NOR_RAM_A11
	 PG2	 ------> FSMC_NOR_RAM_A12
	 PG3	 ------> FSMC_NOR_RAM_A13
	 PG4	 ------> FSMC_NOR_RAM_A14
	 PG5	 ------> FSMC_NOR_RAM_A15
	 PG10	 ------> FSMC_NOR_RAM_NE3
	 */

	/*Enable AHB1 peripheral clock */
	RCC_AHB1PeriphClockCmd(
			RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF
					| RCC_AHB1Periph_GPIOG, ENABLE);

	/* Enable FSMC Clock */
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

	/*Configure GPIO pin alternate function */

	/*Configure GPIO B pin alternate function */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_FSMC);

	/*Configure GPIO D pin alternate function */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

	/*Configure GPIO E pin alternate function */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource3, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);

	/*Configure GPIO F pin alternate function */
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource2, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource3, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource13, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource15, GPIO_AF_FSMC);

	/*Configure GPIO G pin alternate function */
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource2, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource3, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource10, GPIO_AF_FSMC);

	/* Common properties for FSMC bus */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	/*Configure GPIO D pin */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5
			| GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12
			| GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO E pin */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6
			| GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11
			| GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO F pin */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
			| GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14
			| GPIO_Pin_15;
	GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO G pin */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
			| GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10;
	GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO G pin 6 Ready/busy */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*-- FSMC Configuration ------------------------------------------------------*/
	readTimingStructure.FSMC_AddressSetupTime = 1;
	readTimingStructure.FSMC_AddressHoldTime = 0;
	readTimingStructure.FSMC_DataSetupTime = 15; //15*6=90ns
	readTimingStructure.FSMC_BusTurnAroundDuration = 0;
	readTimingStructure.FSMC_CLKDivision = 0;
	readTimingStructure.FSMC_DataLatency = 0;
	readTimingStructure.FSMC_AccessMode = FSMC_AccessMode_B;

	writeTimingStructure.FSMC_AddressSetupTime = 4; //4*6=24ns
	writeTimingStructure.FSMC_AddressHoldTime = 0;
	writeTimingStructure.FSMC_DataSetupTime = 4; //4*6=24ns
	writeTimingStructure.FSMC_BusTurnAroundDuration = 0;
	writeTimingStructure.FSMC_CLKDivision = 0;
	writeTimingStructure.FSMC_DataLatency = 0;
	writeTimingStructure.FSMC_AccessMode = FSMC_AccessMode_B;

	/* FSMC parameters */
	FSMC_NORSRAM_InitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM3;
	FSMC_NORSRAM_InitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAM_InitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
	FSMC_NORSRAM_InitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAM_InitStructure.FSMC_BurstAccessMode =
			FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAM_InitStructure.FSMC_AsynchronousWait =
			FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAM_InitStructure.FSMC_WaitSignalPolarity =
			FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAM_InitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAM_InitStructure.FSMC_WaitSignalActive =
			FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAM_InitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAM_InitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAM_InitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAM_InitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAM_InitStructure.FSMC_ReadWriteTimingStruct = &readTimingStructure;
	FSMC_NORSRAM_InitStructure.FSMC_WriteTimingStruct = &writeTimingStructure;

	/* Apply FSMS settings */
	FSMC_NORSRAMInit(&FSMC_NORSRAM_InitStructure);

	/* Enable FSMC Bank1_NORSRAM3 Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM3, ENABLE);
}

/**
 * Read ID norflash
 *
 * @param NOR_ID structure to store information on NOR flash
 *
 */
void NORFLASH_ReadID(NORFLASH_Id* NOR_ID) {
	NORFLASH_WRITE(0x0555, 0x00AA);
	NORFLASH_WRITE(0x02AA, 0x0055);
	NORFLASH_WRITE(0x0555, 0x0090);

	NOR_ID->Manufacturer_Code =
			*(__IO uint16_t *) (Bank1_NORSRAM3_ADDR + 0x0000);
	NOR_ID->Device_Code1 =
			*(__IO uint16_t *) (Bank1_NORSRAM3_ADDR + 0x0001 * 2);
	NOR_ID->Device_Code2 =
			*(__IO uint16_t *) (Bank1_NORSRAM3_ADDR + 0x000E * 2);
	NOR_ID->Device_Code3 =
			*(__IO uint16_t *) (Bank1_NORSRAM3_ADDR + 0x000F * 2);
}

/**
 * @brief  Erases the specified Nor memory block.
 * @param  BlockAddr: address of the block to erase.
 * @retval NOR_Status: The returned value can be: NOR_SUCCESS, NOR_ERROR
 *         or NOR_TIMEOUT
 */
NORFLASH_Status NORFLASH_EraseBlock(uint32_t blockAddr) {
	NORFLASH_WRITE(0x0555, 0x00AA);
	NORFLASH_WRITE(0x02AA, 0x0055);
	NORFLASH_WRITE(0x0555, 0x0080);
	NORFLASH_WRITE(0x0555, 0x00AA);
	NORFLASH_WRITE(0x02AA, 0x0055);
	NORFLASH_WRITE( blockAddr, 0x30);

	return (NORFLASH_GetStatus(BlockErase_Timeout));
}

/**
 * @brief  Erases the specified Nor memory block.
 * @param  BlockNum: number of the block to erase. (starts at 0)
 * @retval NOR_Status: The returned value can be: NOR_SUCCESS, NOR_ERROR
 *         or NOR_TIMEOUT
 */
NORFLASH_Status NORFLASH_EraseBlockNum(uint32_t blockNum) {
	NORFLASH_WRITE(0x0555, 0x00AA);
	NORFLASH_WRITE(0x02AA, 0x0055);
	NORFLASH_WRITE(0x0555, 0x0080);
	NORFLASH_WRITE(0x0555, 0x00AA);
	NORFLASH_WRITE(0x02AA, 0x0055);
	NORFLASH_WRITE( (blockNum*NORFLASH_SECTOR_SIZE), 0x30);

	return (NORFLASH_GetStatus(BlockErase_Timeout));
}

/**
 * @brief  Erases the entire chip.
 * @param  None
 * @retval NOR_Status: The returned value can be: NOR_SUCCESS, NOR_ERROR
 *         or NOR_TIMEOUT
 */
NORFLASH_Status NORFLASH_EraseChip(void) {
	NORFLASH_WRITE(0x0555, 0x00AA);
	NORFLASH_WRITE(0x02AA, 0x0055);
	NORFLASH_WRITE(0x0555, 0x0080);
	NORFLASH_WRITE(0x0555, 0x00AA);
	NORFLASH_WRITE(0x02AA, 0x0055);
	NORFLASH_WRITE(0x0555, 0x0010);

	return (NORFLASH_GetStatus(ChipErase_Timeout));
}

/**
 * @brief  Writes a word (16 bits) to the NOR memory.
 * @param  WriteAddr: NOR memory internal address to write to.
 * @param  Data: Data to write.
 * @retval NOR_Status: The returned value can be: NOR_SUCCESS, NOR_ERROR
 *         or NOR_TIMEOUT
 */
NORFLASH_Status NORFLASH_WriteWord(uint32_t writeAddr, uint16_t data) {
	NORFLASH_WRITE(0x0555, 0x00AA);
	NORFLASH_WRITE(0x02AA, 0x0055);
	NORFLASH_WRITE(0x0555, 0x00A0);
	NORFLASH_WRITE(writeAddr, data);

	return (NORFLASH_GetStatus(Program_Timeout));
}

/**
 * @brief  Writes a word (16bits) buffer to the FSMC NOR memory.
 * @param  pBuffer: pointer to buffer.
 * @param  WriteAddr: NOR memory internal address from which the data will be
 *         written.
 * @param  NumWordToWrite: number of words to write.
 * @retval NOR_Status: The returned value can be: NOR_SUCCESS, NOR_ERROR
 *         or NOR_TIMEOUT
 */
NORFLASH_Status NORFLASH_WriteBuffer(uint16_t* pBuffer, uint32_t writeAddr,
		uint32_t numWordToWrite) {
	NORFLASH_Status status = NOR_ONGOING;

	do {
		/*Transfer data to the memory */
		status = NORFLASH_WriteWord(writeAddr, *pBuffer++);
		writeAddr = writeAddr + 2;
		numWordToWrite--;
	} while ((status == NOR_SUCCESS) && (numWordToWrite != 0));

	return (status);
}

/**
 * @brief  Writes a word buffer to the FSMC NOR memory. This function
 *         must be used only with S29GL128P NOR memory.
 * @param  pBuffer: pointer to buffer.
 * @param  WriteAddr: NOR memory internal address from which the data will be
 *         written.
 * @param  NumWordToWrite: number of words to write.
 *         The maximum allowed value is 32 words (64 bytes).
 * @retval NOR_Status: The returned value can be: NOR_SUCCESS, NOR_ERROR
 *         or NOR_TIMEOUT
 */
NORFLASH_Status NORFLASH_ProgramBuffer(uint16_t* pBuffer, uint32_t writeAddr,
		uint32_t NumWordToWrite) {
	uint32_t currentaddress = 0x00;
	uint32_t endaddress = 0x00;

	/* Initialize variables */
	currentaddress = writeAddr;
	endaddress = writeAddr + NumWordToWrite - 1;

	/* Issue unlock command sequence */
	NORFLASH_WRITE(0x00555, 0x00AA);
	NORFLASH_WRITE(0x02AA, 0x0055);

	/* Write Write Buffer Load Command at sector address */
	NORFLASH_WRITE(writeAddr, 0x25);
	NORFLASH_WRITE(writeAddr, NumWordToWrite - 1);

	/* Load Data into NOR Buffer */
	while (currentaddress <= endaddress) {
		NORFLASH_WRITE(currentaddress, *pBuffer++);
		currentaddress++;
	}

	/* Issue Write command */
	NORFLASH_WRITE(writeAddr, 0x29);

	return (NORFLASH_GetStatus(Program_Timeout));
}

/**
 * @brief  Reads a word (16 bits) from the NOR memory.
 * @param  ReadAddr: NOR memory internal address to read from.
 * @retval word read from the NOR memory
 */
uint16_t NORFLASH_ReadWord(uint32_t readAddr) {
	NORFLASH_WRITE(0x00555, 0x00AA);
	NORFLASH_WRITE(0x002AA, 0x0055);
	NORFLASH_WRITE(readAddr, 0x00F0);

	return (*(__IO uint16_t *) (Bank1_NORSRAM3_ADDR + readAddr * 2));
}

/**
 * @brief  Reads a block of data from the FSMC NOR memory.
 * @param  pBuffer: pointer to the buffer that receives the data read from the
 *         NOR memory.
 * @param  ReadAddr: NOR memory internal address to read from.
 * @param  NumWordToRead : number of Half word to read.
 * @retval None
 */
void NORFLASH_ReadBuffer(uint16_t* pBuffer, uint32_t ReadAddr,
		uint32_t NumWordToRead) {
	NORFLASH_WRITE(0x0555, 0x00AA);
	NORFLASH_WRITE(0x02AA, 0x0055);
	NORFLASH_WRITE(Bank1_NORSRAM3_ADDR + ReadAddr *2, 0x00F0);

	for (; NumWordToRead != 0x00; NumWordToRead--) /*!< while there is data to read */
	{
		/* Read a word from the NOR */
		*pBuffer++ = *(__IO uint16_t *) (Bank1_NORSRAM3_ADDR + ReadAddr * 2);
		ReadAddr++;
	}
}

/**
 * @brief  Returns the NOR memory to Read mode.
 * @param  None
 * @retval NOR_SUCCESS
 */
NORFLASH_Status NORFLASH_ReturnToReadMode(void) {
	NORFLASH_WRITE(Bank1_NORSRAM3_ADDR, 0x00F0);

	return (NOR_SUCCESS);
}

/**
 * @brief  Returns the NOR memory to Read mode and resets the errors in the NOR
 *         memory Status Register.
 * @param  None
 * @retval NOR_SUCCESS
 */
NORFLASH_Status NORFLASH_Reset(void) {
	NORFLASH_WRITE(0x00555, 0x00AA);
	NORFLASH_WRITE(0x002AA, 0x0055);
	NORFLASH_WRITE(0x00555, 0x00F0);

	return (NOR_SUCCESS);
}
/**
 * @brief  Returns the NOR operation status.
 * @param  Timeout: NOR programming Timeout
 * @retval NOR_Status: The returned value can be: NOR_SUCCESS, NOR_ERROR
 *         or NOR_TIMEOUT
 */
NORFLASH_Status NORFLASH_GetStatus(uint32_t Timeout) {
	uint16_t val1 = 0x00;
	uint16_t val2 = 0x00;
	NORFLASH_Status status = NOR_ONGOING;
	uint32_t timeout = Timeout;

	/*Poll on NOR memory Ready/Busy signal ----------------------------------*/
	while ((GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) != RESET) && (timeout > 0)) {
		timeout--;
	}

	timeout = Timeout;

	while ((GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) == RESET) && (timeout > 0)) {
		timeout--;
	}

	/*Get the NOR memory operation status -----------------------------------*/
	while ((Timeout != 0x00) && (status != NOR_SUCCESS)) {
		Timeout--;

		/*!< Read DQ6 and DQ5 */
		val1 = *(__IO uint16_t *) (Bank1_NORSRAM3_ADDR);
		val2 = *(__IO uint16_t *) (Bank1_NORSRAM3_ADDR);

		/*!< If DQ6 did not toggle between the two reads then return NOR_Success */
		if ((val1 & 0x0040) == (val2 & 0x0040)) {
			return NOR_SUCCESS;
		}

		if ((val1 & 0x0020) != 0x0020) {
			status = NOR_ONGOING;
		}

		val1 = *(__IO uint16_t *) (Bank1_NORSRAM3_ADDR);
		val2 = *(__IO uint16_t *) (Bank1_NORSRAM3_ADDR);

		if ((val1 & 0x0040) == (val2 & 0x0040)) {
			return NOR_SUCCESS;
		} else if ((val1 & 0x0020) == 0x0020) {
			return NOR_ERROR;
		}
	}

	if (Timeout == 0x00) {
		status = NOR_TIMEOUT;
	}

	/*Return the operation status */
	return (status);
}
