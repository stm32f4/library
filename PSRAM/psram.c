/* Includes */
#include "psram.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_fsmc.h"

/* Private macro -------------------------------------------------------------*/
#define Bank1_SRAM2_ADDR  ((uint32_t)0x64000000)

void PSRAM_Init() {
	/* Private typedef ---------------------------------------------------------*/
	GPIO_InitTypeDef GPIO_InitStruct;
	FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef r;
	FSMC_NORSRAMTimingInitTypeDef w;

	/* FSMC_NOR_RAM GPIO Configuration

	 PD0	 ------> FSMC_NOR_RAM_D2
	 PD1	 ------> FSMC_NOR_RAM_D3
	 PD4	 ------> FSMC_NOR_RAM_NOE
	 PD5	 ------> FSMC_NOR_RAM_NWE
	 PD8	 ------> FSMC_NOR_RAM_D13
	 PD9	 ------> FSMC_NOR_RAM_D14
	 PD10	 ------> FSMC_NOR_RAM_D15
	 PD11	 ------> FSMC_NOR_RAM_A16
	 PD12	 ------> FSMC_NOR_RAM_A17
	 PD13	 ------> FSMC_NOR_RAM_A18
	 PD14	 ------> FSMC_NOR_RAM_D0
	 PD15	 ------> FSMC_NOR_RAM_D1

	 PE0	 ------> FSMC_NOR_RAM_NBL0
	 PE1	 ------> FSMC_NOR_RAM_NBL1
	 PE3	 ------> FSMC_NOR_RAM_A19
	 PE4	 ------> FSMC_NOR_RAM_A20
	 PE5	 ------> FSMC_NOR_RAM_A21
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
	 PG9	 ------> FSMC_NOR_RAM_NE2

	 */

	/*Enable or disable the AHB1 peripheral clock */
	RCC_AHB1PeriphClockCmd(
			RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF
					| RCC_AHB1Periph_GPIOG, ENABLE);

	/* Enable FSMC Clock */
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

	/*Configure GPIO pin alternate function */
	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_FSMC); only for NOR
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
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource3, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_FSMC);
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
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_FSMC);

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
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4
			| GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10
			| GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14
			| GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO F pin */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
			| GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14
			| GPIO_Pin_15;
	GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO G pin */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
			| GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_9;
	GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*-- FSMC Configuration ------------------------------------------------------*/
	r.FSMC_AddressSetupTime = 0; //
	r.FSMC_AddressHoldTime = 0; //
	r.FSMC_DataSetupTime = 8; //8*6=48ns
	r.FSMC_BusTurnAroundDuration = 0; //
	r.FSMC_CLKDivision = 0; //
	r.FSMC_DataLatency = 0; //
	r.FSMC_AccessMode = FSMC_AccessMode_A;

	w.FSMC_AddressSetupTime = 0; //
	w.FSMC_AddressHoldTime = 0; //
	w.FSMC_DataSetupTime = 5; //5*6=30ns
	w.FSMC_BusTurnAroundDuration = 0; //
	w.FSMC_CLKDivision = 0; //
	w.FSMC_DataLatency = 0; //
	w.FSMC_AccessMode = FSMC_AccessMode_A;

	/* FSMC parameters */
	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM2;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode =
			FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait =
			FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity =
			FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive =
			FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &r;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &w;

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

	/* Enable FSMC Bank1_SRAM2 Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE);
}

/**
 * @brief  Writes a Half-word buffer to the FSMC SRAM memory.
 * @param  pBuffer : pointer to buffer.
 * @param  WriteAddr : SRAM memory internal address from which the data will be
 *         written.
 * @param  NumHalfwordToWrite : number of half-words to write.
 * @retval None
 */
void PSRAM_WriteBuffer(uint16_t* pBuffer, uint32_t WriteAddr,
		uint32_t NumHalfwordToWrite) {
	for (; NumHalfwordToWrite != 0; NumHalfwordToWrite--) /* while there is data to write */
	{
		/* Transfer data to the memory */
		*(uint16_t *) (Bank1_SRAM2_ADDR + WriteAddr * 2) = *pBuffer++;

		/* Increment the address*/
		WriteAddr++;
	}
}
void PSRAM_WriteWord(uint16_t bytes, uint32_t WriteAddr) {
	*(uint16_t *) (Bank1_SRAM2_ADDR + WriteAddr * 2) = bytes;
}

/**
 * @brief  Reads a block of data from the FSMC SRAM memory.
 * @param  pBuffer : pointer to the buffer that receives the data read from the
 *         SRAM memory.
 * @param  ReadAddr : SRAM memory internal address to read from.
 * @param  NumHalfwordToRead : number of half-words to read.
 * @retval None
 */
void PSRAM_ReadBuffer(uint16_t* pBuffer, uint32_t ReadAddr,
		uint32_t NumHalfwordToRead) {
	for (; NumHalfwordToRead != 0; NumHalfwordToRead--) /* while there is data to read */
	{
		/* Read a half-word from the memory */
		*pBuffer++ = *(__IO uint16_t*) (Bank1_SRAM2_ADDR + ReadAddr * 2);

		/* Increment the address*/
		ReadAddr++;
	}
}

uint16_t PSRAM_ReadWord(uint32_t ReadAddr) {
	return *(uint16_t*) (Bank1_SRAM2_ADDR + ReadAddr * 2);
}
