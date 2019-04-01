

/* Prevent recursive inclusion */
#ifndef __PSRAM_H
#define __PSRAM_H


/* Includes */
#include "stm32f4xx.h"

void PSRAM_Config();
void PSRAM_Init();
void PSRAM_DeInit();

void PSRAM_ReadBuffer(uint16_t* pBuffer, uint32_t ReadAddr, uint32_t NumHalfwordToRead);
void PSRAM_WriteBuffer(uint16_t* pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite);
void PSRAM_WriteWord(uint16_t bytes, uint32_t WriteAddr);
uint16_t PSRAM_ReadWord(uint32_t ReadAddr);
#endif
