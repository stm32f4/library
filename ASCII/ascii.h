#ifndef __ASCII_H
#define __ASCII_H

//******************************************************
//             Includes
//******************************************************
#include "stm32f4xx.h"
#include "stm32f4xx.h"
#include "string.h"
//******************************************************

void getDecimalFromShort(char* buffer, unsigned short value, uint8_t nbDigits);
void getHexFromLong(char* values, unsigned long value, uint8_t nbDigits);

#endif
