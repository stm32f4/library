/**
 *****************************************************************************
 * @title   I2C_Exp.c
 * @author  CooCox
 * @date    31 Oct 2012
 * @brief
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "main.h"
#include "ascii.h"

#define I2C2_SLAVE_ADDRESS    0b11010000
#define I2C2_MASTER_ADDRESS   0x00
typedef enum {
	INIT = 0, SELECT_REG0,

} READ_STATE;

/* Private variables ---------------------------------------------------------*/
READ_STATE state;
I2C_InitTypeDef I2C_InitStructure;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void I2CInit();

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
void DS1307Init() {

	I2CInit();
	I2C_ITConfig(I2C2, I2C_IT_BUF | I2C_IT_EVT, ENABLE);
}

void I2CInit() {

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitStructure.I2C_ClockSpeed = 10000;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = I2C2_MASTER_ADDRESS;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	/* Enable I2C */
	I2C_Cmd(I2C2, ENABLE);
	I2C_Init(I2C2, &I2C_InitStructure);

	/* Configure the SPI interrupt priority */
//	NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
}

void DS1307ReadTime(DATE_TYPE* date) {
	uint8_t index;
	uint8_t values[8] = { 0 };

	I2C_AcknowledgeConfig(I2C2, ENABLE);

	// Start setting register
	I2C_GenerateSTART(I2C2, ENABLE);
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
		;

	I2C_Send7bitAddress(I2C2, I2C2_SLAVE_ADDRESS, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;

	I2C_SendData(I2C2, 0x00);
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	I2C_GenerateSTART(I2C2, ENABLE);
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
		;

	I2C_Send7bitAddress(I2C2, I2C2_SLAVE_ADDRESS, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		;

	for (index = 0; index < 8; index++) {
		while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
			;
		values[index] = I2C_ReceiveData(I2C2);
		if (index == 6) {
			I2C_NACKPositionConfig(I2C2, I2C_NACKPosition_Current);
			I2C_AcknowledgeConfig(I2C2, DISABLE);
		}
	}

	I2C_GenerateSTOP(I2C2, ENABLE);
	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF))
		;

	date->sec1 = (values[0] & 0xF0) >> 4;
	date->sec0 = values[0] & 0x0F;
	date->min1 = (values[1] & 0xF0) >> 4;
	date->min0 = values[1] & 0x0F;
	date->hour1 = (values[2] & 0x30) >> 4;
	date->hour0 = values[2] & 0x0F;
	date->day1 = (values[4] & 0x30) >> 4;
	date->day0 = values[4] & 0x0F;
	date->month1 = (values[5] & 0x10) >> 4;
	date->month0 = values[5] & 0x0F;
	date->year1 = (values[6] & 0xF0) >> 4;
	date->year0 = values[6] & 0x0F;
}

void DS1307SetTime() {
	uint8_t index;
	uint8_t values[8] = { 0x00, 0x15, 0x00, 0x01, 0x15, 0x05, 0x13, 0x10 };

	I2C_AcknowledgeConfig(I2C2, ENABLE);

	// Start setting register
	I2C_GenerateSTART(I2C2, ENABLE);
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
		;

	I2C_Send7bitAddress(I2C2, I2C2_SLAVE_ADDRESS, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;

	I2C_SendData(I2C2, 0x00);
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	for (index = 0; index < 8; index++) {
		I2C_SendData(I2C2, values[index]);
		while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			;
	}

	I2C_NACKPositionConfig(I2C2, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2C2, DISABLE);

	I2C_GenerateSTOP(I2C2, ENABLE);

	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF))
		;
}

void DS1307GetTimeString(DATE_TYPE* date, char* time) {
	time[0] = date->hour1 + 0x30;
	time[1] = date->hour0 + 0x30;
	time[2] = 0x3A;
	time[3] = date->min1 + 0x30;
	time[4] = date->min0 + 0x30;
	time[5] = 0x3A;
	time[6] = date->sec1 + 0x30;
	time[7] = date->sec0 + 0x30;
	time[8] = 0x00;
}

void DS1307GetDateString(DATE_TYPE* date, char* time) {
	time[0] = date->day1 + 0x30;
	time[1] = date->day0 + 0x30;
	time[2] = 0x2F;
	time[3] = date->month1 + 0x30;
	time[4] = date->month0 + 0x30;
	time[5] = 0x2F;
	time[6] = date->year1 + 0x30;
	time[7] = date->year0 + 0x30;
	time[8] = 0x00;
}

void I2C2_EV_IRQHandler() {
	uint32_t event = I2C_GetLastEvent(I2C2);
	switch (event) {
	case I2C_EVENT_MASTER_MODE_SELECT:
		break;
	case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
		break;
	case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
		break;
	case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
		break;
	case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
		break;
	case I2C_EVENT_MASTER_BYTE_RECEIVED:
		break;
	default:
		break;
	}
}

