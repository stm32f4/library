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

/**
 ***************************************
 *  Define the I2C port to use
 ***************************************
 */

#define I2C_PORT I2C2
#define GPIO GPIOB
#define GPIO_AF_I2C_PORT GPIO_AF_I2C2
#define RCC_AHB1Periph_GPIO RCC_AHB1Periph_GPIOB
#define RCC_APB1Periph_I2C RCC_APB1Periph_I2C2
#define GPIO_PinSource_SCL GPIO_PinSource10
#define GPIO_PinSource_SDA GPIO_PinSource11
#define GPIO_Pin_SCL GPIO_Pin_10
#define GPIO_Pin_SDA GPIO_Pin_11
#define I2C_SLAVE_ADDRESS    0b11010000
#define I2C_MASTER_ADDRESS   0x00

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
	I2C_ITConfig(I2C_PORT, I2C_IT_BUF | I2C_IT_EVT, ENABLE);
}

void I2CInit() {

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO, ENABLE);

	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C, ENABLE);

	GPIO_PinAFConfig(GPIO, GPIO_PinSource_SCL, GPIO_AF_I2C_PORT);
	GPIO_PinAFConfig(GPIO, GPIO_PinSource_SDA, GPIO_AF_I2C_PORT);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SCL | GPIO_Pin_SDA;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIO, &GPIO_InitStructure);

	I2C_InitStructure.I2C_ClockSpeed = 10000;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = I2C_MASTER_ADDRESS;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	/* Enable I2C */
	I2C_Cmd(I2C_PORT, ENABLE);
	I2C_Init(I2C_PORT, &I2C_InitStructure);

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

	// Wait for the bus to be free
	while (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_BUSY))
		;

	I2C_AcknowledgeConfig(I2C_PORT, ENABLE);

	// ************************ Select slave and write register address *********************
	// I2C Start
	I2C_GenerateSTART(I2C_PORT, ENABLE);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT))
		;

	// Select slave and write mode
	I2C_Send7bitAddress(I2C_PORT, I2C_SLAVE_ADDRESS, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;

	// Set register address
	I2C_SendData(I2C_PORT, 0x00);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	// ************************ Select slave in read mode *********************
	// I2C Start
	I2C_GenerateSTART(I2C_PORT, ENABLE);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT))
		;

	// Select slave and read mode
	I2C_Send7bitAddress(I2C_PORT, I2C_SLAVE_ADDRESS, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		;

	// Read bytes
	for (index = 0; index < 8; index++) {
		while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_RECEIVED))
			;
		values[index] = I2C_ReceiveData(I2C_PORT);
		if (index == 6) {
			I2C_NACKPositionConfig(I2C_PORT, I2C_NACKPosition_Current);
			I2C_AcknowledgeConfig(I2C_PORT, DISABLE);
		}
	}

	// **************************  I2C Stop **************************************
	I2C_GenerateSTOP(I2C_PORT, ENABLE);
	while (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_STOPF))
		;

	// Get date / time
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
	date->weekDay = values[3];
}

/**
 * Set date time
 */
void DS1307SetTime(DATE_TYPE *date) {
	uint8_t index;
	uint8_t time[8];

	time[0] = (date->sec1 << 4) | (date->sec0);
	time[1] = (date->min1 << 4) | (date->min0);
	time[2] = (date->hour1 << 4) | (date->hour0);
	time[3] = (date->weekDay);
	time[4] = (date->day1 << 4) | (date->day0);
	time[5] = (date->month1 << 4) | (date->month0);
	time[6] = (date->year1 << 4) | (date->year0);
	time[7] = 0x10;

	// BCD : Seconds, Minutes, Hours, Day, Date , Month, Year
	//uint8_t time[8] = { 0x30, 0x12, 0x00, 0x02, 0x24, 0x12, 0x13, 0x10 };

	// Wait for the bus to be free
	while (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_BUSY))
		;

	I2C_AcknowledgeConfig(I2C_PORT, ENABLE);

	// I2C start
	I2C_GenerateSTART(I2C_PORT, ENABLE);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT))
		;

	// Select slave in write mode
	I2C_Send7bitAddress(I2C_PORT, I2C_SLAVE_ADDRESS, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;

	// Set register address to 0x00
	I2C_SendData(I2C_PORT, 0x00);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	// Write loop
	for (index = 0; index < 8; index++) {
		I2C_SendData(I2C_PORT, time[index]);
		while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			;
	}

	// No ACK
	I2C_NACKPositionConfig(I2C_PORT, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2C_PORT, DISABLE);

	// ************************ I2C Stop *************************************
	I2C_GenerateSTOP(I2C_PORT, ENABLE);
	while (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_STOPF))
		;
}

/**
 * Convert time to string
 */
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

/**
 * Convert date to string
 */
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

/**
 * Handler for I2C events
 */
void I2C2_EV_IRQHandler() {
	uint32_t event = I2C_GetLastEvent(I2C_PORT);
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

/**
 * Write one byte
 */
void DS1307Write(uint8_t address, uint8_t value) {

	// Wait for the bus to be free
	while (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_BUSY))
		;

	I2C_AcknowledgeConfig(I2C_PORT, ENABLE);

	GPIO_ResetBits(GPIOF, GPIO_Pin_6);

	// I2C start
	I2C_GenerateSTART(I2C_PORT, ENABLE);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT))
		;

	// Select slave in write mode
	I2C_Send7bitAddress(I2C_PORT, I2C_SLAVE_ADDRESS, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;

	// Set register address
	I2C_SendData(I2C_PORT, address);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	// Write loop
	I2C_SendData(I2C_PORT, value);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	// No ACK
	I2C_NACKPositionConfig(I2C_PORT, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2C_PORT, DISABLE);

	// ************************ I2C Stop *************************************
	I2C_GenerateSTOP(I2C_PORT, ENABLE);
	while (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_STOPF))
		;
}

/**
 * Read one byte
 */
uint8_t DS1307Read(uint8_t address) {

	uint8_t value;

	// Wait for the bus to be free
	while (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_BUSY))
		;

	I2C_AcknowledgeConfig(I2C_PORT, ENABLE);

	// I2C Start
	I2C_GenerateSTART(I2C_PORT, ENABLE);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT))
		;

	// Select slave
	I2C_Send7bitAddress(I2C_PORT, I2C_SLAVE_ADDRESS, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;

	// Send register address
	I2C_SendData(I2C_PORT, address);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	// I2C Start
	I2C_GenerateSTART(I2C_PORT, ENABLE);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_MODE_SELECT))
		;

	// Select slave address
	I2C_Send7bitAddress(I2C_PORT, I2C_SLAVE_ADDRESS, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		;

	// Read byte
	while (!I2C_CheckEvent(I2C_PORT, I2C_EVENT_MASTER_BYTE_RECEIVED))
		;
	value = I2C_ReceiveData(I2C_PORT);

	// NACK
	I2C_NACKPositionConfig(I2C_PORT, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2C_PORT, DISABLE);

	// I2C Stop
	I2C_GenerateSTOP(I2C_PORT, ENABLE);
	while (I2C_GetFlagStatus(I2C_PORT, I2C_FLAG_STOPF))
		;

	return value;
}
