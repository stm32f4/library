#include "STM32F4xx_Conf.h"
#include "serial.h"
#include "5110.h"

#define BufferSize 0x1000

static callBack callFunction;
char RxBuffer[BufferSize];
char TxBuffer[BufferSize];
uint8_t MessageSize;
uint32_t wait = 0;

uint32_t nbCarTxr = 0;
uint32_t nbCarTxs = 0;

volatile uint16_t RxCounter;
volatile uint16_t TxCounter;
volatile uint8_t Sending = 0;
volatile uint8_t Writing = 0;
volatile uint16_t TxReadIndex = 0;
volatile uint16_t TxWriteIndex = 0;

void initUART4(uint32_t baudrate) {

	USART_InitTypeDef USART_config;
	GPIO_InitTypeDef GPIO_config;
	NVIC_InitTypeDef NVIC_InitStructure_UART;

	// Turn on peripheral clocks

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	// Configure TX pin on PA0

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);

	GPIO_StructInit(&GPIO_config);
	GPIO_config.GPIO_Pin = GPIO_Pin_0;
	GPIO_config.GPIO_Mode = GPIO_Mode_AF;
	GPIO_config.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_config.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_config);

	// Configure RX pin on PA1

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

	GPIO_StructInit(&GPIO_config);
	GPIO_config.GPIO_Pin = GPIO_Pin_1;
	GPIO_config.GPIO_Mode = GPIO_Mode_AF;
	GPIO_config.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_config);

	// Configure USART

	USART_StructInit(&USART_config);
	USART_config.USART_BaudRate = baudrate;
	USART_config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_config.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_config.USART_WordLength = USART_WordLength_8b;
	USART_config.USART_Parity = USART_Parity_No;
	USART_config.USART_StopBits = USART_StopBits_1;
	USART_Init(UART4, &USART_config);

	// Enable interrupts
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	USART_ITConfig(UART4, USART_IT_TXE, DISABLE);

	// Set interrupt priority
	NVIC_InitStructure_UART.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure_UART.NVIC_IRQChannelPreemptionPriority = 0xF;
	NVIC_InitStructure_UART.NVIC_IRQChannelSubPriority = 0xF;
	NVIC_Init(&NVIC_InitStructure_UART);

	// Enable USART

	USART_Cmd(UART4, ENABLE);

}

void UART4_IRQHandler_impl(void) {

	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) {

		/* Read one byte from the receive data register */
		RxBuffer[RxCounter] = (USART_ReceiveData(UART4));
		RxCounter++;
		callFunction(RxBuffer, RxCounter);
		if (RxCounter == BufferSize) {
			/* Disable the UART4 Receive interrupt */
			USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
		}
	}

	if (USART_GetITStatus(UART4, USART_IT_TXE) != RESET) {
		if (TxReadIndex == TxWriteIndex && Writing == 0) {
			/* Disable the UART4 Transmit interrupt */
			USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
			Sending = 0;
		} else {
			/* Write one byte to the transmit data register */
			Sending = 1;
			USART_SendData(UART4, TxBuffer[TxReadIndex]);
			nbCarTxs++;
			TxReadIndex++;
			if (TxReadIndex == BufferSize) {
				TxReadIndex = 0;
			}
		}
	}
}

void SerialSendBytes(char* str, uint16_t size) {
	Writing = 1;
	while (size > 0) {
		while (Sending && (TxWriteIndex + 1 == TxReadIndex)) {
			wait++;
		}
		TxBuffer[TxWriteIndex] = *str;
		TxWriteIndex++;
		nbCarTxr++;
		if (TxWriteIndex == BufferSize) {
			TxWriteIndex = 0;
		}
		size--;
		str++;
	}
	Writing = 0;
	if (!Sending) {
		Sending = 1;
		USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
	}
}
void SerialSetCallBack(callBack function) {
	callFunction = function;
}

void SerialClearInputBuffer() {
	RxCounter = 0;
}

