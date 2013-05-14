typedef int (*callBack)(char*,uint16_t);


void initUART4(uint32_t baudrate);
void SerialSendBytes(char* str, uint16_t size);
void SerialSetCallBack(callBack function);
void SerialClearInputBuffer();
void UART4_IRQHandler_impl(void);
