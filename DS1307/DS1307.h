void DS1307Init();
uint8_t DS1307Read(uint8_t address);
void DS1307Write(uint8_t address, uint8_t value);
void DS1307ReadTime(DATE_TYPE * date);
void DS1307SetTime();
void DS1307GetTimeString(DATE_TYPE* date, char* time);
void DS1307GetDateString(DATE_TYPE* date, char* time);
