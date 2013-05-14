#include "ascii.h"

char digitToAscii[16] = { 0x30, 0X31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0X38,
		0x39, 0x61, 0X62, 0x63, 0x64, 0x65, 0x66 };

void getDecimalFromShort(char* values, unsigned short value, uint8_t size) {
	char digit[5];
	uint16_t factors[5] = { 10000, 1000, 100, 10, 1 };
	int i;
	uint8_t zero = 1;

	for (i = 0; i < 5; i++) {
		digit[i] = value / factors[i];
		value = value - digit[i] * factors[i];

		if (zero == 1 && digit[i] == 0 && i < 4) {
			digit[i] = 32;
		} else {
			zero = 0;
			digit[i] += 48;
		}
	}
	memcpy(values, digit + (5 - size), size);
}

void getHexFromLong(char* values, unsigned long value, uint8_t nbDigits) {
	char digits[8];
	int i;
	uint8_t zero = 1;

	for (i = 0; i < 8; i++) {
		digits[i] = value >> ((7 - i) * 4);
		value = value - (digits[i] << ((7 - i) * 4));

		if (zero == 1 && digits[i] == 0) {
			digits[i] = 32;
		} else {
			zero = 0;
			digits[i] = digitToAscii[digits[i]];
		}
	}
	memcpy(values, digits + (8 - nbDigits), nbDigits);
}
