#ifndef HT1622_DEF
#define HT1622_DEF

#include "main.h"
#include "spi.h"

#define BIT_COMA_ENABLE		0xFF;
#define BIT_COMA_DISABLE	0xF;


uint8_t ht1622GenerateBit(uint8_t dataRaw);
void ht1622ProcessDataPrint(uint8_t * dataPrint, uint8_t len, uint8_t * dataBuffer);
uint8_t ht1622SizeOf(uint8_t * buffer);
void ht1622Write(uint8_t type, uint8_t column, uint8_t * dataPrint);
void integerToArray(uint8_t data, uint8_t * buffer);

#endif
