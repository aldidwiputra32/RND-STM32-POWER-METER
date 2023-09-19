#include "HT1622.h"
#include "stm32f0xx_hal.h"
#include "main.h"

#define  CS1 		   1
#define  CS2           2
#define  WR_LOW        (HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET))
#define  WR_HIGH       (HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET))
#define  DATA_LOW      (HAL_GPIO_WritePin(LCD_DATA_GPIO_Port, LCD_DATA_Pin, GPIO_PIN_RESET))
#define  DATA_HIGH     (HAL_GPIO_WritePin(LCD_DATA_GPIO_Port, LCD_DATA_Pin, GPIO_PIN_SET))

uint8_t mapSegmentIC1[56][4] = {
		// Indeks Kolom | segment | COM | segment
		{1,	1,	0,	23},
		{1,	2,	0,	24},
		{1,	3,	2,	24},
		{1,	4,	3,	23},
		{1,	5,	2,	23},
		{1,	6,	1,	23},
		{1,	7,	1,	24},
		{2,	1,	0,	25},
		{2,	2,	0,	26},
		{2,	3,	2,	26},
		{2,	4,	3,	25},
		{2,	5,	2,	25},
		{2,	6,	1,	25},
		{2,	7,	1,	26},
		{3,	1,	0,	27},
		{3,	2,	0,	28},
		{3,	3,	2,	28},
		{3,	4,	3,	27},
		{3,	5,	2,	27},
		{3,	6,	1,	27},
		{3,	7,	1,	28},
		{4,	1,	0,	29},
		{4,	2,	0,	30},
		{4,	3,	2,	30},
		{4,	4,	3,	29},
		{4,	5,	2,	29},
		{4,	6,	1,	29},
		{4,	7,	1,	30},
		{5,	1,	4,	23},
		{5,	2,	4,	24},
		{5,	3,	6,	24},
		{5,	4,	7,	23},
		{5,	5,	6,	23},
		{5,	6,	5,	23},
		{5,	7,	5,	24},
		{6,	1,	4,	25},
		{6,	2,	4,	26},
		{6,	3,	6,	26},
		{6,	4,	7,	25},
		{6,	5,	6,	25},
		{6,	6,	5,	25},
		{6,	7,	5,	26},
		{7,	1,	4,	27},
		{7,	2,	4,	28},
		{7,	3,	6,	28},
		{7,	4,	7,	27},
		{7,	5,	6,	27},
		{7,	6,	5,	27},
		{7,	7,	5,	28},
		{8,	1,	4,	29},
		{8,	2,	4,	30},
		{8,	3,	6,	30},
		{8,	4,	7,	29},
		{8,	5,	6,	29},
		{8,	6,	5,	29},
		{8,	7,	5,	30}
};

uint8_t mapSegmentIC2[56][4] = {
		// Indeks Kolom | segment | COM | segment
		{9,		1,	7,	23},
		{9,		2,	7,	24},
		{9,		3,	5,	24},
		{9,		4,	4,	23},
		{9,		5,	5,	23},
		{9,		6,	6,	23},
		{9,		7,	6,	24},
		{10,	1,	7,	25},
		{10,	2,	7,	26},
		{10,	3,	5,	26},
		{10,	4,	4,	25},
		{10,	5,	5,	25},
		{10,	6,	6,	25},
		{10,	7,	6,	26},
		{11,	1,	7,	27},
		{11,	2,	7,	28},
		{11,	3,	5,	28},
		{11,	4,	4,	27},
		{11,	5,	5,	27},
		{11,	6,	6,	27},
		{11,	7,	6,	28},
		{12,	1,	7,	29},
		{12,	2,	7,	30},
		{12,	3,	5,	30},
		{12,	4,	4,	29},
		{12,	5,	5,	29},
		{12,	6,	6,	29},
		{12,	7,	6,	30},
		{13,	1,	3,	23},
		{13,	2,	3,	24},
		{13,	3,	1,	24},
		{13,	4,	0,	23},
		{13,	5,	1,	23},
		{13,	6,	2,	23},
		{13,	7,	2,	24},
		{14,	1,	3,	25},
		{14,	2,	3,	26},
		{14,	3,	1,	26},
		{14,	4,	0,	25},
		{14,	5,	1,	25},
		{14,	6,	2,	25},
		{14,	7,	2,	26},
		{15,	1,	3,	27},
		{15,	2,	3,	28},
		{15,	3,	1,	28},
		{15,	4,	0,	27},
		{15,	5,	1,	27},
		{15,	6,	2,	27},
		{15,	7,	2,	28},
		{16,	1,	3,	29},
		{16,	2,	3,	30},
		{16,	3,	1,	30},
		{16,	4,	0,	29},
		{16,	5,	1,	29},
		{16,	6,	2,	29},
		{16,	7,	2,	30}
};

uint8_t mapSegmentEnergy[63][4] = {
		// Indeks Kolom | segment | COM | segment
		{17,	1,	2,	11},
		{17,	2,	3,	10},
		{17,	3,	5,	10},
		{17,	4,	5,	11},
		{17,	5,	4,	11},
		{17,	6,	3,	11},
		{17,	7,	4,	10},
		{18,	1,	4,	9},
		{18,	2,	4,	8},
		{18,	3,	6,	8},
		{18,	4,	7,	9},
		{18,	5,	6,	9},
		{18,	6,	5,	9},
		{18,	7,	5,	8},
		{19,	1,	4,	7},
		{19,	2,	4,	6},
		{19,	3,	6,	6},
		{19,	4,	7,	7},
		{19,	5,	6,	7},
		{19,	6,	5,	7},
		{19,	7,	5,	6},
		{20,	1,	4,	5},
		{20,	2,	4,	4},
		{20,	3,	6,	4},
		{20,	4,	7,	5},
		{20,	5,	6,	5},
		{20,	6,	5,	5},
		{20,	7,	5,	4},
		{21,	1,	4,	3},
		{21,	2,	5,	2},
		{21,	3,	7,	2},
		{21,	4,	7,	3},
		{21,	5,	6,	3},
		{21,	6,	5,	3},
		{21,	7,	6,	2},
		{22,	1,	3,	2},
		{22,	2,	3,	3},
		{22,	3,	1,	3},
		{22,	4,	0,	2},
		{22,	5,	1,	2},
		{22,	6,	2,	2},
		{22,	7,	2,	3},
		{23,	1,	3,	4},
		{23,	2,	3,	5},
		{23,	3,	1,	5},
		{23,	4,	0,	4},
		{23,	5,	1,	4},
		{23,	6,	2,	4},
		{23,	7,	2,	5},
		{24,	1,	3,	6},
		{24,	2,	3,	7},
		{24,	3,	1,	7},
		{24,	4,	0,	6},
		{24,	5,	1,	6},
		{24,	6,	2,	6},
		{24,	7,	2,	7},
		{25,	1,	3,	8},
		{25,	2,	3,	9},
		{25,	3,	1,	9},
		{25,	4,	0,	8},
		{25,	5,	1,	8},
		{25,	6,	2,	8},
		{25,	7,	2,	9}
};

uint8_t mapSegmentPointIC1[6][3] = {
		// Indeks | COM | Segment
		{1,	3,	24},
		{2,	3,	26},
		{3,	3,	28},
		{4,	7,	24},
		{5,	7,	26},
		{6,	7,	28}
};

uint8_t mapSegmentPointIC2[6][3] = {
		// Indeks | COM | Segment
		{7,		4,	24},
		{8,		4,	26},
		{9,		4,	28},
		{10,	0,	24},
		{11,	0,	26},
		{12,	0,	28}
};

uint8_t mapSegmentPointEnergy[8][3] = {
		{13,	6,	10},
		{14,	7,	8},
		{15,	7,	6},
		{16,	7,	4},
		{17,	4,	1},
		{18,	0,	3},
		{19,	1,	1},
		{20,	0,	7}
};

uint8_t mapSegmentUnitIC1[6][3] = {
		// Indeks | COM | Segment
		{1,	0,	31},
		{2,	1,	31},
		{3,	3,	31},
		{4,	4,	31},
		{5,	5,	31},
		{6,	7,	31}
};

uint8_t mapSegmentUnitIC2[8][3] = {
		// Indeks | COM | Segment
		{7,		7,	31},
		{8,		6,	31},
		{9,		5,	31},
		{10,	4,	31},
		{11,	4,	30},
		{12,	3,	31},
		{13,	2,	31},
		{14,	1,	31}
};

uint8_t mapSegmentUnitTotal[4][3] = {
		// Indeks | COM | Segment
		{1,	1,	12},
		{2,	0,	10},
		{3,	1,	10},
		{4,	2,	10}
};

uint8_t lcdRam11[32][7]={
	   //SEG | ADDRES(SEG) | DATA(COM0,COM1,COM2.COM3) | FLAG
		{0,		0,	0,	0,	0,	0,	0},
		{1,		2,	0,	0,	0,	0,	0},
		{2,		4,	0,	0,	0,	0,	0},
		{3,		6,	0,	0,	0,	0,	0},
		{4,		8,	0,	0,	0,	0,	0},
		{5,		10,	0,	0,	0,	0,	0},
		{6,		12,	0,	0,	0,	0,	0},
		{7,		14,	0,	0,	0,	0,	0},
		{8,		16,	0,	0,	0,	0,	0},
		{9,		18,	0,	0,	0,	0,	0},
		{10,	20,	0,	0,	0,	0,	0},
		{11,	22,	0,	0,	0,	0,	0},
		{12,	24,	0,	0,	0,	0,	0},
		{13,	26,	0,	0,	0,	0,	0},
		{14,	28,	0,	0,	0,	0,	0},
		{15,	30,	0,	0,	0,	0,	0},
		{16,	32,	0,	0,	0,	0,	0},
		{17,	34,	0,	0,	0,	0,	0},
		{18,	36,	0,	0,	0,	0,	0},
		{19,	38,	0,	0,	0,	0,	0},
		{20,	40,	0,	0,	0,	0,	0},
		{21,	42,	0,	0,	0,	0,	0},
		{22,	44,	0,	0,	0,	0,	0},
		{23,	46,	0,	0,	0,	0,	0},
		{24,	48,	0,	0,	0,	0,	0},
		{25,	50,	0,	0,	0,	0,	0},
		{26,	52,	0,	0,	0,	0,	0},
		{27,	54,	0,	0,	0,	0,	0},
		{28,	56,	0,	0,	0,	0,	0},
		{29,	58,	0,	0,	0,	0,	0},
		{30,	60,	0,	0,	0,	0,	0},
		{31,	62,	0,	0,	0,	0,	0}
};

uint8_t lcdRam12[32][7]={
		//SEG | ADDRES(SEG) | DATA(COM4,COM5,COM6.COM7)
		{0,		1,	0,	0,	0,	0,	0},
		{1,		3,	0,	0,	0,	0,	0},
		{2,		5,	0,	0,	0,	0,	0},
		{3,		7,	0,	0,	0,	0,	0},
		{4,		9,	0,	0,	0,	0,	0},
		{5,		11,	0,	0,	0,	0,	0},
		{6,		13,	0,	0,	0,	0,	0},
		{7,		15,	0,	0,	0,	0,	0},
		{8,		17,	0,	0,	0,	0,	0},
		{9,		19,	0,	0,	0,	0,	0},
		{10,	21,	0,	0,	0,	0,	0},
		{11,	23,	0,	0,	0,	0,	0},
		{12,	25,	0,	0,	0,	0,	0},
		{13,	27,	0,	0,	0,	0,	0},
		{14,	29,	0,	0,	0,	0,	0},
		{15,	31,	0,	0,	0,	0,	0},
		{16,	33,	0,	0,	0,	0,	0},
		{17,	35,	0,	0,	0,	0,	0},
		{18,	37,	0,	0,	0,	0,	0},
		{19,	39,	0,	0,	0,	0,	0},
		{20,	41,	0,	0,	0,	0,	0},
		{21,	43,	0,	0,	0,	0,	0},
		{22,	45,	0,	0,	0,	0,	0},
		{23,	47,	0,	0,	0,	0,	0},
		{24,	49,	0,	0,	0,	0,	0},
		{25,	51,	0,	0,	0,	0,	0},
		{26,	53,	0,	0,	0,	0,	0},
		{27,	55,	0,	0,	0,	0,	0},
		{28,	57,	0,	0,	0,	0,	0},
		{29,	59,	0,	0,	0,	0,	0},
		{30,	61,	0,	0,	0,	0,	0},
		{31,	63,	0,	0,	0,	0,	0}
};

uint8_t lcdRam21[32][7]={
	   //SEG | ADDRES(SEG) | DATA(COM0,COM1,COM2.COM3)
		{0,		0,	0,	0,	0,	0,	0},
		{1,		2,	0,	0,	0,	0,	0},
		{2,		4,	0,	0,	0,	0,	0},
		{3,		6,	0,	0,	0,	0,	0},
		{4,		8,	0,	0,	0,	0,	0},
		{5,		10,	0,	0,	0,	0,	0},
		{6,		12,	0,	0,	0,	0,	0},
		{7,		14,	0,	0,	0,	0,	0},
		{8,		16,	0,	0,	0,	0,	0},
		{9,		18,	0,	0,	0,	0,	0},
		{10,	20,	0,	0,	0,	0,	0},
		{11,	22,	0,	0,	0,	0,	0},
		{12,	24,	0,	0,	0,	0,	0},
		{13,	26,	0,	0,	0,	0,	0},
		{14,	28,	0,	0,	0,	0,	0},
		{15,	30,	0,	0,	0,	0,	0},
		{16,	32,	0,	0,	0,	0,	0},
		{17,	34,	0,	0,	0,	0,	0},
		{18,	36,	0,	0,	0,	0,	0},
		{19,	38,	0,	0,	0,	0,	0},
		{20,	40,	0,	0,	0,	0,	0},
		{21,	42,	0,	0,	0,	0,	0},
		{22,	44,	0,	0,	0,	0,	0},
		{23,	46,	0,	0,	0,	0,	0},
		{24,	48,	0,	0,	0,	0,	0},
		{25,	50,	0,	0,	0,	0,	0},
		{26,	52,	0,	0,	0,	0,	0},
		{27,	54,	0,	0,	0,	0,	0},
		{28,	56,	0,	0,	0,	0,	0},
		{29,	58,	0,	0,	0,	0,	0},
		{30,	60,	0,	0,	0,	0,	0},
		{31,	62,	0,	0,	0,	0,	0}
};

uint8_t lcdRam22[32][7]={
		//SEG | ADDRES(SEG) | DATA(COM4,COM5,COM6.COM7)
		{0,		1,	0,	0,	0,	0,	0},
		{1,		3,	0,	0,	0,	0,	0},
		{2,		5,	0,	0,	0,	0,	0},
		{3,		7,	0,	0,	0,	0,	0},
		{4,		9,	0,	0,	0,	0,	0},
		{5,		11,	0,	0,	0,	0,	0},
		{6,		13,	0,	0,	0,	0,	0},
		{7,		15,	0,	0,	0,	0,	0},
		{8,		17,	0,	0,	0,	0,	0},
		{9,		19,	0,	0,	0,	0,	0},
		{10,	21,	0,	0,	0,	0,	0},
		{11,	23,	0,	0,	0,	0,	0},
		{12,	25,	0,	0,	0,	0,	0},
		{13,	27,	0,	0,	0,	0,	0},
		{14,	29,	0,	0,	0,	0,	0},
		{15,	31,	0,	0,	0,	0,	0},
		{16,	33,	0,	0,	0,	0,	0},
		{17,	35,	0,	0,	0,	0,	0},
		{18,	37,	0,	0,	0,	0,	0},
		{19,	39,	0,	0,	0,	0,	0},
		{20,	41,	0,	0,	0,	0,	0},
		{21,	43,	0,	0,	0,	0,	0},
		{22,	45,	0,	0,	0,	0,	0},
		{23,	47,	0,	0,	0,	0,	0},
		{24,	49,	0,	0,	0,	0,	0},
		{25,	51,	0,	0,	0,	0,	0},
		{26,	53,	0,	0,	0,	0,	0},
		{27,	55,	0,	0,	0,	0,	0},
		{28,	57,	0,	0,	0,	0,	0},
		{29,	59,	0,	0,	0,	0,	0},
		{30,	61,	0,	0,	0,	0,	0},
		{31,	63,	0,	0,	0,	0,	0}
};

uint8_t alphaNumeric[38][2]={
		// CHAR | BIT DISPLAY SEVEN SEGMENT
		{'0',	63},
		{'1',	6},
		{'2',	91},
		{'3',	79},
		{'4',	102},
		{'5',	109},
		{'6',	125},
		{'7',	7},
		{'8',	127},
		{'9',	111},
		{'a', 	119},
		{'b',	124},
		{'c', 	57},
		{'d',	94},
		{'e',	121},
		{'f',	113},
		{'g',	61},
		{'h',	116},
		{'i',	4},
		{'j',	14},
		{'k',	117},
		{'l',	56},
		{'m',	85},
		{'n',	84},
		{'o',	92},
		{'p',	115},
		{'q',	103},
		{'r',	80},
		{'s',	109},
		{'t',	120},
		{'u',	62},
		{'v',	28},
		{'w',	106},
		{'x',	118},
		{'y',	110},
		{'z',	91},
		{'.',	255},
		{'/',	0}
};

uint8_t ht1622BuferWrite[3][13] = {
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

uint8_t dataWriteLcd[128][2];
uint8_t dataBuffer[7];


void ht1622ProcessDataPrint(uint8_t type ,uint8_t * dataPrint, uint8_t len, uint8_t * dataBuffer){
	dataBuffer[0] = dataPrint[0];
	uint8_t indeksBuffer = 1;
	for(uint8_t indeks=1;indeks<len;indeks++){
		if((dataPrint[indeks] != '.') && (dataPrint[indeks-1] != '.')){
			dataBuffer[indeksBuffer++] = BIT_COMA_DISABLE; // bit null
			dataBuffer[indeksBuffer++] = dataPrint[indeks];
		}else{
			if(dataPrint[indeks] == '.'){
				dataBuffer[indeksBuffer++] = BIT_COMA_ENABLE; // bit comma
			}else if(dataPrint[indeks-1] == '.'){
				dataBuffer[indeksBuffer++] = dataPrint[indeks];
			}
		}
	}
	if(type == FOUR_DIGIT){
		while(indeksBuffer < 7){dataBuffer[indeksBuffer++] = BIT_COMA_DISABLE;}
	}else if(type == NINE_DIGIT){
		while(indeksBuffer < 17){dataBuffer[indeksBuffer++] = BIT_COMA_DISABLE;}
	}
}

uint8_t ht1622SizeOf(uint8_t * buffer){
	uint8_t indeks = 0;
	uint8_t value = 0;
	while(buffer[indeks++] != '\n'){
		value = indeks;
		if(indeks >= 100){ // timeour
			value = 0;
			break;
		}
	}
	return value;
}

void ht1622UpdateRamChar(uint8_t type, uint8_t typeDigit, uint8_t column, uint8_t * dataPrintChar, uint8_t len){
	uint8_t dataPrint[7];
	// PROCESSING DATA PRINT
	ht1622ProcessDataPrint(typeDigit, dataPrintChar, len, dataPrint);
	ht1622HandleUpdateRam(type, typeDigit, column, dataPrint);
}

void ht1622UpdateRamFloat(uint8_t type, uint8_t typeDigit, uint8_t column, float dataPrintFloat){
	uint8_t dataPrint[17];
	uint8_t dataPrintRaw[10];
	// PROCESSING DATA MINUS >> SIGNED VALUE
	if(dataPrintFloat < 0){
		if(column==1){
			lcdRam11[22][3]=1;
			lcdRam11[22][6]=1;
		}else if(column==2){
			lcdRam12[22][2]=1;
			lcdRam12[22][6]=1;
		}else if(column==3){
			lcdRam22[13][5]=1;
			lcdRam22[13][6]=1;
		}else if(column==4){
			lcdRam21[13][5]=1;
			lcdRam21[13][6]=1;
		}
		dataPrintFloat *= (-1);
	}
	// PROCESSING DATA PRINT
	sprintf(dataPrintRaw,"%.1f\n",dataPrintFloat);
	ht1622ProcessDataPrint(typeDigit, dataPrintRaw, ht1622SizeOf(dataPrintRaw), dataPrint);
	ht1622HandleUpdateRam(type, typeDigit, column, dataPrint);
}

void ht1622HandleUpdateRam(uint8_t type, uint8_t typeDigit, uint8_t column, uint8_t * dataPrint){ // dataPrint >>= count bit fic is 7 bit
	uint8_t dataBit, comBit, segBit;
	uint8_t ramAddressValue, ramAddressIndeks;
	uint8_t dataBitArray[8];

	// FILTER 7 SGMENT >>= FOUR DIGIT(CURRENT_RMS, VOLTAGE_RMS, VOLTAGE_RMS_DIV, ACTIVE_POWER, REACTIVE_POWER, APPARENT_POWER, ACTIVE_ENERGY, REACTIVE_ENERGY, POWER_FACTOR)
	if((type == NONE)||(type == CURRENT_RMS)||(type == VOLTAGE_RMS)||(type == VOLTAGE_RMS_DIV)||(type == ACTIVE_POWER)||(type == REACTIVE_POWER)||(type == REACTIVE_POWER)||(type == APPARENT_POWER)||(type == POWER_FACTOR)){
		// ================================================= SEGMENT ROW 1 ========================================================
		if(column == 1){
			uint8_t segColumnIndeks = 1;
			// ITERATE 7-SEGMENT COLUMN
			for(uint8_t indeks=0;indeks<7;indeks++){
				// FILTER FOF NUMBBER SEGMENT >>= 7-SEGMENT
				if((dataPrint[indeks] != 0xF) && (dataPrint[indeks] != 0xFF)){ // bit NULL & bit coma
					// ITERATE CONVERT DATA FROM CHAR TO BIT SEGMENT
					for(uint8_t indeks1=0;indeks1<38;indeks1++){
						if(dataPrint[indeks] == (uint8_t)alphaNumeric[indeks1][0]){
							dataBit = alphaNumeric[indeks1][1];
							integerToArray(dataBit, dataBitArray);
							break;
						}
					}
					// ITERATE GET ADDRESSING DATA BIT FOR 7-SEGMENT
					for(uint8_t segment=1;segment<8;segment++){
						// ITERATE GET DATA COM, SEG, ADDR RAM
						for(uint8_t indeks2=0;indeks2<56;indeks2++){
							if((mapSegmentIC1[indeks2][0] == segColumnIndeks) && (mapSegmentIC1[indeks2][1] == segment)){
								segBit = mapSegmentIC1[indeks2][3];
								comBit = mapSegmentIC1[indeks2][2];
								// GET DATA ADDRESS RAM IC DRIVER LCD
								for(uint8_t indeks3=0;indeks3<32;indeks3++){
									if(lcdRam11[indeks3][0] == segBit){
										ramAddressValue = lcdRam11[indeks3][1];
										ramAddressIndeks = indeks3;
										break;
									}
								}
								break;
							}
						}
						// UPDATE VALUE TO RAM IC DRIVER & ACTIVATING FLAG FOR UPDATE TO LCD DRIVER
						lcdRam11[ramAddressIndeks][comBit+2] = dataBitArray[segment-1];
						lcdRam11[ramAddressIndeks][6] = 1;
						// SWITCH TO THE MEXT COLUMN 7-SEGMENT
					}
					segColumnIndeks++;
				// FILTER FOR COMA SEGMENT >>= ENABLE OR DISABLE COMA
				}else{
					// FILTER INDEKS ARRAY FOR COMA POSITION
					if((indeks==1) || (indeks==3) || (indeks==5)){
						uint8_t pointColumnIndeks;
						// GET COLUMN INDEKS
						if(indeks==1)pointColumnIndeks=1;
						if(indeks==3)pointColumnIndeks=2;
						if(indeks==5)pointColumnIndeks=3;

						// GET COMMON  & SEGMENT
						for(uint8_t indeks4=0;indeks4<6;indeks4++){
							if(mapSegmentPointIC1[indeks4][0] == pointColumnIndeks){
								comBit = mapSegmentPointIC1[indeks4][1];
								segBit = mapSegmentPointIC1[indeks4][2];
								// GET DATA ADDRESS RAM IC DRIVER LCD
								for(uint8_t indeks1=0;indeks1<32;indeks1++){
									if(lcdRam11[indeks1][0] == segBit){
										ramAddressValue = lcdRam11[indeks1][1];
										ramAddressIndeks = indeks1;
										break;
									}
								}
								break;
							}

						}
						// UPDATE VALUE TO RAM IC DRIVER & ACTIVATING FLAG FOR UPDATE TO LCD DRIVER
						// DISABLE COMA SEGMENT
						if(dataPrint[indeks] == 0xF){ 					// BIT DISABLE COMA
							lcdRam11[ramAddressIndeks][comBit+2] = 0;	// DISABLE SEGMENT COMMA
						// EANABLE COMA SEGMENT
						}else if(dataPrint[indeks] == 0xFF){ 			// BIT ENABLE COMA
							lcdRam11[ramAddressIndeks][comBit+2] = 1;
						}
						lcdRam11[ramAddressIndeks][6] = 1;
					}
				}
			}
		// ================================================= SEGMENT ROW 2 ========================================================
		}else if(column == 2){
			uint8_t segColumnIndeks = 5;
			// ITERATE 7-SEGMENT COLUMN
			for(uint8_t indeks=0;indeks<7;indeks++){
				// FILTER FOF NUMBBER SEGMENT >>= 7-SEGMENT
				if((dataPrint[indeks] != 0xF) && (dataPrint[indeks] != 0xFF)){ // bit NULL & bit coma
					// ITERATE CONVERT DATA FROM CHAR TO BIT SEGMENT
					for(uint8_t indeks1=0;indeks1<38;indeks1++){
						if(dataPrint[indeks] == (uint8_t)alphaNumeric[indeks1][0]){
							dataBit = alphaNumeric[indeks1][1];
							integerToArray(dataBit, dataBitArray);
							break;
						}
					}
					// ITERATE GET ADDRESSING DATA BIT FOR 7-SEGMENT
					for(uint8_t segment=1;segment<8;segment++){
						// ITERATE GET DATA COM, SEG, ADDR RAM
						for(uint8_t indeks2=0;indeks2<56;indeks2++){
							if((mapSegmentIC1[indeks2][0] == segColumnIndeks) && (mapSegmentIC1[indeks2][1] == segment)){
								segBit = mapSegmentIC1[indeks2][3];
								comBit = mapSegmentIC1[indeks2][2];
								// GET DATA ADDRESS RAM IC DRIVER LCD
								for(uint8_t indeks3=0;indeks3<32;indeks3++){
									if(lcdRam12[indeks3][0] == segBit){
										ramAddressValue = lcdRam12[indeks3][1];
										ramAddressIndeks = indeks3;
										break;
									}
								}
								break;
							}
						}
						// UPDATE VALUE TO RAM IC DRIVER & ACTIVATING FLAG FOR UPDATE TO LCD DRIVER
						lcdRam12[ramAddressIndeks][comBit+2-4] = dataBitArray[segment-1];
						lcdRam12[ramAddressIndeks][6] = 1;
						// SWITCH TO THE MEXT COLUMN 7-SEGMENT
					}
					segColumnIndeks++;
				// FILTER FOR COMA SEGMENT >>= ENABLE OR DISABLE COMA
				}else{
					// FILTER INDEKS ARRAY FOR COMA POSITION
					if((indeks==1) || (indeks==3) || (indeks==5)){
						uint8_t pointColumnIndeks;
						// GET COLUMN INDEKS
						if(indeks==1)pointColumnIndeks=4;
						if(indeks==3)pointColumnIndeks=5;
						if(indeks==5)pointColumnIndeks=6;
						// GET COMMON  & SEGMENT
						for(uint8_t indeks4=0;indeks4<6;indeks4++){
							if(mapSegmentPointIC1[indeks4][0] == pointColumnIndeks){
								comBit = mapSegmentPointIC1[indeks4][1];
								segBit = mapSegmentPointIC1[indeks4][2];
								// GET DATA ADDRESS RAM IC DRIVER LCD
								for(uint8_t indeks1=0;indeks1<32;indeks1++){
									if(lcdRam12[indeks1][0] == segBit){
										ramAddressValue = lcdRam12[indeks1][1];
										ramAddressIndeks = indeks1;
										break;
									}
								}
								break;
							}
						}
						// UPDATE VALUE TO RAM IC DRIVER & ACTIVATING FLAG FOR UPDATE TO LCD DRIVER
						// DISABLE COMA SEGMENT
						if(dataPrint[indeks] == 0xF){ 					// BIT DISABLE COMA
							lcdRam12[ramAddressIndeks][comBit+2-4] = 0;	// DISABLE SEGMENT COMMA
						// EANABLE COMA SEGMENT
						}else if(dataPrint[indeks] == 0xFF){ 			// BIT ENABLE COMA
							lcdRam12[ramAddressIndeks][comBit+2-4] = 1;
						}
						lcdRam12[ramAddressIndeks][6] = 1;
					}
				}
			}
		// ================================================= SEGMENT ROW 3 ========================================================
		}else if(column == 3){
			uint8_t segColumnIndeks = 9;
			// ITERATE 7-SEGMENT COLUMN
			for(uint8_t indeks=0;indeks<7;indeks++){
				// FILTER FOF NUMBBER SEGMENT >>= 7-SEGMENT
				if((dataPrint[indeks] != 0xF) && (dataPrint[indeks] != 0xFF)){ // bit NULL & bit coma
					// ITERATE CONVERT DATA FROM CHAR TO BIT SEGMENT
					for(uint8_t indeks1=0;indeks1<38;indeks1++){
						if(dataPrint[indeks] == (uint8_t)alphaNumeric[indeks1][0]){
							dataBit = alphaNumeric[indeks1][1];
							integerToArray(dataBit, dataBitArray);
							break;
						}
					}
					// ITERATE GET ADDRESSING DATA BIT FOR 7-SEGMENT
					for(uint8_t segment=1;segment<8;segment++){
						// ITERATE GET DATA COM, SEG, ADDR RAM
						for(uint8_t indeks2=0;indeks2<56;indeks2++){
							if((mapSegmentIC2[indeks2][0] == segColumnIndeks) && (mapSegmentIC2[indeks2][1] == segment)){
								segBit = mapSegmentIC2[indeks2][3];
								comBit = mapSegmentIC2[indeks2][2];
								// GET DATA ADDRESS RAM IC DRIVER LCD
								for(uint8_t indeks3=0;indeks3<32;indeks3++){
									if(lcdRam22[indeks3][0] == segBit){
										ramAddressValue = lcdRam22[indeks3][1];
										ramAddressIndeks = indeks3;
										break;
									}
								}
								break;
							}
						}
						// UPDATE VALUE TO RAM IC DRIVER & ACTIVATING FLAG FOR UPDATE TO LCD DRIVER
						lcdRam22[ramAddressIndeks][comBit+2-4] = dataBitArray[segment-1];
						lcdRam22[ramAddressIndeks][6] = 1;
						// SWITCH TO THE MEXT COLUMN 7-SEGMENT
					}
					segColumnIndeks++;
				// FILTER FOR COMA SEGMENT >>= ENABLE OR DISABLE COMA
				}else{
					// FILTER INDEKS ARRAY FOR COMA POSITION
					if((indeks==1) || (indeks==3) || (indeks==5)){
						uint8_t pointColumnIndeks;
						// GET COLUMN INDEKS
						if(indeks==1)pointColumnIndeks=7;
						if(indeks==3)pointColumnIndeks=8;
						if(indeks==5)pointColumnIndeks=9;
						// GET COMMON  & SEGMENT
						for(uint8_t indeks4=0;indeks4<6;indeks4++){
							if(mapSegmentPointIC2[indeks4][0] == pointColumnIndeks){
								comBit = mapSegmentPointIC2[indeks4][1];
								segBit = mapSegmentPointIC2[indeks4][2];
								// GET DATA ADDRESS RAM IC DRIVER LCD
								for(uint8_t indeks1=0;indeks1<32;indeks1++){
									if(lcdRam22[indeks1][0] == segBit){
										ramAddressValue = lcdRam22[indeks1][1];
										ramAddressIndeks = indeks1;
										break;
									}
								}
								break;
							}
						}
						// UPDATE VALUE TO RAM IC DRIVER & ACTIVATING FLAG FOR UPDATE TO LCD DRIVER
						// DISABLE COMA SEGMENT
						if(dataPrint[indeks] == 0xF){ 					// BIT DISABLE COMA
							lcdRam22[ramAddressIndeks][comBit+2-4] = 0;	// DISABLE SEGMENT COMMA
						// EANABLE COMA SEGMENT
						}else if(dataPrint[indeks] == 0xFF){ 			// BIT ENABLE COMA
							lcdRam22[ramAddressIndeks][comBit+2-4] = 1;
						}
						lcdRam22[ramAddressIndeks][6] = 1;
					}
				}
			}
		// ================================================= SEGMENT ROW 4 ========================================================
		}else if(column == 4){
			uint8_t segColumnIndeks = 13;
			// ITERATE 7-SEGMENT COLUMN
			for(uint8_t indeks=0;indeks<7;indeks++){
				// FILTER FOF NUMBBER SEGMENT >>= 7-SEGMENT
				if((dataPrint[indeks] != 0xF) && (dataPrint[indeks] != 0xFF)){ // bit NULL & bit coma
					// ITERATE CONVERT DATA FROM CHAR TO BIT SEGMENT
					for(uint8_t indeks1=0;indeks1<38;indeks1++){
						if(dataPrint[indeks] == (uint8_t)alphaNumeric[indeks1][0]){
							dataBit = alphaNumeric[indeks1][1];
							integerToArray(dataBit, dataBitArray);
							break;
						}
					}
					// ITERATE GET ADDRESSING DATA BIT FOR 7-SEGMENT
					for(uint8_t segment=1;segment<8;segment++){
						// ITERATE GET DATA COM, SEG, ADDR RAM
						for(uint8_t indeks2=0;indeks2<56;indeks2++){
							if((mapSegmentIC2[indeks2][0] == segColumnIndeks) && (mapSegmentIC2[indeks2][1] == segment)){
								segBit = mapSegmentIC2[indeks2][3];
								comBit = mapSegmentIC2[indeks2][2];
								// GET DATA ADDRESS RAM IC DRIVER LCD
								for(uint8_t indeks3=0;indeks3<32;indeks3++){
									if(lcdRam21[indeks3][0] == segBit){
										ramAddressValue = lcdRam21[indeks3][1];
										ramAddressIndeks = indeks3;
										break;
									}
								}
								break;
							}
						}
						// UPDATE VALUE TO RAM IC DRIVER & ACTIVATING FLAG FOR UPDATE TO LCD DRIVER
						lcdRam21[ramAddressIndeks][comBit+2] = dataBitArray[segment-1];
						lcdRam21[ramAddressIndeks][6] = 1;
						// SWITCH TO THE MEXT COLUMN 7-SEGMENT
					}
					segColumnIndeks++;
				// FILTER FOR COMA SEGMENT >>= ENABLE OR DISABLE COMA
				}else{
					// FILTER INDEKS ARRAY FOR COMA POSITION
					if((indeks==1) || (indeks==3) || (indeks==5)){
						uint8_t pointColumnIndeks;
						// GET COLUMN INDEKS
						if(indeks==1)pointColumnIndeks=10;
						if(indeks==3)pointColumnIndeks=11;
						if(indeks==5)pointColumnIndeks=12;
						// GET COMMON  & SEGMENT
						for(uint8_t indeks4=0;indeks4<6;indeks4++){
							if(mapSegmentPointIC2[indeks4][0] == pointColumnIndeks){
								comBit = mapSegmentPointIC2[indeks4][1];
								segBit = mapSegmentPointIC2[indeks4][2];
								// GET DATA ADDRESS RAM IC DRIVER LCD
								for(uint8_t indeks1=0;indeks1<32;indeks1++){
									if(lcdRam21[indeks1][0] == segBit){
										ramAddressValue = lcdRam21[indeks1][1];
										ramAddressIndeks = indeks1;
										break;
									}
								}
								break;
							}
						}
						// UPDATE VALUE TO RAM IC DRIVER & ACTIVATING FLAG FOR UPDATE TO LCD DRIVER
						// DISABLE COMA SEGMENT
						if(dataPrint[indeks] == 0xF){ 					// BIT DISABLE COMA
							lcdRam21[ramAddressIndeks][comBit+2] = 0;	// DISABLE SEGMENT COMMA
						// EANABLE COMA SEGMENT
						}else if(dataPrint[indeks] == 0xFF){ 			// BIT ENABLE COMA
							lcdRam21[ramAddressIndeks][comBit+2] = 1;
						}
						lcdRam21[ramAddressIndeks][6] = 1;
					}
				}
			}
		}
		// ================================================= UNIT SENSOR ========================================================
		// ENABLE OR DISABLE  UNIT SENSOR SEGMENT FOR IC1
		if((type == CURRENT_RMS)||(type == VOLTAGE_RMS)||(type == ACTIVE_POWER)||(type == REACTIVE_POWER)||(type == REACTIVE_POWER)||(type == APPARENT_POWER)||(type == POWER_FACTOR)){
			lcdRam11[22][3+2] = 1; 		lcdRam11[22][2+2] = 0;		// AB phase
			lcdRam12[22][6+2-4] = 1;	lcdRam12[22][5+2-4] = 0;	// BC Phase
			lcdRam22[13][5+2-4] = 1;	lcdRam22[13][6+2-4] = 0;    // CA Phase
			// ENABLE FLAG
			lcdRam11[22][6] = 1;
			lcdRam12[22][6] = 1;
			lcdRam22[13][6] = 1;
			if(type == CURRENT_RMS){handleUnitSegment(0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1);}
			else if(type == VOLTAGE_RMS){handleUnitSegment(0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1);}
			else if(type == ACTIVE_POWER){handleUnitSegment(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1);}
			else if(type == REACTIVE_POWER){handleUnitSegment(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1);}
			else if(type == APPARENT_POWER){handleUnitSegment(0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1);}
			else if(type == POWER_FACTOR){handleUnitSegment(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1);}
		// EMABLE OR DISABLE PHASE OR DIV PHASE
		}else if(type == VOLTAGE_RMS_DIV){
			handleUnitSegment(0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1);
			lcdRam11[22][3+2] = 1; 		lcdRam11[22][2+2] = 1;		// AB phase
			lcdRam12[22][6+2-4] = 1;	lcdRam12[22][5+2-4] = 1;	// BC Phase
			lcdRam22[13][5+2-4] = 1;	lcdRam22[13][6+2-4] = 1;    // CA Phase
			// ENABLE FLAG
			lcdRam11[22][6] = 1;
			lcdRam12[22][6] = 1;
			lcdRam22[13][6] = 1;
		}
	// FILTER 7 SGMENT >>= NINE DIGIT(ACTIVE_ENERGY, REACTIVE_ENERGY)
	}else if((type == ACTIVE_ENERGY)||(type == REACTIVE_ENERGY)){
		// ENABLE OR DIABLE UNIT SEGMENT
//		if(type == ACTIVE_ENERGY){handleUnitSegment(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1);}
//		else if(type == REACTIVE_ENERGY){handleUnitSegment(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1);}
		// WRITE 7-SEGMENT >>= NINE DIGIT
		uint8_t segColumnIndeks = 17;
		// ITERATE 7-SEGMENT COLUMN
		for(uint8_t indeks7=0;indeks7<17;indeks7++){
			// FILTER FOF NUMBBER SEGMENT >>= 7-SEGMENT
			if((dataPrint[indeks7] != 0xF) && (dataPrint[indeks7] != 0xFF)){ // bit NULL & bit coma
				// ITERATE CONVERT DATA FROM CHAR TO BIT SEGMENT
				for(uint8_t indeks1=0;indeks1<38;indeks1++){
					if(dataPrint[indeks7] == (uint8_t)alphaNumeric[indeks1][0]){
						dataBit = alphaNumeric[indeks1][1];
						integerToArray(dataBit, dataBitArray);
						break;
					}
				}
				// ITERATE GET ADDRESSING DATA BIT FOR 7-SEGMENT
				for(uint8_t segment=1;segment<8;segment++){
					// ITERATE GET DATA COM, SEG, ADDR RAM
					for(uint8_t indeks2=0;indeks2<63;indeks2++){
						if((mapSegmentEnergy[indeks2][0] == segColumnIndeks) && (mapSegmentEnergy[indeks2][1] == segment)){
							segBit = mapSegmentEnergy[indeks2][3];
							comBit = mapSegmentEnergy[indeks2][2];
							// GET DATA ADDRESS RAM IC DRIVER LCD
							for(uint8_t indeks3=0;indeks3<32;indeks3++){
								if((comBit>=0) && (comBit<=3)){
									if(lcdRam21[indeks3][0] == segBit){
										ramAddressValue = lcdRam21[indeks3][1];
										ramAddressIndeks = indeks3;
										break;
									}
								}else if((comBit>=4) && (comBit<= 7)){
									if(lcdRam22[indeks3][0] == segBit){
										ramAddressValue = lcdRam22[indeks3][1];
										ramAddressIndeks = indeks3;
										break;
									}
								}
							}
							break;
						}
					}
					// UPDATE VALUE TO RAM IC DRIVER & ACTIVATING FLAG FOR UPDATE TO LCD DRIVER
					// FILTER RAM IC DRIVER
					if((comBit>=0) && (comBit<=3)){
						lcdRam21[ramAddressIndeks][comBit+2] = dataBitArray[segment-1];
						lcdRam21[ramAddressIndeks][6] = 1;
					}else if((comBit>=4) && (comBit<= 7)){
						lcdRam22[ramAddressIndeks][comBit+2-4] = dataBitArray[segment-1];
						lcdRam22[ramAddressIndeks][6] = 1;
					}
					// ENABLE FLAG
					// SWITCH TO THE MEXT COLUMN 7-SEGMENT
				}
				segColumnIndeks++;
			// FILTER FOR COMA SEGMENT >>= ENABLE OR DISABLE COMA
			}else{
				// FILTER INDEKS ARRAY FOR COMA POSITION
				if((indeks7==1)||(indeks7==3)||(indeks7==5)||(indeks7==7)||(indeks7==9)||(indeks7==11)||(indeks7==13)||(indeks7==15)){
					uint8_t pointColumnIndeks;
					// GET COLUMN INDEKS
					if(indeks7==1)pointColumnIndeks=13;
					if(indeks7==3)pointColumnIndeks=14;
					if(indeks7==5)pointColumnIndeks=15;
					if(indeks7==7)pointColumnIndeks=16;
					if(indeks7==9)pointColumnIndeks=17;
					if(indeks7==11)pointColumnIndeks=18;
					if(indeks7==13)pointColumnIndeks=19;
					if(indeks7==15)pointColumnIndeks=20;
					// GET COMMON  & SEGMENT
					for(uint8_t indeks5=0;indeks5<8;indeks5++){
						if(mapSegmentPointEnergy[indeks5][0] == pointColumnIndeks){
							comBit = mapSegmentPointEnergy[indeks5][1];
							segBit = mapSegmentPointEnergy[indeks5][2];
							// GET DATA ADDRESS RAM IC DRIVER LCD
							for(uint8_t indeks6=0;indeks6<32;indeks6++){
								if((comBit>=0) && (comBit<=3)){
									if(lcdRam21[indeks6][0] == segBit){
										ramAddressValue = lcdRam21[indeks6][1];
										ramAddressIndeks = indeks6;
										break;
									}
								}else if((comBit>=4) && (comBit<=7)){
									if(lcdRam22[indeks6][0] == segBit){
										ramAddressValue = lcdRam22[indeks6][1];
										ramAddressIndeks = indeks6;
										break;
									}
								}
							}
							break;
						}
					}
					// UPDATE VALUE TO RAM IC DRIVER & ACTIVATING FLAG FOR UPDATE TO LCD DRIVER
					// DISABLE COMA SEGMENT
					if(dataPrint[indeks7] == 0xF){ 					// BIT DISABLE COMA
						if((comBit>=0) && (comBit<=3)){lcdRam21[ramAddressIndeks][comBit+2] = 0;	lcdRam21[ramAddressIndeks][6] = 1;}	// DISABLE SEGMENT COMMA
						if((comBit>=4) && (comBit<=7)){lcdRam22[ramAddressIndeks][comBit+2-4] = 0;	lcdRam22[ramAddressIndeks][6] = 1;}	// DISABLE SEGMENT COMMA
					// EANABLE COMA SEGMENT
					}else if(dataPrint[indeks7] == 0xFF){ 			// BIT ENABLE COMA
						if((comBit>=0) && (comBit<=3)){lcdRam21[ramAddressIndeks][comBit+2] = 1;	lcdRam21[ramAddressIndeks][6] = 1;}	// DISABLE SEGMENT COMMA
						if((comBit>=4) && (comBit<=7)){lcdRam22[ramAddressIndeks][comBit+2-4] = 1;	lcdRam22[ramAddressIndeks][6] = 1;}	// DISABLE SEGMENT COMMA
					}
				}
			}
		}
	}
}

void handleUnitSegment(
			uint8_t powerActiveKw,
			uint8_t powerActiveMW,
			uint8_t current,
			uint8_t powerReactiveKw,
			uint8_t powerReactiveMw,
			uint8_t voltage,
			uint8_t powerApprarentKw,
			uint8_t powerApparentMw,
			uint8_t voltageK,
			uint8_t freq,
			uint8_t temp,
			uint8_t percent,
			uint8_t energyActive,
			uint8_t energyReactive,
			uint8_t powerFactor,
			uint8_t energyTotal
){
	// UPDATE VALUE TO RAM IC DRIVER ADDRESS
	lcdRam11[31][0+2] = powerActiveKw; 		// Active Power (KW) IC1
	lcdRam11[31][1+2] = powerActiveMW; 		// Active Power (MW )IC1
	lcdRam11[31][3+2] = current;			// current IC1
	lcdRam12[31][4+2-4] = powerReactiveKw; 	// Reactive Power (KVar) IC1
	lcdRam12[31][5+2-4] = powerReactiveMw; 	// Reactive Power (MVar) IC1
	lcdRam12[31][7+2-4] = voltage; 			// Voltage IC1
	lcdRam22[31][7+2-4] = powerApprarentKw; // Apparent Power (KVa) IC2
	lcdRam22[31][6+2-4] = powerApparentMw; 	// Apparent Power (MVa) IC2
	lcdRam22[31][5+2-4] = voltageK; 		// Volrage (Kv) IC2
	lcdRam22[31][4+2-4] = freq; 			// frequency IC2
	lcdRam22[30][4+2-4] = temp;				// disable segement temperature >>= not used
	lcdRam21[31][3+2] = percent;			// precent
	lcdRam21[31][2+2] = energyActive;		// total active energy
	lcdRam21[31][1+2] = energyReactive;		// total rective energy
	lcdRam21[10][1+2] = powerFactor;		// power factor
	lcdRam21[12][1+2] = energyTotal;		// energy total
	// ENABLE FLAG
	lcdRam11[31][6] = 1;
	lcdRam12[31][6] = 1;
	lcdRam21[31][6] = 1;
	lcdRam21[10][6] = 1;
	lcdRam21[12][6] = 1;
	lcdRam22[31][6] = 1;
	lcdRam22[30][6] = 1;
}

void ht1622EncodeGroup(){
	uint8_t indeksBuffer=0;
	// LCD RAM11
	for(uint8_t indeks=0;indeks<32;indeks++){
		if(lcdRam11[indeks][6] == 1){
			dataWriteLcd[indeksBuffer][0] = ht1622EncodeSingle(lcdRam11[indeks][1], lcdRam11[indeks]);
			dataWriteLcd[indeksBuffer][1] = 11;
			indeksBuffer++;
		}
		if(lcdRam12[indeks][6] == 1){
			dataWriteLcd[indeksBuffer][0] = ht1622EncodeSingle(lcdRam12[indeks][1], lcdRam12[indeks]);
			dataWriteLcd[indeksBuffer][1] = 12;
			indeksBuffer++;
		}
		if(lcdRam21[indeks][6] == 1){
			dataWriteLcd[indeksBuffer][0] = ht1622EncodeSingle(lcdRam21[indeks][1], lcdRam21[indeks]);
			dataWriteLcd[indeksBuffer][1] = 21;
			indeksBuffer++;
		}
		if(lcdRam22[indeks][6] == 1){
			dataWriteLcd[indeksBuffer][0] = ht1622EncodeSingle(lcdRam22[indeks][1], lcdRam22[indeks]);
			dataWriteLcd[indeksBuffer][1] = 22;
			indeksBuffer++;
		}
	}
	dataWriteLcd[indeksBuffer][0] = 65535;
	dataWriteLcd[indeksBuffer][1] = 65535;
}

uint16_t ht1622EncodeSingle(uint8_t address, uint8_t * data){
	uint16_t buffer = 0;
	uint16_t address16 = (uint8_t)address;
	uint16_t data16 = (uint8_t)data;
	buffer = 0b101 << 13;
	buffer = buffer | (address16 << 7);
	buffer = buffer | (data[2] << 6);
	buffer = buffer | (data[3] << 5);
	buffer = buffer | (data[4] << 4);
	buffer = buffer | (data[5] << 3);
	return buffer;
}

uint8_t ht1622GenerateBit(uint8_t dataRaw){
	uint8_t buffer = 0;
	for(uint8_t indeks=0; indeks<38; indeks++){
		if(dataRaw == alphaNumeric[indeks][0]){
			buffer =  alphaNumeric[indeks][1];
			break;
		}else{
			buffer = 0;
		}
	}
	return buffer;
}

void integerToArray(uint8_t data, uint8_t * buffer){
    uint8_t dataBuffer;
    for(uint8_t indeks=0;indeks<8;indeks++){
        dataBuffer = (data>>indeks) & 0b0000001;
        buffer[indeks] = dataBuffer;
    }
}

static void CS_LOW(uint8_t type){
	if(type == CS1)HAL_GPIO_WritePin(CS1_DRIVER_GPIO_Port, CS1_DRIVER_Pin, GPIO_PIN_RESET);
	else if(type == CS2)HAL_GPIO_WritePin(CS2_DRIVER_GPIO_Port, CS2_DRIVER_Pin, GPIO_PIN_RESET);
}

static void CS_HIGH(uint8_t type){
	if(type == CS1)HAL_GPIO_WritePin(CS1_DRIVER_GPIO_Port, CS1_DRIVER_Pin, GPIO_PIN_SET);
	else if(type == CS2)HAL_GPIO_WritePin(CS2_DRIVER_GPIO_Port, CS2_DRIVER_Pin, GPIO_PIN_SET);
}

static void addr_cmd_bit(uint8_t data, uint8_t cnt)
{
	uint8_t i;
	for(i = 0; i < cnt; i++) {
		WR_LOW;
		if(data & 0x80)
			DATA_HIGH;
		else
			DATA_LOW;
		WR_HIGH;
		data <<= 1;
	}
}

static void data_bit(uint8_t data, uint8_t cnt)
{
	uint8_t i;
	for(i = 0; i < cnt; i++) {
		WR_LOW;
		if(data & 0x01)
			DATA_HIGH;
		else
			DATA_LOW;
		WR_HIGH;
		data >>= 1;
	}
}

void send_command(uint8_t type, uint8_t cmd)
{
	CS_LOW(type);
	addr_cmd_bit(0x80, 3);
	addr_cmd_bit(cmd, 9);
	CS_HIGH(type);
	__NOP();
}

/* seg_addr: A5~A0 (0011 1111) 0x3F
   com_data: D3~D0 (0000 1111) 0x0F */
void write_seg_data_4(uint8_t type, uint8_t seg_addr, uint8_t com_data)
{
	seg_addr <<= 2;
	CS_LOW(type);
	addr_cmd_bit(0xA0, 3);
	addr_cmd_bit(seg_addr, 6);
	data_bit(com_data, 4);
	CS_HIGH(type);
	__NOP();
}

void write_seg_data_44(uint8_t type, uint8_t seg_addr, uint8_t *com_data, uint16_t count)
{
	uint16_t i;
	seg_addr <<= 2;
	CS_LOW(type);
	addr_cmd_bit(0xA0, 3);
	addr_cmd_bit(seg_addr, 6);
	for(i = 0; i < count; i++, com_data++) {
		data_bit(*com_data, 8);
	}
	CS_HIGH(type);
	__NOP();
}

void write_seg_data_bit_4(uint8_t type,uint8_t seg_addr, uint8_t d3, uint8_t d2, uint8_t d1, uint8_t d0)
{
	write_seg_data_4(type, seg_addr, d3<<3 | d2<<2 | d1<<1 | d0<<0);
}

void set_all(void)
{
	uint16_t i;
	for(i = 0; i < 0x3F; i++) {  //A5~A0: 00111111
		write_seg_data_4(CS1, i, 0x0F); //D3~D0: 00001111 set 1
		write_seg_data_4(CS2, i, 0x0F); //D3~D0: 00001111 set 1
	}
}

void clean_all()
{
	uint16_t i;
	for(i = 0; i < 0x3F; i++) {  //A5~A0: 00111111
		write_seg_data_4(CS1, i, 0x00); //D3~D0: 00001111 set 0
		write_seg_data_4(CS2, i, 0x00); //D3~D0: 00001111 set 0
	}
}

void ht1622_init(void)
{
	// IC1
	send_command(CS1, BIAS);
//	send_command(CS1, RC32);
	send_command(CS1, RC1);
	send_command(CS1, SYSEN);
	send_command(CS1, LCDON);
	// IC2
	send_command(CS2, BIAS);
//	send_command(CS2, RC32);
	send_command(CS2, RC1);
	send_command(CS2, SYSEN);
	send_command(CS2, LCDON);
}

void ht1622ClearSegment(){
	// LCD RAM COMBINE (LCDRAM11,LCDRAM12,LCDRAM21,LCDRAM22)
	for(uint8_t seg=23;seg<=31;seg++){
		for(uint8_t com=2;com<=5;com++){
			lcdRam11[seg][com] = 0;
			lcdRam11[seg][6] = 1;
			lcdRam12[seg][com] = 0;
			lcdRam12[seg][6] = 1;
			lcdRam21[seg][com] = 0;
			lcdRam21[seg][6] = 1;
			lcdRam22[seg][com] = 0;
			lcdRam22[seg][6] = 1;
		}
	}
	// LCDrRAM 11
	for(uint8_t com=2;com<=5;com++){
		lcdRam11[22][com] = 0;
		lcdRam11[22][6] = 1;
	}
	// LCD RAM 12
	for(uint8_t com=2;com<=5;com++){
		lcdRam12[22][com] = 0;
		lcdRam12[22][6] = 1;
	}
	// LCD RAM COMBINE (LCDRAM21,LCDRAM22)
	for(uint8_t seg=1;seg<=13;seg++){
		for(uint8_t com=2;com<=5;com++){
			lcdRam21[seg][com] = 0;
			lcdRam21[seg][6] = 1;
			lcdRam22[seg][com] = 0;
			lcdRam22[seg][6] = 1;
		}
	}
}

void ht1622Print(void){
	for(uint8_t indeks=0;indeks<32;indeks++){
		if(lcdRam11[indeks][6] == 1){write_seg_data_bit_4(1, lcdRam11[indeks][1], lcdRam11[indeks][5], lcdRam11[indeks][4], lcdRam11[indeks][3], lcdRam11[indeks][2]);}
		if(lcdRam12[indeks][6] == 1){write_seg_data_bit_4(1, lcdRam12[indeks][1], lcdRam12[indeks][5], lcdRam12[indeks][4], lcdRam12[indeks][3], lcdRam12[indeks][2]);}
		if(lcdRam21[indeks][6] == 1){write_seg_data_bit_4(2, lcdRam21[indeks][1], lcdRam21[indeks][5], lcdRam21[indeks][4], lcdRam21[indeks][3], lcdRam21[indeks][2]);}
		if(lcdRam22[indeks][6] == 1){write_seg_data_bit_4(2, lcdRam22[indeks][1], lcdRam22[indeks][5], lcdRam22[indeks][4], lcdRam22[indeks][3], lcdRam22[indeks][2]);}
	}
}




// ==============================================================================================================================
//for(uint8_t indeks=1;indeks<7;indeks++){
//		if(mapSegmentUnitIC1[indeks][0] == type){
//			segBit = mapSegmentUnitIC1[indeks][2];
//			comBit = mapSegmentUnitIC1[indeks][1];
//			break;
//		}
//	}
//	// GET DATA ADDRESS RAM IC DRIVER LCD
//	for(uint8_t indeks=0;indeks<32;indeks++){
//		// RAM ADDRESS FOR COM 0,1,2,3
//		if(comBit>=0 && comBit<=3){
//			for(uint8_t indeks1=0;indeks1<32;indeks1++){
//				if(lcdRam11[indeks1][0] == segBit){
//					ramAddressValue = lcdRam11[indeks1][1];
//					ramAddressIndeks = indeks1;
//					break;
//				}
//			}
//			break;
//		}
//		// RAM ADDRESS FOR COM 4,5,6,7
//		if(comBit>=4 && comBit<=6){
//			for(uint8_t indeks1=0;indeks1<32;indeks1++){
//				if(lcdRam12[indeks1][0] == segBit){
//					ramAddressValue = lcdRam12[indeks1][1];
//					ramAddressIndeks = indeks1;
//					break;
//				}
//			}
//			break;
//		}
//	}
//	lcdRam11[ramAddressIndeks][comBit+2] = 1;
//
