#ifndef HT1622_DEF
#define HT1622_DEF

#include "main.h"
#include "spi.h"

#define BIT_COMA_ENABLE		0xFF;
#define BIT_COMA_DISABLE	0xF;

/*
 *   - Power Factor
  - Active Energy >> KWh
 */
// TYPE SEGMENT
#define FOUR_DIGIT 4
#define NINE_DIGIT 9

// IC1: current, voltage, voltage div ,active power, reactive power
#define CURRENT_RMS			3	// A
#define VOLTAGE_RMS 		6 	// V
#define VOLTAGE_RMS_DIV		66 	// V
#define ACTIVE_POWER		1 	// KW
#define REACTIVE_POWER		4 	// VAR

// IC2: apparent power, active energy, reactive energy, power factor
#define APPARENT_POWER		7 	// VA
#define ACTIVE_ENERGY		13 	// KWH
#define REACTIVE_ENERGY		14 	// KVARH
#define POWER_FACTOR		17  // none
#define CS1					1
#define CS2					2

uint8_t ht1622GenerateBit(uint8_t dataRaw);
void ht1622ProcessDataPrint(uint8_t type ,uint8_t * dataPrint, uint8_t len, uint8_t * dataBuffer);
uint8_t ht1622SizeOf(uint8_t * buffer);
void ht1622UpdateRam(uint8_t type, uint8_t typeDigit, uint8_t column, float dataPrintFloat);
void integerToArray(uint8_t data, uint8_t * buffer);
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
);
void ht1622EncodeGroup();
uint8_t ht1622GenerateBit(uint8_t dataRaw);
uint16_t ht1622EncodeSingle(uint8_t address, uint8_t * data);

#define  BIAS          	0x29
#define  RC256         	0x18
#define  RC128         	0xA7
#define  RC64         	0xA6
#define  RC32        	0xA5
#define  RC16        	0xA4
#define  RC8         	0xA3
#define  RC4         	0xA2
#define  RC2         	0xA1
#define  RC1         	0xA0
#define  SYSEN         	0x01
#define  LCDON         	0x03
#define  NORMAL_MODE	0xE3
#define	 NONE			0

void send_command(uint8_t type, uint8_t cmd);
void write_seg_data_4(uint8_t type, uint8_t seg_addr, uint8_t com_data);
void write_seg_data_44(uint8_t type, uint8_t seg_addr, uint8_t *com_data, uint16_t count);
void write_seg_data_bit_4(uint8_t type,uint8_t seg_addr, uint8_t d3, uint8_t d2, uint8_t d1, uint8_t d0);
void set_all(void);
void clean_all(void);
void ht1622_init(void);
void ht1622Print(void);

#endif
