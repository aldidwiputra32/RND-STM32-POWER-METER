#ifndef __MODBUS_SLAVE
#define __MODBUS_SLAVE
#include"main.h"
#include "usart.h"

#define MODBUS_OK 				200
#define ILLEGAL_FUNCTION 		0x01
#define ILLEGAL_DATA_ADDRESS 	0x02
#define ILLEGAL_DATA_VALUE 		0x03
#define ILLEGAL_DATA_CRC 		0x04
#define ILLEGAL_SLAVE_ID 		0x05
#define ACTION_READ 			0
#define ACTION_WRITE 			1
#define SIZE_DATA 				((uint16_t)256)

#define SLAVEID_DEF 			1

typedef struct{
	UART_HandleTypeDef * huart;
	USART_TypeDef * huartInstance;
	uint8_t dataTX[256];
	uint8_t dataRX[256];
	uint8_t sizeRX;
	uint16_t startAddr;
	uint16_t numReg;
	uint8_t slaveAddrMaster;
	uint8_t slaveAddrSlave;
	uint8_t slaveAddrSlaveSecond;
	uint8_t functionCode;
	uint16_t endAddr;
	uint16_t crc;
	uint16_t crcMaster;
	int trigState;
	uint16_t * holdingRegisterAddress;
	uint16_t * holdingRegisterValue;
	uint16_t holdingRegisterSize;
	GPIO_TypeDef * enableGpioPort;
	uint16_t enableGpioPin;
	uint16_t * holdingRegisterValueRX;
}MODBUS;

//CALLBACK FUNCTION UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef*);

//VARIABLE GENERATE CRC
uint16_t modbusCreateCRC(uint8_t buf[], int len);
uint8_t byteHigh(uint16_t buf);
uint8_t byteLow(uint16_t buf);

uint16_t modbusGetIndeks(uint16_t * arr, uint16_t data, uint16_t size);
void ModbusEncodeAddCRC(MODBUS * Modbus, uint8_t * data, uint8_t size);
void modbusEncodeAssemble(uint8_t * data, uint8_t * indeksTarget, uint8_t * value, int size);
void ModbusBegin(MODBUS *modbus, UART_HandleTypeDef * huart, int trigState, uint8_t slaveAddrSlave, uint8_t slaveAddrSlaveSecond, uint16_t * holdingRegisterAddress, uint16_t * holdingRegisterValue, uint16_t * holdingRegisterSize, GPIO_TypeDef * gpioPort, uint16_t gpioPin);
void modbusReceive(MODBUS * Modbus);
void modbusTransmit(MODBUS * Modbus);
int modbusHandlSetSlaveAddr(MODBUS * Modbus);
void modbusDecode(MODBUS * Modbus);
uint8_t modbusCheck(MODBUS * Mobbus);
void modbusException(MODBUS * Modbus, uint8_t exceptionCode);
uint8_t modbusGetParam(MODBUS * Modbus);
void modbusEncode(MODBUS * Modbus);
uint16_t bytePack(uint8_t dataHigh, uint8_t dataLow);
void testing(MODBUS * Modbus);
#endif
