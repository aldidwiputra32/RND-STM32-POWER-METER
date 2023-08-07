/*
 * library modbus slave v0.1
 * by Aldi Dwi Putra
 */

#include "modbusSlave.h"
#include "usart.h"
#include "main.h"
#define MODBUS_HANDLE_RESPONS 0

MODBUS Modbus;
extern triggerTX;
uint16_t addressModbus;

void ModbusBegin(MODBUS *modbus, UART_HandleTypeDef * huart, int trigState, uint8_t slaveAddrSlave, uint8_t slaveAddrSlaveSecond, uint16_t * holdingRegisterAddress, uint16_t * holdingRegisterValue, uint16_t * holdingRegisterSize, GPIO_TypeDef * gpioPort, uint16_t gpioPin){
	modbus->trigState = trigState;
	modbus->huart = huart;
	modbus->huartInstance = huart->Instance;
	modbus->slaveAddrSlave = slaveAddrSlave;
	modbus->slaveAddrSlaveSecond = slaveAddrSlaveSecond;
	modbus->holdingRegisterAddress = holdingRegisterAddress;
	modbus->holdingRegisterValue = holdingRegisterValue;
	modbus->holdingRegisterSize = holdingRegisterSize;
	modbus->enableGpioPort = gpioPort;
	modbus->enableGpioPin = gpioPin;
}

void modbusReceive(MODBUS * Modbus){
	HAL_GPIO_WritePin(Modbus->enableGpioPort,Modbus->enableGpioPin,GPIO_PIN_RESET);
	HAL_UARTEx_ReceiveToIdle_DMA(Modbus->huart,Modbus->dataRX,SIZE_DATA);
}

void modbusTransmit(MODBUS * Modbus){
	HAL_GPIO_WritePin(Modbus->enableGpioPort,Modbus->enableGpioPin,GPIO_PIN_SET);
	HAL_UART_Transmit_DMA(Modbus->huart,Modbus->dataTX,Modbus->startAddr);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart , uint16_t Size){
	Modbus.sizeRX = Size;
	if(huart->Instance == Modbus.huartInstance){
		// DECODE DATA FROM SERIAL MASTER
		uint8_t state = modbusGetParam(&Modbus);
		if(state == MODBUS_OK){
			modbusEncode(&Modbus);
			modbusTransmit(&Modbus);
		}else{
			if(MODBUS_HANDLE_RESPONS){
				modbusException(&Modbus,state);
				modbusTransmit(&Modbus);
			}else{
				modbusReceive(&Modbus);
			}
		}
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart){
	if(huart->Instance == Modbus.huartInstance){
		modbusReceive(&Modbus);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == Modbus.huartInstance){
		modbusReceive(&Modbus);
	}
}

// Generate the MODBUS RTU CRC
uint16_t modbusCreateCRC(uint8_t * buf, int len){
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];				// XOR byte into least sig. byte of crc
    for (int i = 8; i != 0; i--) {			// Loop over each bit
      if ((crc & 0x0001) != 0) {			// If the LSB is set
        crc >>= 1;							// Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else crc >>= 1;						// Just shift right
    }
  }
  return crc;
}

void modbusEncodeAssemble(uint8_t * data, uint8_t * indeksTarget, uint8_t * value, int size){
	for(int indeks = 0; indeks<size;indeks++){
		data[indeksTarget[indeks]] = value[indeks];
	}
}

void ModbusEncodeAddCRC(MODBUS * Modbus, uint8_t * data, uint8_t size){
	Modbus->crc = modbusCreateCRC(data, size);
	data[size] = byteLow(Modbus->crc);
	data[size+1] = byteHigh(Modbus->crc);
}

void modbusEncode(MODBUS * Modbus){
	uint16_t address;
	uint16_t startAddressBuffer = Modbus->startAddr;
	Modbus->dataTX[0] = Modbus->slaveAddrMaster;
	Modbus->dataTX[1] = Modbus->functionCode;
	int offsetByte;
	if(Modbus->functionCode==0x03){
		Modbus->dataTX[2] = Modbus->numReg+Modbus->numReg;
		offsetByte = 3;
		for(int indeks=0;indeks<Modbus->numReg;indeks++){
			address = modbusGetIndeks(Modbus->holdingRegisterAddress, Modbus->startAddr, Modbus->holdingRegisterSize);
			Modbus->dataTX[offsetByte++] = byteHigh(Modbus->holdingRegisterValue[address]);
			Modbus->dataTX[offsetByte++] = byteLow(Modbus->holdingRegisterValue[address]);
			Modbus->startAddr++;
		}
		// CHANGE STARTADDR VARIABLE TO SIZE BYTE FOR UART TRANSFER ARGUMENT
		addressModbus = Modbus->startAddr;
		Modbus->startAddr = offsetByte;
		Modbus->crc = modbusCreateCRC(Modbus->dataTX,offsetByte);
		Modbus->dataTX[Modbus->startAddr++] = byteLow(Modbus->crc);
		Modbus->dataTX[Modbus->startAddr++] = byteHigh(Modbus->crc);
		Modbus->crc = 0;
	}
	if(Modbus->functionCode == 0x06){
		address = modbusGetIndeks(Modbus->holdingRegisterAddress, startAddressBuffer, Modbus->holdingRegisterSize);
		Modbus->holdingRegisterValue[address] = bytePack(Modbus->dataRX[4],Modbus->dataRX[5]);
		uint8_t indeksTarget[] = {2,3,4,5};
		uint8_t value[] = {
				Modbus->dataRX[2],
				Modbus->dataRX[3],
				byteHigh(Modbus->holdingRegisterValue[address]),
				byteLow(Modbus->holdingRegisterValue[address]),
		};
		modbusEncodeAssemble(Modbus->dataTX, indeksTarget, value, 4);
		ModbusEncodeAddCRC(Modbus, Modbus->dataTX, 6);

		// CHANGE STARTADDR VARIABLE TO SIZE BYTE FOR UART TRANSFER ARGUMENT
		addressModbus = Modbus->startAddr;
		Modbus->startAddr = 8;
		Modbus->trigState = 1;
		Modbus->crc = 0;
	}
	if(Modbus->functionCode == 0x10){
		offsetByte = 7;
		for(int indeks=0;indeks<Modbus->numReg;indeks++){
			address = modbusGetIndeks(Modbus->holdingRegisterAddress, startAddressBuffer+indeks, Modbus->holdingRegisterSize);
			Modbus->holdingRegisterValue[address] = bytePack(Modbus->dataRX[offsetByte++],Modbus->dataRX[offsetByte++]);
		}
		uint8_t indeksTarget[] = {2,3,4,5};
		uint8_t value[] = {
				Modbus->dataRX[2],
				Modbus->dataRX[3],
				byteHigh(Modbus->numReg),
				byteLow(Modbus->numReg)
		};
		modbusEncodeAssemble(Modbus->dataTX, indeksTarget, value, 4);
		ModbusEncodeAddCRC(Modbus, Modbus->dataTX, 6);
		// CHANGE STARTADDR VARIABLE TO SIZE BYTE FOR UART TRANSFER ARGUMENT
		addressModbus = Modbus->startAddr;
		Modbus->startAddr = 8;
		Modbus->trigState = 1;
		Modbus->crc = 0;
	}
}

void modbusDecode(MODBUS * Modbus){
	// HANDLE READ AND WRITE SINGLE HOLDING REGISTER
	if((Modbus->functionCode==0x06) || (Modbus->functionCode==0x03)){
		Modbus->crc =  modbusCreateCRC(Modbus->dataRX, 6);
		Modbus->startAddr = bytePack(Modbus->dataRX[2],Modbus->dataRX[3]);
		Modbus->crcMaster  = bytePack(Modbus->dataRX[7], Modbus->dataRX[6]);
		// FUNCTION CODE READ MULTIPLE HOLDING REGISTER
		if(Modbus->functionCode == 0x03){
			Modbus->numReg = bytePack(Modbus->dataRX[4],Modbus->dataRX[5]);
			Modbus->endAddr = Modbus->startAddr + Modbus->numReg - 1;
		}
	}
	// FUNCTION CODE WIRTE MULTPLEHOLDING REGISTER AND ACTION
	if(Modbus->functionCode==0x10){
		Modbus->crc = modbusCreateCRC(Modbus->dataRX,Modbus->sizeRX-2);
		Modbus->startAddr = bytePack(Modbus->dataRX[2],Modbus->dataRX[3]);
		Modbus->crcMaster = bytePack(Modbus->dataRX[Modbus->sizeRX-1],Modbus->dataRX[Modbus->sizeRX-2]);
		Modbus->numReg = bytePack(Modbus->dataRX[4],Modbus->dataRX[5]);
		Modbus->endAddr = Modbus->startAddr+ Modbus->numReg - 1;
	}
}

uint8_t byteLow(uint16_t buf){return (uint8_t)((buf & 0x00FF));}
uint8_t byteHigh(uint16_t buf){return (uint8_t)((buf & 0xFF00) >> 8);}
uint16_t bytePack(uint8_t dataHigh, uint8_t dataLow){return ((dataHigh<<8)|dataLow);}

void modbusException(MODBUS * Modbus, uint8_t exceptionCode){
	uint8_t indeksTarget[] = {0,1,2};
	uint8_t value[] = {
			Modbus->slaveAddrMaster,
			Modbus->functionCode,
			exceptionCode
	};
	modbusEncodeAssemble(Modbus->dataTX, indeksTarget, value, 3);
	ModbusEncodeAddCRC(Modbus, Modbus->dataTX, 3);
	Modbus->startAddr = 5;
}

uint8_t modbusCheck(MODBUS * Modbus){
	uint8_t state = 1;
	modbusDecode(Modbus);
	// HANDLE READ MULTIPLE & WRITE SINGLE HOLDING REGISTER FC=0x03 & 0x06
	if((Modbus->functionCode==0x03) || (Modbus->functionCode==0x06) || (Modbus->functionCode==0x10)){
		if((Modbus->functionCode==0x03) || (Modbus->functionCode==0x10)){
			uint16_t addressConfirm = modbusGetIndeks(Modbus->holdingRegisterAddress, Modbus->startAddr, Modbus->holdingRegisterSize);
			uint16_t dataOld = Modbus->holdingRegisterAddress[addressConfirm];
			for(uint16_t indeks=0;indeks<Modbus->numReg;indeks++){
				uint8_t stateCheck = Modbus->holdingRegisterAddress[addressConfirm+indeks] - dataOld == 1;
				stateCheck = stateCheck || (Modbus->holdingRegisterAddress[addressConfirm+indeks] - dataOld == 0);
				if(stateCheck && addressConfirm != 0xFF){
					int addr = addressConfirm+indeks;
					if(addr<addressConfirm) addr = addressConfirm;
					dataOld = Modbus->holdingRegisterAddress[addr];
					state = MODBUS_OK;
				}else{
					state = ILLEGAL_DATA_VALUE;
					break;
				}
			}
		}
		if(state == ILLEGAL_DATA_VALUE){
			state = ILLEGAL_DATA_VALUE;
		}
		else if(Modbus->endAddr>0xFFFF){
			state = ILLEGAL_DATA_ADDRESS;
		} else if(Modbus->crcMaster != Modbus->crc){
			state = ILLEGAL_DATA_CRC;
		}else{
			state = MODBUS_OK;
		}
	// HANDLE WRITE MULTIPLE HOLDING REGISTER FC=0x10=16
	}
	return state;
}

uint8_t modbusGetParam(MODBUS * Modbus){
	// SEPARATE & DECODE DATA FROM SERIAL MASTER
	uint8_t state = 0;
	Modbus->slaveAddrMaster = Modbus->dataRX[0];
	Modbus->functionCode = Modbus->dataRX[1];
	if((Modbus->slaveAddrMaster == Modbus->slaveAddrSlave) || (Modbus->slaveAddrMaster == Modbus->slaveAddrSlaveSecond)){
		state = modbusCheck(Modbus);
	}else{
		state = ILLEGAL_SLAVE_ID;
	}
	return state;
}

uint16_t modbusGetIndeks(uint16_t *arr, uint16_t data, uint16_t size){
	uint8_t value = -1;
	for(int indeks=0;indeks<size; indeks++){
		if(arr[indeks] == data){
			value = indeks;
			break;
		}
	}
	return value;
}
