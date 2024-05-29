#include "memory.h"

uint8_t memoryTrigReset = 0;
uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
uint32_t Address = 0, PageError = 0, flashAddrVirtual = 0;

extern uint64_t energyActiveA_uint;
extern uint64_t energyActiveB_uint;
extern uint64_t energyActiveC_uint;
extern uint64_t energyReactiveA_uint;
extern uint64_t energyReactiveB_uint;
extern uint64_t energyReactiveC_uint;
extern uint16_t gainCurrentButton_stm32;
extern MODBUS Modbus;

static FLASH_EraseInitTypeDef EraseInitStruct;

uint32_t GetPage(uint32_t Addr){
	return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}

void memorySetup(uint32_t start, uint32_t end){
	/* Clear OPTVERR bit set on virgin samples */
	// __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	// FirstPage = GetPage(start);
	/* Get the number of pages to erase from 1st page */
	NbOfPages = ((end - start)+1) / FLASH_PAGE_SIZE;
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = start;
	EraseInitStruct.NbPages     = NbOfPages;
}

void memoryMultipleRead64(uint32_t startAddr, int number, uint64_t * data){
	int page = 0;
	for(int indeks=0;indeks<number;indeks++){
		data[page++] = *(uint32_t *)startAddr;
		startAddr = startAddr + 4;
		data[page++] = *(uint32_t *)startAddr;
		startAddr = startAddr + 4;
		data[indeks] = bytePack64(data[page-1],data[page-2]);
	}
	for (int indeks=number;indeks<number*2;indeks++){
		data[indeks] = 0;
	}
}

void memoryMultipleRead32(uint32_t startAddr, int number, uint32_t * data){
	for(int index=0;index<number;index++){
		memorySingleRead(startAddr, &data[index]);
		startAddr += 4;
	}
}

void memorySingleRead(uint32_t startAddr, uint32_t * buffer){
	int page = 0;
	uint32_t data[2];
	data[page++] = *(uint16_t*)startAddr;
	startAddr = startAddr + 2;
	data[page++] = *(uint16_t*)startAddr;
	startAddr = startAddr + 2;
	*buffer = bytePack32(data[page-1],data[page-2]);
}

void memoryVirtualRead(uint32_t startAddr, uint32_t endAddr, uint32_t startAddrVirtual, uint32_t * data, uint8_t number){
	uint32_t startAddrScan = startAddr;
	uint32_t tresholdAddr = 0;
	while(1){
		// GET DATA FROM FLASH MEMORY
		memorySingleRead(startAddrScan, &tresholdAddr);
		// SCAN DATA FROM FLASH ADDRESS
 		if(((tresholdAddr) == 0xffffffff) || (startAddrScan > (endAddr-3))){
			if(startAddrScan > (endAddr-3)){
				startAddrScan = startAddrScan + 4;
			}
			// GET LAST ADDRESS WRITED
			flashAddrVirtual = startAddrScan - 4*number + 4*startAddrVirtual;
			if(flashAddrVirtual < startAddr){
				*data = NULL_DATA;
				flashAddrVirtual = startAddr;
			}else{
				// GET DATA FROM FLASH MEMORY
				memorySingleRead(flashAddrVirtual, data); // *data = *(uint32_t *)startAddr;
			}
			break;
		}else{
			startAddrScan += 4;
		}
	}
}

void memoryReset(uint32_t start ,uint32_t end){
	memorySetup(start, end);
	HAL_FLASH_Unlock();
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
		HAL_FLASH_GetError();
	}
	HAL_FLASH_Lock();
}

uint8_t memoryMultipleWrite(uint32_t startAddr, uint32_t endAddr, int number, uint32_t * data){
	memorySetup(startAddr, endAddr);
	uint8_t state = MEMORY_OK;
	HAL_FLASH_Unlock();
	for(int indeks=0;indeks<number;indeks++){
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, startAddr, data[indeks]) == HAL_OK){
			startAddr = startAddr + 4;
		}else{
			state = MEMORY_ERROR;
			break;
		}
	}
	HAL_FLASH_Lock();
	return state;
}

void memoryVirtualWrite(uint32_t data, uint32_t startAddr, uint32_t endAddr){
	uint32_t value = 0;
	memoryTrigReset = 0; // status trigger reset
	while(1){
		memorySingleRead(flashAddrVirtual, &value);
		if((value) == 0xffffffff){
			HAL_StatusTypeDef status;
			HAL_FLASH_Unlock();
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddrVirtual, data);
			HAL_FLASH_Lock();
			if(status == HAL_OK){
				break;
			}
		}else{
			flashAddrVirtual += 4;
			if(flashAddrVirtual > endAddr - 3 ){ // size per address 8 byte (DoubleWord)
				memoryTrigReset = 1; // status trigger reset
				memoryReset(startAddr,endAddr);
				flashAddrVirtual = startAddr;
			}
		}
	}
}

uint64_t bytePack64(uint32_t dataHigh, uint32_t dataLow){
	return ((uint64_t)dataHigh<<32|(uint64_t)dataLow);
}

uint32_t bytePack32(uint16_t dataHigh, uint16_t dataLow){
	return ((uint32_t)dataHigh<<16|(uint32_t)dataLow);
}

// FINGERPRINT WRITE MEMORY
/* Parameter:
 * 	uint32_t * data >> data array 3 indeks for write
 */
void pmMemoryWrite(uint32_t * data){
	uint8_t bufferMemoryTrigReset;
	do{
		bufferMemoryTrigReset = 0;
		for(uint8_t index=0;index<5;index++){
			memoryVirtualWrite(data[index], FLASH_USER_START_ADDR, FLASH_USER_END_ADDR);bufferMemoryTrigReset |= memoryTrigReset;
			if(bufferMemoryTrigReset){
				break;
			}
		}
	}while(bufferMemoryTrigReset);
}

// FINGERPRINT READ MEMORY
/* Parameter:
 * 	uint32_t * data >> data array indeks for read
 */
void pmMemoryRead(uint32_t * data){
	for(uint8_t index=0;index<5;index++){
		memoryVirtualRead(FLASH_USER_START_ADDR, FLASH_USER_END_ADDR, index, &data[index], 5);
	}
}

void pmEncode(uint32_t * data){
	/* LIST OF CONTENT
	 * 1.energy active total >> 64 bit
	 * 2.energy reactive total >> 64 bit
	 * 3.gain current button >> 16 bit
	 * 4.slave address >> 16 bit
	 */
	uint64_t bufferEnergy = 0;
	bufferEnergy = energyActiveA_uint + energyActiveB_uint + energyActiveC_uint;
	data[0] = (uint32_t)(bufferEnergy);
	data[1] = (uint32_t)(bufferEnergy>>32);
	bufferEnergy = energyReactiveA_uint + energyReactiveB_uint + energyReactiveC_uint;
	data[2] = (uint32_t)(bufferEnergy);
	data[3] = (uint32_t)(bufferEnergy>>32);
	data[4] = (((uint32_t)gainCurrentButton_stm32)<<16) | ((uint32_t)Modbus.slaveAddrSlaveSecond);
}

void pmDecode(uint32_t * data){
	/* LIST OF CONTENT
	 * 1.energy active total >> 64 bit
	 * 2.energy reactive total >> 64 bit
	 * 3.gain current button >> 16 bit
	 * 4.slave address >> 16 bit
	 */
	uint64_t bufferEnergy = 0;
	// ENERGY ACTIVE
	bufferEnergy = ((uint64_t)data[0]) | (((uint64_t)data[1])<<32);
	if(bufferEnergy == 0xffffffffffffffff){
		energyActiveA_uint = energyActiveB_uint = energyActiveC_uint = 0; // DEFAULT VALUE
	}else{
		energyActiveA_uint = energyActiveB_uint = energyActiveC_uint = bufferEnergy/3;
	}
	// ENERGY REACTIVE
	bufferEnergy = ((uint64_t)data[2]) | (((uint64_t)data[3])<<32);
	if(bufferEnergy == 0xffffffffffffffff){
		energyActiveA_uint = energyActiveB_uint = energyActiveC_uint = 0; // DEFAULT VALUE
	}else{
		energyReactiveA_uint = energyReactiveB_uint = energyReactiveC_uint = bufferEnergy/3;
	}
	if(data[4] == 0xffffffff){
		// GAIN CURRENT BUTTON
		gainCurrentButton_stm32 = GAIN_BUTTON_DEF;
		// MODBUS ID
		Modbus.slaveAddrSlaveSecond = SLAVEID_DEF;
	}else{
		// GAIN CURRENT BUTTON
		gainCurrentButton_stm32 = (uint16_t)(data[4]>>16);
		// MODBUS ID
		Modbus.slaveAddrSlaveSecond = (uint8_t)data[4];
	}
}
