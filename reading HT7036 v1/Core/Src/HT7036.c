#include "HT7036.h"

float HFconstVal;

void spiDisable(){HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);}
void spiEnable(){HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);}

HAL_StatusTypeDef spiWrite16(uint8_t address, uint16_t dataSet){
	HAL_StatusTypeDef status;

	uint8_t dataTX[2];
	uint8_t request = address | 0x80;
	dataTX[0] = (uint8_t)(dataSet >> 8);
	dataTX[1] = (uint8_t)(dataSet);

	spiEnable();
	status = HAL_SPI_Transmit(&hspi2, &request, 1, 75);
	if(status == HAL_OK) HAL_SPI_Transmit(&hspi2,&dataTX[0],1,75);
	if(status == HAL_OK) HAL_SPI_Transmit(&hspi2,&dataTX[1],1,75);
	spiDisable();

	return status;
}

HAL_StatusTypeDef spiWrite24(uint8_t address, uint32_t dataSet){
	HAL_StatusTypeDef status;
	uint8_t dataTX[3];
	dataTX[0] = (uint8_t)(dataSet >> 16);
	dataTX[1] = (uint8_t)(dataSet >> 8);
	dataTX[2] = (uint8_t)(dataSet);
	uint8_t request = address | 0x80;
	spiEnable();
	status = HAL_SPI_Transmit(&hspi2, &request, 1, 75);
	if(status == HAL_OK)status = HAL_SPI_Transmit(&hspi2, &dataTX[0], 1, 75);
	if(status == HAL_OK)status = HAL_SPI_Transmit(&hspi2, &dataTX[1], 1, 75);
	if(status == HAL_OK)status = HAL_SPI_Transmit(&hspi2, &dataTX[2], 1, 75);
	spiDisable();
	return status;
}

HAL_StatusTypeDef spiCommandSpecial(uint8_t address, uint32_t dataSet){
	HAL_StatusTypeDef status;
	uint8_t dataTX[3];
	dataTX[0] = (uint8_t)(dataSet >> 16) & 0xFF;
	dataTX[1] = (uint8_t)(dataSet >> 8) & 0xFF;
	dataTX[2] = (uint8_t)(dataSet) & 0xFF;
	uint8_t request = address | 0xC0;
	spiEnable();
	status = HAL_SPI_Transmit(&hspi2, &request, 1, 75);
	if(status == HAL_OK)status = HAL_SPI_Transmit(&hspi2, &dataTX[0], 1, 75);
	if(status == HAL_OK)status = HAL_SPI_Transmit(&hspi2, &dataTX[1], 1, 75);
	if(status == HAL_OK)status = HAL_SPI_Transmit(&hspi2, &dataTX[2], 1, 75);
	spiDisable();
	return status;
}

uint32_t spiReadCalib(uint8_t address){
	HAL_StatusTypeDef buffer1, buffer2;
	uint8_t request = address | 0x00;
	uint8_t dataRX[3];
	uint32_t dataRXbuffer[3];

	spiEnable();
	HAL_SPI_Transmit(&hspi2, &request, 1, 75);
	HAL_SPI_Receive(&hspi2, &dataRX[0], 1, 75);
	HAL_SPI_Receive(&hspi2, &dataRX[1], 1, 75);
	HAL_SPI_Receive(&hspi2, &dataRX[2], 1, 75);
	spiDisable();

	dataRXbuffer[0] = (uint32_t)(dataRX[0] << 16);
	dataRXbuffer[1] = (uint32_t)(dataRX[1] << 8 );
	dataRXbuffer[2] = (uint32_t)(dataRX[2]);
	uint32_t data = dataRXbuffer[0] | dataRXbuffer[1] | dataRXbuffer[2];

	return data;
}

HAL_StatusTypeDef spiWriteCalib(uint8_t address, uint32_t dataSet){
	HAL_StatusTypeDef status;
	uint8_t dataTX[3];
	dataTX[0] = (uint8_t)(dataSet >> 16);
	dataTX[1] = (uint8_t)(dataSet >> 8);
	dataTX[2] = (uint8_t)(dataSet);
	uint8_t request = address | 0x80;
	spiEnable();
	status = HAL_SPI_Transmit(&hspi2, &request, 1, 75);
	if(status == HAL_OK)status = HAL_SPI_Transmit(&hspi2, &dataTX[0], 1, 75);
	if(status == HAL_OK)status = HAL_SPI_Transmit(&hspi2, &dataTX[1], 1, 75);
	if(status == HAL_OK)status = HAL_SPI_Transmit(&hspi2, &dataTX[2], 1, 75);
	spiDisable();
	return status;
}

uint16_t spiRead16(uint8_t address){
	HAL_StatusTypeDef buffer1, buffer2;
	uint8_t request = address | 0x00;
	uint8_t dataRX[2];
	uint16_t dataRXbuffer[2];

	spiEnable();
	HAL_SPI_Transmit(&hspi2, &request, 1, 75);
	HAL_SPI_Receive(&hspi2, &dataRX[0], 1, 75);
	HAL_SPI_Receive(&hspi2, &dataRX[1], 1, 75);
	spiDisable();

	dataRXbuffer[0] = (uint16_t)(dataRX[0] << 8);
	dataRXbuffer[1] = (uint16_t)dataRX[1];
	uint16_t data = dataRXbuffer[0] | dataRXbuffer[1];

	return data;
}

uint32_t spiRead24(uint8_t address){
	 HAL_StatusTypeDef buffer1, buffer2, buffer3;
	 uint8_t request = address | 0x00;
	 uint8_t dataRX[3];
	 uint32_t dataRXBuffer[3];
	 uint32_t data;

	 spiEnable();
	 HAL_SPI_Transmit(&hspi2, &request, 1, 75);
	 buffer1 = HAL_SPI_Receive(&hspi2, &dataRX[0], 1, 75);
	 dataRXBuffer[0] = (uint32_t)(dataRX[0] << 16);
	 buffer2 = HAL_SPI_Receive(&hspi2, &dataRX[1], 1, 75);
	 dataRXBuffer[1] = (uint32_t)(dataRX[1] << 8);
	 buffer3 = HAL_SPI_Receive(&hspi2, &dataRX[2], 1, 75);
	 dataRXBuffer[2] = (uint32_t)(dataRX[2] << 0);
	 spiDisable();

	 data = dataRXBuffer[0] | dataRXBuffer[1] | dataRXBuffer[2];

	 return data;
}

void powerInit(){
	// SOFTWARE RESET
	spiCommandSpecial(w_reset, BYTE_NULL);
}

void powerSetup(uint8_t * address, uint32_t * dataSet, HAL_StatusTypeDef * dataStatus, uint8_t numberCalib){
	uint32_t check;
	// ENABLE CALIBRATION MODE & ENABLE READ CALIRATION MODE
	spiCommandSpecial(w_calib_state, BYTE_ENABLE);
	HAL_Delay(10);
	spiCommandSpecial(w_read_calib, BYTE_ENABLE);
	HAL_Delay(10);
//	// SETUP ADC STATE >> EMABLE ADC VRMS AND IRMS
//	spiWriteCalib(w_ModeCfg, 0xF9FE);
//	// SETUP EMC CONFIG
	spiWriteCalib(w_EMCfg, 0x0003);
	HAL_Delay(10);
//	// SETUP EMU CONFIG
//	spiWriteCalib(w_EMUCfg, 0x3400);
//	// READING VALUE PARAMETER

	// -----------------------------SETUP BASED ON DATASHEET---------------------------------
	/* WRITE MODE CONFIGURATION REGISTER
	 * Turn on the Vref Chopper function to improve Vref performance; turn on the power
	 * effective value slowly Speed mode to reduce jitter; configure EMU clock 921.6kHz to reduce power consumption; enable 6 ADCs;
	 */
	spiWriteCalib(w_ModeCfg,0xB97E); 	// 1011 1001 0111 1110
	HAL_Delay(10);
	/* WRITE INTO EMU
	 * Turn on energy metering, use power as the basis for creep start, turn off fundamental wave power Can,
	 * choose PQS mode for apparent power energy
	 */
	spiWriteCalib(w_EMUCfg,0xF804);		// 1111 1000 0000 0100
	HAL_Delay(10);

	/* WRITE IN THE ANALOG MODULE ENABLE REGISTER
	 * turn on the high-pass filter; turn on the BOR power monitoring circuit;
	 */
	spiWriteCalib(w_ModuleCFG,0x3427);	// 0011 0100 0010 0111
	HAL_Delay(10);

	/* WRITE CONFIG HFCONST */
//	spiWriteCalib(w_Hfconst, 0x0A00);
	HFconstVal = (float)spiReadCalib(w_Hfconst);
	HAL_Delay(10);
//	spiWriteCalib(w_UgainA, 0x022E);
//	spiWriteCalib(w_UgainB, 0x022E);
//	spiWriteCalib(w_UgainC, 0x022E);
	// READING VALUE PARAMETER
	check = spiReadCalib(w_ModeCfg);
	check = spiReadCalib(w_EMUIE);
	check = spiReadCalib(w_EMCfg);
	check = spiReadCalib(w_ModuleCFG);
	check = spiReadCalib(w_PGACtrl);
	check = spiReadCalib(w_EMUCfg);
	check = spiReadCalib(w_Hfconst);

	// WRTIE CALIBRATION PARAMETER BASED ON ATRIBUTE
	for(int indeks=0;indeks<numberCalib;indeks++){
		spiWriteCalib(address[indeks], dataSet[indeks]);
		HAL_Delay(10);
		check = spiReadCalib(address[indeks]);
		// CHECK VALUE AFTER WRIING PARAMETER REGISTER
		if(check == dataSet[indeks])dataStatus[indeks] = HAL_OK;
		else dataStatus[indeks] = HAL_ERROR;
	}
	// DISBALE CALIBRATION MODE & DISABLE READ CALIRATION MODE
	spiCommandSpecial(w_read_calib, BYTE_DISABLE);
	HAL_Delay(10);
	spiCommandSpecial(w_calib_state, BYTE_DISABLE);
	HAL_Delay(75);
}

void powerReadSensor(uint8_t * address, uint32_t * valueBuffer, float * valueFloat, uint8_t size){
	int32_t bufferSign;
	for(uint8_t indeks=0;indeks<size;indeks++){
		valueBuffer[indeks] = spiRead24(address[indeks]);
		// GROUPING DATA POWER PARAMETER >> ???
		if(indeks>=0 && indeks<20){
			// FORMULA >> powerData * 2.592*10^10/(HFconst*EC*2^23)  | HFconst = 1280(def)  &  EC = 6400
			bufferSign = unsignToSign(&valueBuffer[indeks], BIT_SIZE_24);
			valueFloat[indeks] = (float)bufferSign * COEF_POWER(HFconstVal); // (405000)/(128*64*8388608)
		}
		// GROUPING DATA RMS ???
		if(indeks>=20 && indeks<34){
			//[VRMS] FORMULA >> rmsData / 2 ^ 13
			if(indeks<23)valueFloat[indeks] = (float)valueBuffer[indeks] / 8192;
			//[VRMS COMBINE] FORMULA >> rmsData / 2 ^ 12
			else if(indeks==23)valueFloat[indeks] = (float)valueBuffer[indeks] / 4096;
			//[VRMS] FORMULA >> rmsData / 2 ^ 13
			else if(indeks<27)valueFloat[indeks] = (float)valueBuffer[indeks] / 8192;
			//[VRMS COMBINE] FORMULA >> rmsData / 2 ^ 12
			else if(indeks==27)valueFloat[indeks] = (float)valueBuffer[indeks] / 4096;
			//[LINE RMS] FORMULA >> rmsData / 2 ^ 13
			else valueFloat[indeks] = (float)valueBuffer[indeks] / 8192;
		}
		// GROUPING DATA POWER FACTOR
		if(indeks>=34 && indeks<38){
			// FORMULA >> pwrFactor / 2 ^ 23
			bufferSign = unsignToSign(&valueBuffer[indeks], BIT_SIZE_24);
			valueFloat[indeks] = (float)bufferSign / 8388608;
			HAL_Delay(10);
		}
		// GROUPING DATA ENERGY
		if(indeks<=38 && indeks<46){
			//
		}
	}
}

uint32_t powerScanValue(uint8_t address, uint32_t * addressBuffer ,uint32_t * valueBuffer, uint8_t size){
	uint32_t value = 0;
	for(uint8_t indeks=0;indeks<size;indeks++){
		if(address == addressBuffer[indeks]){
			value = valueBuffer[indeks];
			break;
		}
	}
	return value;
}

int32_t unsignToSign(uint32_t * data, uint8_t bitsize){
	int32_t value;
	// FOR BYTE LENGTH 24 BIT
	if(bitsize == BIT_SIZE_24){
		if(*data > 0x800000){						// data > 2^23
			value = (int32_t)(*data - 0x1000000); 	// data - 2^24
		}else value = *data;
	}
	// FOR BYTE LENGTH 32 BIT
	if(bitsize == BIT_SIZE_32){
		if(*data > 0x80000000){						// data > 2^31
			value = (int32_t)(*data - 0x100000000);	// data - 2^32
		}else value = *data;
	}
	return value;
}

uint32_t powerCalculateCalib(uint8_t type, uint32_t dataRaw, uint32_t dataActual){
	// CALCULATE OFFSET PARAMTER
	if((type == VRMS_OFFSET)||(type == IRMS_OFFSET)){
		uint32_t value;
		// value = data ^ 2 / 2 ^ 15
		value = (dataRaw*dataRaw) / 32768;
		return value;
	}
	// CALCULATE GAIN PARAMETER
	if(type == VRMS_GAIN){
		float gainFloat;
		float valueBuffer;
		valueBuffer = dataRaw / 8192;
		// GET GAIN VRMS
		gainFloat = ((float)dataActual/valueBuffer) - 1.000f;
		// VALUE = VALUE * 2 ^ 15
		if(gainFloat >= 0)gainFloat = gainFloat * 32768.0f;
		// VALUE = 2 ^ 16 + VALUE * 2 * 15
		if(gainFloat < 0){
			gainFloat = gainFloat * 32768.0f;
			gainFloat = gainFloat + 65536.0f;
		}
		return gainFloat;
	}
}

void powerRestoreCalib(){
	// RESTORE ALL CALIBRATION PARAMETER
	spiCommandSpecial(w_calib_restore, BYTE_NULL);
}

// USED OR UNUSED ?????????????
void powerCalib(uint8_t * addressBuffer, uint32_t * dataSet, HAL_StatusTypeDef * status, uint8_t size){
	uint32_t check;
	// ENABLE CALIBRATION MODE & ENABLE READ CALIRATION MODE
	spiCommandSpecial(w_calib_state, BYTE_ENABLE);
	spiCommandSpecial(w_read_calib, BYTE_ENABLE);
	// WRITING REGISTER FOR CALIBRATION
	for(uint8_t indeks=0;indeks<size;indeks++){
		spiWriteCalib(addressBuffer[indeks], dataSet[indeks]);
		check = spiReadCalib(addressBuffer[indeks]);
		if(check == dataSet[indeks])status[indeks] = HAL_OK;
		else status[indeks] = HAL_ERROR;
	}
	spiCommandSpecial(w_calib_state, BYTE_DISABLE);
	spiCommandSpecial(w_read_calib, BYTE_DISABLE);
}
