#include "HT7036.h"
#include "usart.h"
#include <stdio.h>

extern uint32_t valueSensor[32];
extern float valueFloat[32];
extern uint64_t powerTimerDelta;
uint64_t valueUint64[8];
uint8_t dataPrint[1100];
float HFconstVal;
float ECValA = 0;
float ECValB = 0;
float ECValC = 0;
float ECVal = 0;
float ECDef = 43.7;
float bufferEnergy[8];
double bufferEnergySUM[8];
uint64_t energyModbus[8];

extern float	gainVoltageA,		gainVoltageB,		gainVoltageC,
				gainCurrentA,		gainCurrentB,		gainCurrentC,
				offsetVoltageA, 	offsetVoltageB,		offsetVoltageC,
				offsetCurrentA,		offsetCurrentB,		offsetCurrentC;

extern uint16_t offsetVolt_ht7036,	offsetCurr_ht7036,	gainVolt_ht7036,	gainCurr_ht7036,
				offsetVolt_stm32,	offsetCurr_stm32,	gainVolt_stm32,		gainCurr_stm32;

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
	spiCommandSpecial(w_read_calib, BYTE_ENABLE);
	spiWriteCalib(w_EMCfg, 0x0003);
	// -----------------------------SETUP BASED ON DATASHEET---------------------------------
	/* WRITE MODE CONFIGURATION REGISTER
	 * Turn on the Vref Chopper function to improve Vref performance; turn on the power
	 * effective value slowly Speed mode to reduce jitter; configure EMU clock 921.6kHz
	 * to reduce power consumption; enable 6 ADCs;
	 */
	spiWriteCalib(w_ModeCfg,0xB97E); 	// 1011 1001 0111 1110
	/* WRITE INTO EMU
	 * Turn on energy metering, use power as the basis for creep start, turn off fundamental wave power Can,
	 * choose PQS mode for apparent power energy
	 */
	spiWriteCalib(w_EMUCfg,0xF804);		// 1111 1000 0000 0100
	/* WRITE IN THE ANALOG MODULE ENABLE REGISTER
	 * turn on the high-pass filter; turn on the BOR power monitoring circuit;
	 */
	spiWriteCalib(w_ModuleCFG,0x3427);	// 0011 0100 0010 0111
	// -------------------------------------------------------------------------------------

	/* WRITE CONFIG HFCONST */
	HFconstVal = (float)spiReadCalib(w_Hfconst);
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

void powerMultiReadSensor(uint8_t * address, uint32_t * valueBuffer, float * valueFloat, uint8_t size){
	int32_t bufferSign;
	uint8_t stateEnergy = 1;
	for(uint8_t indeks=0;indeks<size;indeks++){
		valueBuffer[indeks] = spiRead24(address[indeks]);
		// GROUPING DATA RMS ???
		if(indeks>=0 && indeks<8){
			//[VRMS] FORMULA >> rmsData / 2 ^ 13
			if(indeks<3)valueFloat[indeks] = (float)valueBuffer[indeks] / 8192;
			//[VRMS COMBINE] FORMULA >> rmsData / 2 ^ 12
			else if(indeks==3)valueFloat[indeks] = (float)valueBuffer[indeks] / 4096;
			//[VRMS] FORMULA >> rmsData / 2 ^ 13
			else if(indeks<7)valueFloat[indeks] = (float)valueBuffer[indeks] / 8192;
			//[VRMS COMBINE] FORMULA >> rmsData / 2 ^ 12
			else if(indeks==7)valueFloat[indeks] = (float)valueBuffer[indeks] / 4096;
			//[LINE RMS] FORMULA >> rmsData / 2 ^ 13
			else valueFloat[indeks] = (float)valueBuffer[indeks] / 8192;
		}
		// GROUPING DATA POWER PARAMETER >> solved
		if(indeks>=8 && indeks<20){
			if(ECVal == 0)ECVal = ECDef;
			// FORMULA >> powerData * 2.592*10^10/(HFconst*EC*2^23)  | HFconst = 1280(def)  &  EC = 6400
			bufferSign = unsignToSign(&valueBuffer[indeks], BIT_SIZE_24);
			valueFloat[indeks] = (float)bufferSign * coefPower(HFconstVal, ECVal);
			// FORMULA >> powerdata * 2 * 2.592*10^10/(HFconst*EC*2^23)
			if((indeks==11)||(indeks==15)||(indeks==19))valueFloat[indeks] = (float)bufferSign * 2 * coefPower(HFconstVal, ECVal); // (405000)/(128*64*8388608)
			// SAMPLING DATA ACTEVE REACTIVE POWER FOR ENERGY CALCULTION
			if((indeks-8)>=0 && (indeks-8)<8){
				bufferEnergy[indeks-8] = valueFloat[indeks];
			}
		}
		// GROUPING DATA POWER FACTOR
		if(indeks>=20 && indeks<24){
			// FORMULA >> pwrFactor / 2 ^ 23
			bufferSign = unsignToSign(&valueBuffer[indeks], BIT_SIZE_24);
			valueFloat[indeks] = (float)bufferSign / 8388608;
			HAL_Delay(10);
		}
		// GROUPING DATA ENERGY
		if(indeks>=24 && indeks<32){
			handleAbsolute(&bufferEnergy[indeks-24]);
			// CALCULATION MANUAL DATA SENSOR ENERGY => power*deltaSampling/3600000 >> all value must be uin64_t type variable
			bufferEnergySUM[indeks-24] += (double)((float)bufferEnergy[indeks-24]*((float)powerTimerDelta/1000.00f)/3600.00f);  // watt hour
			memcpy(&energyModbus[indeks-24], &bufferEnergySUM[indeks-24],sizeof(bufferEnergySUM[indeks-24]));
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
	// FOR BYTE LENGTH 21 BIT
	if(bitsize == BIT_SIZE_21){
		if(*data >= 0x100000){						// data >= 2^20
			value = (int32_t)(*data - 0x1000000);	// data - 2^24
		} else value = *data;
	}
	return value;
}

uint32_t powerCalculateCalib(uint8_t type, uint32_t dataRaw, float dataActual){
	float gainFloat;
	float valueBuffer;
	// CALCULATE OFFSET PARAMTER
	if((type == VRMS_OFFSET)||(type == IRMS_OFFSET)){
		uint32_t value;
		// value = data ^ 2 / 2 ^ 15
		value = (dataRaw*dataRaw) / 32768;
		return value;
	}
	// CALCULATE GAIN PARAMETER
	if((type == VRMS_GAIN)||(type == IRMS_GAIN)){
		if(type == VRMS_GAIN) valueBuffer = (float)dataRaw / 8192;
		if(type == IRMS_GAIN) valueBuffer = (float)dataRaw / 8192;
		// GET GAIN VRMS
		gainFloat = (dataActual/valueBuffer) - 1.000f;
		// VALUE = VALUE * 2 ^ 15
		if(gainFloat >= 0)gainFloat = gainFloat * 32768.0f;
		// VALUE = 2 ^ 16 + VALUE * 2 * 15
		if(gainFloat < 0){
			gainFloat = gainFloat * 32768.0f;
			gainFloat = gainFloat + 65536.0f;
		}
		return (uint32_t)gainFloat;
	}
}

void powerRestoreCalib(){
	// RESTORE ALL CALIBRATION PARAMETER
	spiCommandSpecial(w_calib_restore, BYTE_NULL);
	HAL_Delay(100);
}

void powerMultiCalib(uint8_t * addressBuffer, uint32_t * dataSet, HAL_StatusTypeDef * status, uint8_t size){
	uint32_t check;
	// ENABLE CALIBRATION MODE & ENABLE READ CALIRATION MODE
	powerCalibMode(ENABLE);
	// WRITING REGISTER FOR CALIBRATION
	for(uint8_t indeks=0;indeks<size;indeks++){
		spiWriteCalib(addressBuffer[indeks], dataSet[indeks]);
		check = spiReadCalib(addressBuffer[indeks]);
		if(check == dataSet[indeks])status[indeks] = HAL_OK;
		else status[indeks] = HAL_ERROR;
	}
	// DISABLE CALIBRATION MODE & ENABLE READ CALIRATION MODE
	powerCalibMode(DISABLE);
}

void powerSingleCalib(uint8_t addressBuffer, uint32_t * dataSet, HAL_StatusTypeDef * status){
	uint32_t check;
	// ENABLE CALIBRATION MODE & ENABLE READ CALIRATION MODE
	powerCalibMode(ENABLE);
	// WRITING REGISTER FOR CALIBRATION
	spiWriteCalib(addressBuffer, *dataSet);
	check = spiReadCalib(addressBuffer);
	if(check == *dataSet) *status = HAL_OK;
	else *status = HAL_ERROR;
	// DISABLE CALIBRATION MODE & ENABLE READ CALIRATION MODE
	powerCalibMode(DISABLE);
}

uint32_t powerSingleRecalib(uint8_t type, uint8_t addressWrite, uint32_t * dataSet, uint8_t addressRead, HAL_StatusTypeDef * status){
	uint32_t dataWrite, dataRaw;
	// RESET PARAMETER CALIBRATION
	dataWrite = 0;
	powerSingleCalib(addressWrite, &dataWrite, status);
	HAL_Delay(1000);
	// GETTING DATA SENSOR NON-CALIBRATION
	dataRaw = spiRead24(addressRead);
	if((type == VRMS_OFFSET) || (type == IRMS_OFFSET)){
		dataWrite = powerCalculateCalib(type, dataRaw, 0);
		if(*dataSet == 1)powerSingleCalib(addressWrite, &dataWrite, status);
		else if(*dataSet > 1)powerSingleCalib(addressWrite, dataSet, status);
	}
	if((type == VRMS_GAIN) || (type == IRMS_GAIN)){
		dataWrite = powerCalculateCalib(type, dataRaw, (float)*dataSet/100);
		powerSingleCalib(addressWrite, &dataWrite, status);
	}
	return dataWrite;
}

void handleAbsolute(double * value){
	if(*value < 0){
		*value = *value *(-1);
	}
}

void powerCalibMode(uint8_t state){
	if(state == (uint8_t)ENABLE){
		spiCommandSpecial(w_calib_state, BYTE_ENABLE);
		spiCommandSpecial(w_read_calib, BYTE_ENABLE);
	}
	else if(state == (uint8_t)DISABLE){
		spiCommandSpecial(w_calib_state, BYTE_DISABLE);
		spiCommandSpecial(w_read_calib, BYTE_DISABLE);
	}
}
float calcVoltDif(float val1, float val2){
	//((V A + V B)/2)*sqr(1/2)  | 1,4142135623730950488016887242097 >> akar2 dari 2
	return ((val1 + val2)/2*1.4142135623730950488016887242097);
}
float calcMeterConstant(uint32_t dataBit, float hfConst, float dataAcual){
	if(dataAcual > 0){
		return ((dataBit*2.592*10000000000)/(8388608*hfConst*dataAcual));
	}else{
		return ECVal;
	}
}
float coefPower(float hfconst, float ec){return ((2.592*10000000000)/(hfconst*ec*8388608.000f));}
uint32_t floatToInt32(float * data){return *((uint32_t*)data);}
float int32ToFloat(uint32_t * data){return *((float*)data);}

// SPLIT BYTE
uint16_t byte64High1(uint64_t buf){return(uint16_t)((buf & 0xFFFF000000000000) >> 48);}
uint16_t byte64High2(uint64_t buf){return(uint16_t)((buf & 0xFFFF00000000) >> 32);}
uint16_t byte64Low1(uint64_t buf){return(uint16_t)((buf & 0xFFFF0000) >> 16);}
uint16_t byte64Low2(uint64_t buf){return (uint16_t)((buf & 0xFFFF));}
uint8_t byte16Low(uint16_t buf){return (uint8_t)((buf & 0x00FF));}
uint8_t byte16High(uint16_t buf){return (uint8_t)((buf & 0xFF00) >> 8);}
void uint64ToUint8(uint8_t * buffer, uint64_t data){
	buffer[0] = byte16High(byte64High1(data));
	buffer[1] = byte16Low(byte64High1(data));
	buffer[2] = byte16High(byte64High2(data));
	buffer[3] = byte16Low(byte64High2(data));
	buffer[4] = byte16High(byte64Low1(data));
	buffer[5] = byte16Low(byte64Low1(data));
	buffer[6] = byte16High(byte64Low2(data));
	buffer[7] = byte16Low(byte64Low2(data));
}

void uint8Touint64(uint64_t * buffer, uint8_t * data){
	uint64_t buffer64;
	uint32_t buffer32;
	uint16_t buffer16;

	buffer16 = uint8ToUint16(data[0], data[1]);
	buffer64 = (uint64_t)buffer16 << 48;
	buffer16 = uint8ToUint16(data[2], data[3]);
	buffer64 = buffer64 | ((uint64_t)buffer16 << 32);
	buffer16 = uint8ToUint16(data[4], data[5]);
	buffer64 = buffer64 | ((uint64_t)buffer16 << 16);
	buffer16 = uint8ToUint16(data[6], data[7]);
	buffer64 = buffer64 | (uint64_t)(buffer16);

	*buffer = buffer64;
}

// PACK BYTE
uint64_t uint32ToUint64(uint32_t high, uint32_t low){return (((uint64_t)high<<32) | (uint64_t)low);}
uint32_t uint16ToUint32(uint16_t high, uint16_t low){return((uint32_t)high<<16 | (uint32_t)low);}
uint16_t uint8ToUint16(uint8_t high, uint8_t low){return ((uint16_t)high<<8 | (uint16_t)low);}



void powerDebug(){
	// ENABLE MODBUS
	HAL_GPIO_WritePin(MODBUS_En_GPIO_Port,MODBUS_En_Pin,GPIO_PIN_RESET);

	sprintf(dataPrint,"\r\n=======Sampling Time %d(ms)=======\r\n",powerTimerDelta);
	HAL_UART_Transmit(&huart2, dataPrint, 40, 1000);
	memset(dataPrint, 0, sizeof(dataPrint));
	// DEBUGGING VALUE >> GROUP POWER
	serialPrint("\r\n------------Power Active------------\r\n", 40);
	sprintf(dataPrint,"A=%.6f(%d)\r\nB=%.6f(%d)\r\nC=%.6f(%d)\r\nCombine=%.6f(%d)\r\n",
				  valueFloat[8],valueSensor[8],
				  valueFloat[9],valueSensor[9],
				  valueFloat[10],valueSensor[10],
				  valueFloat[11],valueSensor[11]
	);
	HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
	memset(dataPrint, 0, sizeof(dataPrint));
	serialPrint("\r\n------------Power Rective-----------\r\n", 40);
	sprintf(dataPrint,"A=%.6f(%d)\r\nB=%.6f(%d)\r\nC=%.6f(%d),\r\nCombine=%.6f(%d)\r\n",
				  valueFloat[12],valueSensor[12],
				  valueFloat[13],valueSensor[13],
				  valueFloat[14],valueSensor[14],
				  valueFloat[15],valueSensor[15]
	);
	HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
	memset(dataPrint, 0, sizeof(dataPrint));
	serialPrint("\r\n------------Power Apparent----------\r\n", 40);
	sprintf(dataPrint,"A=%.6f(%d)\r\nB=%.6f(%d)\r\nC=%.6f(%d)\r\nCombine=%.6f(%d)\r\n",
				  valueFloat[16],valueSensor[16],
				  valueFloat[17],valueSensor[17],
				  valueFloat[18],valueSensor[18],
				  valueFloat[19],valueSensor[19]
	);
	HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
	memset(dataPrint, 0, sizeof(dataPrint));
	serialPrint("\r\n------------Voltage RMS-------------\r\n", 40);
	sprintf(dataPrint,"A=%.6f(%d)\r\nB=%.6f(%d)\r\nC=%.6f(%d)\r\nVector=%.6f(%d)\r\n",
				  valueFloat[0],valueSensor[0],
				  valueFloat[1],valueSensor[1],
				  valueFloat[2],valueSensor[2],
				  valueFloat[3],valueSensor[3]
	);
	HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
	memset(dataPrint, 0, sizeof(dataPrint));
	serialPrint("\r\n------------Current RMS-------------\r\n", 40);
	sprintf(dataPrint,"A=%.6f(%d)\r\nB=%.6f(%d)\r\nC=%.6f(%d)\r\nVector=%.6f(%d)\r\n",
				  valueFloat[4],valueSensor[4],
				  valueFloat[5],valueSensor[5],
				  valueFloat[6],valueSensor[6],
				  valueFloat[7],valueSensor[7]
	);
	HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
	memset(dataPrint, 0, sizeof(dataPrint));
	serialPrint("\r\n------------Power Factor------------\r\n", 40);
	sprintf(dataPrint,"A=%.6f(%d)\r\nB=%.6f(%d)\r\nC=%.6f(%d)\r\nCombine=%.6f(%d)\r\n",
				  valueFloat[20],valueSensor[20],
				  valueFloat[21],valueSensor[21],
				  valueFloat[22],valueSensor[22],
				  valueFloat[23],valueSensor[23]
	);
	HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
	memset(dataPrint, 0, sizeof(dataPrint));
	serialPrint("\r\n------------Energy Active-----------\r\n", 40);
	sprintf(dataPrint,"A=%0.6f(%lu)\r\nB=%0.6f(%lu)\r\nC=%0.6f(%lu)\r\nCombine=%0.6f(%lu)\r\n",
				  bufferEnergySUM[0],energyModbus[24],
				  bufferEnergySUM[1],energyModbus[25],
				  bufferEnergySUM[2],energyModbus[26],
				  bufferEnergySUM[3],energyModbus[27]
		  );
	HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
	memset(dataPrint, 0, sizeof(dataPrint));
	serialPrint("\r\n------------Energy Reactive-----------\r\n", 40);
	sprintf(dataPrint,"A=%0.6f(%lu)\r\nB=%0.6f(%lu)\r\nC=%0.6f(%lu)\r\nCombine=%0.6f(%lu)\r\n",
				  bufferEnergySUM[4],energyModbus[28],
				  bufferEnergySUM[5],energyModbus[29],
				  bufferEnergySUM[6],energyModbus[30],
				  bufferEnergySUM[7],energyModbus[31]
		  );
	HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
	memset(dataPrint, 0, sizeof(dataPrint));
//	serialPrint("\r\n------------Energy Buffer-----------\r\n", 40);
//	sprintf(dataPrint,"A=%0.6f(%0.6f)\r\nB=%0.6f(%0.6f)\r\nC=%0.6f(%0.6f)\r\nCombine=%0.6f(%0.6f)\r\n",
//				  bufferEnergy32[0],bufferEnergy32[1],
//				  bufferEnergy32[2],bufferEnergy32[3],
//				  bufferEnergy32[4],bufferEnergy32[5],
//				  bufferEnergy32[6],bufferEnergy32[7]
//		  );
//	HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
//	memset(dataPrint, 0, sizeof(dataPrint));
	serialPrint("\r\n------------Else Sensor-----------\r\n", 40);
	sprintf(dataPrint,"EC=%.6f\r\n",
				  ECVal
		  );
	HAL_UART_Transmit(&huart2, dataPrint, 20, 100);
	memset(dataPrint, 0, sizeof(dataPrint));


	// DISABLE MODBUS
	HAL_GPIO_WritePin(MODBUS_En_GPIO_Port,MODBUS_En_Pin,GPIO_PIN_SET);
}
// SETUP CALIB
/* Dataframe
 * GV,actualData
 * GI,actualData
 * OV
 * OI
 * MC,actuaData
 */
//void powerSetupCalib(uint8_t type, uint32_t dataActual){
//	uint32_t calibVal;
//	if(type == VRMS_GAIN){
//		spiRead24(r_IaRms);
//		calibVal = powerCalculateCalib(type, valueSensor, dataActual);
//	}
//
//}

/*
 * NOTE >
 * dataRX[0] =  powerCalculateCalib(VRMS_GAIN, 2115766, 220); 	// 02 august >> 60709
 * dataRX[0] =  powerCalculateCalib(VRMS_GAIN, 2083696, 227); 	// 03 august >> 62011
 * dataRX[0] = powerCalculateCalib(IRMS_GAIN, 19715, 1.19);	// 03 august >> 48970
 * dataRX[0] = powerCalculateCalib(IRMS_GAIN, 18225, 1.16);	// 04 august >> 49853

 */

// -------------------------------------------------------------------------DATA PRINT FOR DEBUGGING----------------------------------------------------------------------
//sprintf(
//	  			  dataPrint,
//	  			  "\r\n\r\nangleA=%.2f, angleB=%.2f, angleC=%.2f, npowerActiveA=%.6f(%d)[%.6f] ,powerActiveB=%.6f(%d)[%.6f], powerActiveC=%.6f(%d)[%.6f], PowerActiveCombine=%.6f(%d)[%.6f], powerReactiveA=%.6f(%d), powerReactiveB=%.6f(%d), powerReactiveC=%.6f(%d), powerReactiveCombine=%.6f(%d), powerApparentA=%.6f(%d), powerApparentB=%.6f(%d), powerApparentC=%.6f(%d), powerApparentCombine=%.6f(%d), rmsVoltageA=%.6f(%d), rmsVoltageB=%.6f(%d), rmsVoltageC=%.6f(%d), rmsVoltageVector=%.6f(%d), rmsCurrentA=%.6f(%d), rmsCurrentB=%.6f(%d), rmsCurrentC=%.6f(%d), rmsCurrentVector=%.6f(%d), powerFactorA=%.6f(%d), powerFactorB=%.6f(%d), powerFactorC=%.6f(%d), powerFactorCombine=%.6f(%d), energyActiveA=%.6f(%d), energyActiveB=%.6f(%d), energyActiveC=%.6f(%d), energyActiveCombine=%.6f(%d), energyReactiveA=%.6f(%d), energyReactiveB=%.6f(%d), energyReactiveC=%.6f(%d), energyReactiveCombine=%.6f(%d)\r\n\r\n",
//	  			  angle[0],angle[1],angle[2],valueFloat[0],valueSensor[0], valueFloat[0]*467,valueFloat[1],valueSensor[1],valueFloat[1]*467,valueFloat[2],valueSensor[2],valueFloat[2]*467,valueFloat[3],valueSensor[3],valueFloat[3]*467,valueFloat[4],valueSensor[4],
//	  			  valueFloat[5],valueSensor[5],valueFloat[6],valueSensor[6],valueFloat[7],valueSensor[7],valueFloat[8],valueSensor[8],
//	  			  valueFloat[9],valueSensor[9],valueFloat[10],valueSensor[10],valueFloat[11],valueSensor[11],valueFloat[20],valueSensor[20],
//	  			  valueFloat[21],valueSensor[21],valueFloat[22],valueSensor[22],valueFloat[23],valueSensor[23],valueFloat[24],valueSensor[24],valueFloat[25],valueSensor[25],
//	  			  valueFloat[26],valueSensor[26],valueFloat[27],valueSensor[27],valueFloat[34],valueSensor[34],valueFloat[35],valueSensor[35],valueFloat[36],valueSensor[36],valueFloat[37],valueSensor[37],
//	  			  valueFloat[38],valueSensor[38],valueFloat[39],valueSensor[39],valueFloat[40],valueSensor[40],valueFloat[41],valueSensor[41],
//	  			  valueFloat[42],valueSensor[42],valueFloat[43],valueSensor[43],valueFloat[44],valueSensor[44],valueFloat[45],valueSensor[45]
//	  );
// -------------------------------------------------------------------------DATA PRINT FOR DEBUGGING----------------------------------------------------------------------
// READING ANGLE POWER
//	  float angle[3];
//	  uint32_t angleBuffer[3];
//	  int32_t angleSign[3];
//	  uint8_t angleAddress[3] = {
//			  r_Pga,
//			  r_Pgb,
//			  r_Pgc,
//	  };
//	  for(int indeks=0; indeks<3; indeks++){
//		  angleBuffer[indeks] = spiRead24(angleAddress[indeks]);
//		  angleSign[indeks] = unsignToSign(&angleBuffer[indeks], BIT_SIZE_21);
//		  angle[indeks] = angleSign[indeks]*180/0x100000;	// dataSign*180/2^20
//	  }

//	  sprintf(dataPrint,"\r\n=======Sampling Time %d(ms)=======\r\n",powerTimerDelta);
//	  HAL_UART_Transmit(&huart2, dataPrint, 40, 1000);
//	  memset(dataPrint, 0, sizeof(dataPrint));
//	  powerMultiReadSensor(addrSensor, valueSensor, valueFloat, 46);
//	  // DEBUGGING VALUE >> GROUP POWER
//	  serialPrint("\r\n------------Power Active------------\r\n", 40);
//	  sprintf(dataPrint,"A=%.6f(%d)\r\nB=%.6f(%d)\r\nC=%.6f(%d)\r\nCombine=%.6f(%d)\r\n",
//			  valueFloat[0],valueSensor[0],
//			  valueFloat[1],valueSensor[1],
//			  valueFloat[2],valueSensor[2],
//			  valueFloat[3],valueSensor[3]
//	  );
//	  HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
//	  memset(dataPrint, 0, sizeof(dataPrint));
//	  serialPrint("\r\n------------Power Rective-----------\r\n", 40);
//	  sprintf(dataPrint,"A=%.6f(%d)\r\nB=%.6f(%d)\r\nC=%.6f(%d),\r\nCombine=%.6f(%d)\r\n",
//			  valueFloat[4],valueSensor[4],
//			  valueFloat[5],valueSensor[5],
//			  valueFloat[6],valueSensor[6],
//			  valueFloat[7],valueSensor[7]
//	  );
//	  HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
//	  memset(dataPrint, 0, sizeof(dataPrint));
//	  serialPrint("\r\n------------Power Apparent----------\r\n", 40);
//	  sprintf(dataPrint,"A=%.6f(%d)\r\nB=%.6f(%d)\r\nC=%.6f(%d)\r\nCombine=%.6f(%d)\r\n",
//			  valueFloat[8],valueSensor[8],
//			  valueFloat[9],valueSensor[9],
//			  valueFloat[10],valueSensor[10],
//			  valueFloat[11],valueSensor[11]
//	  );
//	  HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
//	  memset(dataPrint, 0, sizeof(dataPrint));
//	  serialPrint("\r\n------------Voltage RMS-------------\r\n", 40);
//	  sprintf(dataPrint,"A=%.6f(%d)\r\nB=%.6f(%d)\r\nC=%.6f(%d)\r\nVector=%.6f(%d)\r\n",
//			  valueFloat[20],valueSensor[20],
//			  valueFloat[21],valueSensor[21],
//			  valueFloat[22],valueSensor[22],
//			  valueFloat[23],valueSensor[23]
//	  );
//	  HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
//	  memset(dataPrint, 0, sizeof(dataPrint));
//	  serialPrint("\r\n------------Current RMS-------------\r\n", 40);
//	  sprintf(dataPrint,"A=%.6f(%d)\r\nB=%.6f(%d)\r\nC=%.6f(%d)\r\nVector=%.6f(%d)\r\n",
//			  valueFloat[24],valueSensor[24],
//			  valueFloat[25],valueSensor[25],
//			  valueFloat[26],valueSensor[26],
//			  valueFloat[27],valueSensor[27]
//	  );
//	  HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
//	  memset(dataPrint, 0, sizeof(dataPrint));
//	  serialPrint("\r\n------------Power Factor------------\r\n", 40);
//	  sprintf(dataPrint,"A=%.6f(%d)\r\nB=%.6f(%d)\r\nC=%.6f(%d)\r\nCombine=%.6f(%d)\r\n",
//			  valueFloat[34],valueSensor[34],
//			  valueFloat[35],valueSensor[35],
//			  valueFloat[36],valueSensor[36],
//			  valueFloat[37],valueSensor[37]
//	  );
//	  HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
//	  memset(dataPrint, 0, sizeof(dataPrint));
//	  serialPrint("\r\n------------Energy Active-----------\r\n", 40);
//	  sprintf(dataPrint,"A=%.6f(%d)\r\nB=%.6f(%d)\r\nC=%.6f(%d)\r\nCombine=%.6f(%d)\r\n",
//			  valueFloat[38],valueSensor[38],
//			  valueFloat[39],valueSensor[39],
//			  valueFloat[40],valueSensor[40],
//			  valueFloat[41],valueSensor[41]
//	  );
//	  HAL_UART_Transmit(&huart2, dataPrint, 100, 100);
//	  memset(dataPrint, 0, sizeof(dataPrint));
//	  HAL_Delay(1000);
//uint8_t addrSensor[] = {
//		// POWER REGISTER >> 20 addrs
//		r_Pa,				r_Pb,				r_Pc,				r_Pt,
//		r_Qa,				r_Qb,				r_Qc,				r_Qt,
//		r_Sa,				r_Sb,				r_Sc,				r_St,
//		r_LinePa,			r_LinePb,			r_LinePc,			r_LinePt,
//		r_LineQa,			r_LineQb,			r_LineQc,			r_LineQt,
//		// RMS REGISTER >> 14 addrs
//		r_UaRms,			r_UbRms,			r_UcRms,			r_UtRms,
//		r_IaRms,			r_IbRms,			r_IcRms,			r_ItRms,
//		r_LineUaRrms,		r_LineUbRrms, 		r_LineUcRrms,
//		r_LineIaRrms, 		r_LineIbRrms,		r_LineIcRrms,
//		// POWER FACTOR REGISTER >> 4 addrs
//		r_Pfa,				r_Pfb,				r_Pfc,				r_Pft,
//		// ENERGY REGISTER >> 8 addrs
//		r_Epa,				r_Epb, 				r_Epc,				r_Ept,
//		r_Eqa,				r_Eqb,				r_Eqc,				r_Eqt
//		// TOTAL REGISTER >> 46
//};
// POWER REGISTER
//float 	powerActiveA,		powerActiveB,		powerActiveC,		PowerActiveCombine,   // V x i x cos phi
//		powerReactiveA,		powerReactiveB,		powerReactiveC,		powerReactiveCombine, // V x i x sin phi
//		powerApparentA,		powerApparentB,		powerApparentC,		powerApparentCombine, // V x i
//		powerActiveWaveA,	powerActiveWaveB,	powerActiveWaveC,	powerActiveWaveSum,
//		powerReactiveWaveA,	powerReactiveWaveB,	powerReactiveWaveC,	powerReactiveWaveCombine;
//
//// RMS REGISTER
//float 	rmsVoltageA,		rmsVoltageB,		rmsVoltageC,		rmsVoltageVector,
//		rmsCurrentA,		rmsCurrentB,		rmsCurrentC,		rmsCurrentVector,
//		rmsVoltageWaveA,	rmsVoltageWaveB,	rmsVoltageWaveC,
//		rmsCurrentWaveA,	rmsCurrentWaveB,	rmsCurrentWaveC;
//
//// POWER FACTOR REGISTER
//float 	powerFactorA,		powerFactorB,		powerFactorC, 		powerFactorCombine;
//
//// ENERGY REGISTER
//float 	energyActiveA,		energyActiveB, 		energyActiveC,		energyActiveCombine,
//		energyReactiveA,	energyReactiveB, 	energyReactiveC, 	energyReactiveCombine;
