#include "HT7036.h"
#include "usart.h"
#include <stdio.h>
#include "main.h"

// testing start
#include "modbusSlave.h"
extern MODBUS Modbus;
// testing end

extern uint32_t valueSensor[32];
extern float valueFloat[32];
uint64_t powerTimer = 0;
uint64_t powerTimerDelta = 0;
//uint8_t dataPrint[1100];
float HFconstVal;
float ECValA = 0;
float ECValB = 0;
float ECValC = 0;
float ECVal = 0;
float ECDef = 43.7;
float bufferEnergy[8];
double bufferEnergySUM[8];
double bufferEnergyOffset[8];
uint64_t energyModbus[8];
uint32_t check;

extern float	gainVoltageA,		gainVoltageB,		gainVoltageC,
				gainCurrentA,		gainCurrentB,		gainCurrentC,
				offsetVoltageA, 	offsetVoltageB,		offsetVoltageC,
				offsetCurrentA,		offsetCurrentB,		offsetCurrentC,
				rmsVoltageA,		rmsVoltageB,		rmsVoltageC,
				rmsCurrentA,		rmsCurrentB,		rmsCurrentC;
extern float 	rmsVoltageAB,		rmsVoltageBC,		rmsVoltageCA;

extern uint64_t energyActiveA_uint;
extern uint64_t energyActiveB_uint;
extern uint64_t energyActiveC_uint;
extern uint64_t energyActiveCombine_uint;
extern uint64_t energyReactiveA_uint;
extern uint64_t energyReactiveB_uint;
extern uint64_t energyReactiveC_uint;
extern uint64_t energyReactiveCombine_uint;

extern uint16_t offsetVolt_ht7036;
extern uint16_t offsetCurr_ht7036;
extern uint16_t gainVolt_ht7036;
extern uint16_t gainCurr_ht7036;
extern uint16_t offsetVolt_stm32;
extern uint16_t offsetCurr_stm32;
extern uint16_t gainVolt_stm32;
extern uint16_t gainCurr_stm32;
extern uint16_t idHt7036;
extern float gainVoltage;
extern float gainCurrent;
extern float offsetVoltage;
extern float offsetCurrent;
extern float powerCoefActiveA;
extern float powerCoefReactiveA;
extern float powerCoefApparentA;
extern float powerCoefActiveB;
extern float powerCoefReactiveB;
extern float powerCoefApparentB;
extern float powerCoefActiveC;
extern float powerCoefReactiveC;
extern float powerCoefApparentC;
extern uint16_t gainCurrentButton_stm32;
extern float gainPF_stm32;
extern float offsetPF_stm32;
extern uint16_t calibPF_ht7036;

void spiDisable(){HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);}
void spiEnable(){HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);}

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

uint32_t spiRead24(uint8_t address){
	 uint8_t request = address | 0x00;
	 uint8_t dataRX[3];
	 uint32_t dataRXBuffer[3];
	 uint32_t data;

	 spiEnable();
	 HAL_SPI_Transmit(&hspi2, &request, 1, 75);
	 HAL_SPI_Receive(&hspi2, &dataRX[0], 1, 75);
	 dataRXBuffer[0] = (uint32_t)(dataRX[0] << 16);
	 HAL_SPI_Receive(&hspi2, &dataRX[1], 1, 75);
	 dataRXBuffer[1] = (uint32_t)(dataRX[1] << 8);
	 HAL_SPI_Receive(&hspi2, &dataRX[2], 1, 75);
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
	// -------------------------------PHASE CORRETION---------------------------------------------
	// power factor A
	// spiWriteCalib(w_PhSregApq0, PHASE_CORRECTION_ZERO);
	spiWriteCalib(w_PhSregApq1, calibPF_ht7036);
	// power factor B
	// spiWriteCalib(w_PhSregBpq0, PHASE_CORRECTION_ZERO);
	spiWriteCalib(w_PhSregBpq1, calibPF_ht7036);
	// power factor C
	// spiWriteCalib(w_PhSregCpq0, PHASE_CORRECTION_ZERO);
	spiWriteCalib(w_PhSregCpq1, calibPF_ht7036);

	// -------------------------------------------------------------------------------------------
	/* WRITE CONFIG HFCONST */
	HFconstVal = (float)spiReadCalib(w_Hfconst);
	// READING VALUE PARAMETERd
	check = spiReadCalib(w_ModeCfg);
	check = spiReadCalib(w_PhSregApq1);
	//check = spiReadCalib(w_EMCfg);
	//check = spiReadCalib(w_ModuleCFG);
	//check = spiReadCalib(w_PGACtrl);
	//check = spiReadCalib(w_EMUCfg);
	//check = spiReadCalib(w_Hfconst);

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
	idHt7036 = spiReadCalib(deviceId);
}

void powerMultiReadSensor(uint8_t * address, uint32_t * valueBuffer, float * valueFloat, uint8_t size){
	int32_t bufferSign;
	powerTimerDelta = HAL_GetTick() - powerTimer; powerTimer = HAL_GetTick();
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
			// calculate power in phase a
			if(indeks==8)valueFloat[indeks] = (float)bufferSign * powerCoefActiveA * gainCurrentA * gainCurrentButton_stm32 * gainVoltageA;
			if(indeks==9)valueFloat[indeks] = (float)bufferSign * powerCoefActiveB * gainCurrentB * gainCurrentButton_stm32 * gainVoltageB;
			if(indeks==10)valueFloat[indeks] = (float)bufferSign * powerCoefActiveC * gainCurrentC * gainCurrentButton_stm32 * gainVoltageC;

			// calculate power phase b
			if(indeks==12)valueFloat[indeks] = (float)bufferSign * powerCoefReactiveA * gainCurrentA * gainCurrentButton_stm32 * gainVoltageA;
			if(indeks==13)valueFloat[indeks] = (float)bufferSign * powerCoefReactiveB * gainCurrentB * gainCurrentButton_stm32 * gainVoltageB;
			if(indeks==14)valueFloat[indeks] = (float)bufferSign * powerCoefReactiveC * gainCurrentC * gainCurrentButton_stm32 * gainVoltageC;

			// calculate power phase c
			if(indeks==16)valueFloat[indeks] = (float)bufferSign * powerCoefApparentA * gainCurrentA * gainCurrentButton_stm32 * gainVoltageA;
			if(indeks==17)valueFloat[indeks] = (float)bufferSign * powerCoefApparentB * gainCurrentB * gainCurrentButton_stm32 * gainVoltageB;
			if(indeks==18)valueFloat[indeks] = (float)bufferSign * powerCoefApparentC * gainCurrentC * gainCurrentButton_stm32 * gainVoltageC;

			// calculate power combine phase abc
			if(indeks==11)valueFloat[indeks] = (float)bufferSign * 2 * powerCoefActiveA * gainCurrentA * gainCurrentButton_stm32 * gainVoltageA; // (405000)/(128*64*8388608)
			if(indeks==15)valueFloat[indeks] = (float)bufferSign * 2 * powerCoefReactiveB * gainCurrentB * gainCurrentButton_stm32 * gainVoltageB; // (405000)/(128*64*8388608)
			if(indeks==19)valueFloat[indeks] = (float)bufferSign * 2 * powerCoefApparentC * gainCurrentC * gainCurrentButton_stm32 * gainVoltageC; // (405000)/(128*64*8388608)
			// SAMPLING DATA ACTEVE REACTIVE POWER FOR ENERGY CALCULTION
			if((indeks-8)>=0 && (indeks-8)<8){
				bufferEnergy[indeks-8] = valueFloat[indeks];
			}
		}
		// GROUPING DATA POWER FACTOR
		if(indeks>=20 && indeks<24){
			// FORMULA >> pwrFactor / 2 ^ 23
			bufferSign = unsignToSign(&valueBuffer[indeks], BIT_SIZE_24);
			handleAbsolute32(&bufferSign);
			valueFloat[indeks] = (float)bufferSign / 8388608;
			// HANDLING REGRESION LINIER
			valueFloat[indeks] = (valueFloat[indeks] * gainPF_stm32) + offsetPF_stm32;
			if(valueFloat[indeks] > 1)valueFloat[indeks] = 1;
			else if(valueFloat[indeks] < 0)valueFloat[indeks] = 0;
		}
		// GROUPING DATA ENERGY
		if(indeks>=24 && indeks<32){
			// ABSOLUTED VALUE FUNCTION
			handleAbsolute(&bufferEnergy[indeks-24]);
			// CALCULATION MANUAL DATA SENSOR ENERGY => power*deltaSampling/3600000 >> all value must be uin64_t type variable
			bufferEnergySUM[indeks-24] += (double)(bufferEnergy[indeks-24]*((float)powerTimerDelta/1000.00f)/3600.00f);  // watt hour
			energyModbus[indeks-24] = (uint64_t)bufferEnergySUM[indeks-24];
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
	uint32_t value;
	// CALCULATE OFFSET PARAMTER
	if((type == VRMS_OFFSET)||(type == IRMS_OFFSET)){
		// value = data ^ 2 / 2 ^ 15
		value = (dataRaw*dataRaw) / 32768;
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
		value = (uint32_t)gainFloat;
	}
	return value;
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

uint32_t powerSingleRecalib(uint8_t type, uint8_t addressWrite, uint32_t * dataSet, uint8_t addressRead, HAL_StatusTypeDef * status, uint32_t dataOld){
	uint32_t dataWrite, dataRaw, buffer32;
	// RESET PARAMETER CALIBRATION
	dataWrite = 0;
	powerSingleCalib(addressWrite, &dataWrite, status);
	HAL_Delay(1000);
	// GETTING DATA SENSOR NON-CALIBRATION
	dataRaw = spiRead24(addressRead);
	if((type == VRMS_OFFSET) || (type == IRMS_OFFSET)){
		buffer32 = powerCalculateCalib(type, dataRaw, 0);
		if(buffer32 != 0xffffffff){
			dataWrite = buffer32;
			if(*dataSet == 1)powerSingleCalib(addressWrite, &dataWrite, status);
			else if(*dataSet > 1)powerSingleCalib(addressWrite, dataSet, status);
		}else dataWrite = dataOld;
	}
	if((type == VRMS_GAIN) || (type == IRMS_GAIN)){
		buffer32 = powerCalculateCalib(type, dataRaw, (float)*dataSet/100);
		if(buffer32 != 0xffffffff){
			dataWrite = buffer32;
			powerSingleCalib(addressWrite, &dataWrite, status);
		}else dataWrite = dataOld;
	}
	if(type == PF_CALIB){
		powerSingleCalib(addressWrite, dataSet, status);
	}
	return dataWrite;
}

void handleAbsolute(float * value){
	if(*value < 0){
		*value = *value *(-1);
	}
}

void handleAbsolute32(int32_t * value){
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
	//  ((V A + V B)/2)*sqr(1/2)  | 1,4142135623730950488016887242097 >> akar2 dari 2
	return ((val1 + val2)/2*1.7320508075688772935274463415);
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
uint16_t byte32High(uint32_t buf){return (uint16_t)((buf & 0xFFFF0000) >> 16);}
uint16_t byte32Low(uint32_t buf){return (uint16_t)(buf & 0x0000FFFF);}
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

//void calculatePower(){
//	float powerApparentA,powerApparentB,powerApparentC;
//	// power Apparent A
//	powerApparentA = ((valueFloat[0]*gainVoltage)+offsetVoltage) * ((valueFloat[4]*gainCurrent*gainCurrentButton_stm32)+offsetCurrent);
//	// power Apparent B
//	powerApparentB = ((valueFloat[1]*gainVoltage)+offsetVoltage) * ((valueFloat[4]*gainCurrent*gainCurrentButton_stm32)+offsetCurrent);
//	// power Apparent C
//	powerApparentC = ((valueFloat[2]*gainVoltage)+offsetVoltage) * ((valueFloat[5]*gainCurrent*gainCurrentButton_stm32)+offsetCurrent);
//	// POWER ACTIVE A
//	bufferEnergy[0] = powerApparentA * valueFloat[20];
//	// POWER ACTIVE B
//	bufferEnergy[1] = powerApparentB * valueFloat[21];
//	// POWER ACTIVE C
//	bufferEnergy[2] = powerApparentC * valueFloat[22];
//}
