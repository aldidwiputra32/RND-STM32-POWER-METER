#include "HT7036.h"

uint8_t addrSensor[] = {
		// POWER REGISTER >> 20 addrs
		r_Pa,				r_Pb,				r_Pc,				r_Pt,
		r_Qa,				r_Qb,				r_Qc,				r_Qt,
		r_Sa,				r_Sb,				r_Sc,				r_St,
		r_LinePa,			r_LinePb,			r_LinePc,			r_LinePt,
		r_LineQa,			r_LineQb,			r_LineQc,			r_LineQt,
		// RMS REGISTER >> 14 addrs
		r_UaRms,			r_UbRms,			r_UcRms,			r_UtRms,
		r_IaRms,			r_IbRms,			r_IcRms,			r_ItRms,
		r_LineUaRrms,		r_LineUbRrms, 		r_LineUcRrms,
		r_LineIaRrms, 		r_LineIbRrms,		r_LineIcRrms,
		// POWER FACTOR REGISTER >> 4 addrs
		r_Pfa,				r_Pfb,				r_Pfc,				r_Pft,
		// ENERGY REGISTER >> 8 addrs
		r_Epa,				r_Epb, 				r_Epc,				r_Ept,
		r_Eqa,				r_Eqb,				r_Eqc,				r_Eqt
		// TOTAL REGISTER >> 46
};
uint8_t sizeSensor = sizeof(addrSensor)/sizeof(addrSensor[0]);
uint32_t valueSensor[46];

// POWER REGISTER
float 	powerActiveA,		powerActiveB,		powerActiveC,		PowerActiveCombine,
		powerReactiveA,		powerReactiveB,		powerReactiveC,		powerReactiveCombine,
		powerApparentA,		powerApparentB,		powerApparentC,		powerApparentCombine,
		powerActiveWaveA,	powerActiveWaveB,	powerActiveWaveC,	powerActiveWaveSum,
		powerReactiveWaveA,	powerReactiveWaveB,	powerReactiveWaveC,	powerReactiveWaveCombine;

// RMS REGISTER
float 	rmsVoltageA,		rmsVoltageB,		rmsVoltageC,		rmsVoltageVector,
		rmsCurrentA,		rmsCurrentB,		rmsCurrentC,		rmsCurrentVector,
		rmsVoltageWaveA,	rmsVoltageWaveB,	rmsVoltageWaveC,
		rmsCurrentWaveA,	rmsCurrentWaveB,	rmsCurrentWaveC;

// POWER FACTOR REGISTER
float 	powerFactorA,		powerFactorB,		powerFactorC, 		powerFactorCombine;

// ENERGY REGISTER
float 	energyActiveA,		energyActiveB, 		energyActiveC,		energyActiveCombine,
		energyReactiveA,	energyReactiveB, 	energyReactiveC, 	energyReactiveCombine;

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
	spiCommandSpecial(w_calib, BYTE_ENABLE);
	spiCommandSpecial(w_read_calib, BYTE_ENABLE);

	// WRTIE CALIBRATION PARAMETER BASED ON ATRIBUTE
	for(int indeks=0;indeks<numberCalib;indeks++){
		spiWriteCalib(address[indeks], dataSet[indeks]);
		check = spiReadCalib(address[indeks]);
		// CHECK VALUE AFTER WRIING PARAMETER REGISTER
		if(check == dataSet[indeks])dataStatus[indeks] = HAL_OK;
		else dataStatus[indeks] = HAL_ERROR;
	}
	// DISBALE CALIBRATION MODE & DISABLE READ CALIRATION MODE
	spiCommandSpecial(w_read_calib, BYTE_DISABLE);
	spiCommandSpecial(w_calib, BYTE_DISABLE);
	HAL_Delay(75);
}

uint32_t uint24ToInt24(uint32_t data){
	testing
}
