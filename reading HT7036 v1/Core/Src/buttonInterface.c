#include "buttonInterface.h"

uint32_t bufferCalibIndeks = 0;
uint16_t buttonStatus;
uint8_t buttonTrigger = 0;
uint8_t menuLevel = MENU_LEVEL_0;
uint8_t menuParam = MENU_CURRENT;
uint8_t bufferCalib[4];
uint8_t stateConfigButton = 0;

static uint8_t flagdataOld = 1;
uint8_t flagGetDataOld = 0;
uint8_t	buttonSlaveID;
uint8_t	buttonWiringType;
uint8_t	buttonEnergyActive = 1;
uint8_t	buttonEnergyReactive = 1;
uint8_t stateCursor = 1;
uint32_t paramLv1, 				paramLv2, 				paramLv3;
float 	buttonVoltageOffset,	buttonVoltageGain, 		buttonCurrentOffset, 	buttonCurrentGain;

extern uint16_t offsetVolt_stm32,		offsetCurr_stm32,		gainVolt_stm32,			gainCurr_stm32;
extern uint64_t	energyActiveA_uint,		energyActiveB_uint,		energyActiveC_uint,		energyActiveCombine_uint,
				energyReactiveA_uint,	energyReactiveB_uint, 	energyReactiveC_uint,	energyReactiveCombine_uint;
extern uint8_t 	stateConfig;
extern MODBUS 	Modbus;
extern uint16_t	powerWiringType;
extern double bufferEnergySUM[8];

extern float 	powerActiveA,		powerActiveB,		powerActiveC,		PowerActiveCombine,   // V x i x cos phi
				powerReactiveA,		powerReactiveB,		powerReactiveC,		powerReactiveCombine, // V x i x sin phi
				powerApparentA,		powerApparentB,		powerApparentC,		powerApparentCombine; // V x i
// RMS REGISTER
extern float 	rmsVoltageA,		rmsVoltageB,		rmsVoltageC,		rmsVoltageVector,
				rmsCurrentA,		rmsCurrentB,		rmsCurrentC,		rmsCurrentVector;
// POWER FACTOR REGISTER
extern float 	powerFactorA,		powerFactorB,		powerFactorC, 		powerFactorCombine;

// ENERGY REGISTER
extern float 	energyActiveA,		energyActiveB, 		energyActiveC,		energyActiveCombine,
				energyReactiveA,	energyReactiveB, 	energyReactiveC,	energyReactiveCombine;

extern float 	rmsVoltageAB,		rmsVoltageBC,		rmsVoltageCA;
extern float	gainCurrent;

void menuLoop(){
	// STATE MACHINE PROCESSING
	// MODE SAVE SETTING
	if(buttonTrigger){
		if(flagGetDataOld){
			// GET DATA FROM EEPROM DATA
			buttonVoltageOffset = (float)offsetVolt_stm32/1000;
			buttonCurrentOffset = (float)offsetCurr_stm32/1000;
			buttonVoltageGain = (float)gainVolt_stm32/1000;
			buttonCurrentGain = (float)gainCurr_stm32;

			buttonSlaveID = Modbus.slaveAddrSlaveSecond;

			buttonWiringType = (uint8_t)powerWiringType;

			flagGetDataOld = 0;
		}

		// ---------------------------------------STATE SETTING SAVE DATA-------------------------------------
		if(menuLevel == MENU_LEVEL_0){
			if(buttonStatus == BTN_NEXT){
				paramLv1++;
				if(paramLv1 > 6)paramLv1 = 0;
			}
		}
		if(menuLevel == MENU_LEVEL_SAVE){
			if(buttonStatus == BTN_UP){
				paramLv3++;
				handleTreshold(&paramLv3, 2, 0);   // 0: CANCEL, 1: BACK, 2: SAVE
			}else if(buttonStatus == BTN_SET){
				// ACTION SAVE SETTING
				if(paramLv3 == SAVE){
					// GROUP CALIBRATION POWER >> GIAN CURRENT
					gainCurr_stm32 = (uint16_t)(buttonCurrentGain);
					gainCurrent = buttonCurrentGain;
					// GROUP MODBUS SLAVE ID >> MIDBUS ID
					Modbus.slaveAddrSlaveSecond = buttonSlaveID;
					// GROUP ENERGY ACTIVE REACTIVE
					if(buttonEnergyActive == RESET){
						bufferEnergySUM[0] = bufferEnergySUM[1] = bufferEnergySUM[2] = bufferEnergySUM[3] = 0.00;
						energyActiveA_uint = energyActiveB_uint = energyActiveC_uint = energyActiveCombine_uint = 0;
						buttonEnergyActive = NON_RESET;
					}
					if(buttonEnergyReactive == RESET){
						bufferEnergySUM[4] = bufferEnergySUM[5] = bufferEnergySUM[6] = bufferEnergySUM[7] = 0.00;
						energyReactiveA_uint = energyReactiveB_uint = energyReactiveC_uint = energyReactiveCombine_uint = 0;
						buttonEnergyReactive = NON_RESET;
					}
					menuLevel = MENU_LEVEL_0;
					// TRIGGER FOR SAVING DATA TO EEPROM
					stateConfig = 1;
					stateConfigButton = 1;
				// ACTION CANCEL SETTING
				}else if(paramLv3 == BACK){
					stateConfig = 0;
					menuLevel = MENU_LEVEL_1;
				// ACTIO BACK SETTING
				}else if(paramLv3 == CANCEL){
					stateConfig = 0;
					menuLevel = MENU_LEVEL_0;
				}
				paramLv3 = 0;
			}
		}
		// ---------------------------------------STATE SETTING LEVEL 1---------------------------------------
		else if(menuLevel == MENU_LEVEL_1){
			// BUTTON MANAGEMENT BEGIN
			if(buttonStatus == BTN_UP){
				paramLv1++;
				handleTreshold(&paramLv1, 2, 0);
				buttonStatus = BTN_IDLE;
			}else if(buttonStatus == BTN_NEXT){
				menuLevel = MENU_LEVEL_3;
				buttonStatus = BTN_IDLE;
				// REFRESH DATA OLD FROM EEPROM
				if(paramLv1 == MENU_CURRENT){
					if(flagdataOld){
						floatTodisplay(bufferCalib, buttonCurrentGain);
						flagdataOld = 0;
						bufferCalibIndeks = 0;
						// GET DATASET FROM EEPROM DATA TO BUFFER CALIB >> JUST ON COLUMN
						paramLv3 = bufferCalib[bufferCalibIndeks];
					}
				}else if(paramLv1 == MENU_MODBUS){
					if(flagdataOld){
						uint8_t bufferOld = buttonSlaveID;
						bufferCalib[2] = bufferOld%10; bufferOld/=10;
						bufferCalib[1] = bufferOld%10; bufferOld/=10;
						bufferCalib[0] = bufferOld%10; bufferOld/=10;
						flagdataOld = 0;
						bufferCalibIndeks = 0;
						// GET DATASET FROM EEPROM DATA TO BUFFER CALIB >> JUST ON COLUMN
						paramLv3 = bufferCalib[bufferCalibIndeks];
					}
				}else if(paramLv1 == MENU_ENERGY){
					if(flagdataOld){
						paramLv3 = (uint8_t)buttonEnergyActive && (uint8_t)buttonEnergyReactive;
					}
				}
			// ACTION FOR SAVING DATA
			}else if(buttonStatus == BTN_ENTER){
				menuLevel = MENU_LEVEL_SAVE;
				buttonStatus = BTN_IDLE;
			}
		// ---------------------------------------STATE SETTING LEVEL 3---------------------------------------
		}else if(menuLevel == MENU_LEVEL_3){
			// GAIN CURRENT
			if(paramLv1 == MENU_CURRENT){
				if(buttonStatus == BTN_UP){
					paramLv3++;
					handleTreshold(&paramLv3, 9, 0);
					bufferCalib[bufferCalibIndeks] = paramLv3;
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_NEXT){
					// GET DATASET FROM PARAMLV3 TO BUFFERCALIB
					bufferCalib[bufferCalibIndeks] = paramLv3;
					// SWITCH TO THE NEXT COLUMN
					bufferCalibIndeks++;
					handleTreshold(&bufferCalibIndeks, 3, 0);
					// GET DATASET FROM EEPROM DATA TO BUFFER CALIB
					paramLv3 = bufferCalib[bufferCalibIndeks];
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_ENTER){
					bufferCalibIndeks = 0;
					menuLevel = MENU_LEVEL_1;
					buttonStatus = BTN_IDLE;
					flagdataOld = 1;
				}else if(buttonStatus == BTN_SET){
					uint8_t state = 0;
					if(convertRawBtnToFloat(&buttonCurrentGain, bufferCalib, 65535)){
						menuLevel = MENU_LEVEL_1;
						bufferCalibIndeks = 0;
						flagdataOld = 1;
						memset(bufferCalib, 0, sizeof(bufferCalib));
					}else{
						menuLevel = MENU_LEVEL_3;
					}
					buttonStatus = BTN_IDLE;
					paramLv3 = 0;
				}
			}
			// MODBUS: SLAVE ID
			else if(paramLv1 == MENU_MODBUS){
				if(buttonStatus == BTN_UP){
					paramLv3++;
					handleTreshold(&paramLv3, 9, 0);
					bufferCalib[bufferCalibIndeks] = paramLv3;
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_NEXT){
					bufferCalib[bufferCalibIndeks] = paramLv3;
					bufferCalibIndeks++;
					handleTreshold(&bufferCalibIndeks, 2, 0);
					// GET DATASET FROM EEPROM DATA TO BUFFER CALIB
					paramLv3 = bufferCalib[bufferCalibIndeks];
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_SET){
					// SAVE DATA SLAVE TO VARIABLE VOLATILE
					uint16_t buffer16;
					buffer16 = (bufferCalib[0]*100) + (bufferCalib[1]*10) + (bufferCalib[2]);
					if(buffer16 < 0xFF){
						buttonSlaveID = (uint8_t)buffer16;
						menuLevel = MENU_LEVEL_1;
						bufferCalibIndeks = 0;
						flagdataOld = 1;
						memset(bufferCalib, 0, sizeof(bufferCalib));
					}else{
						menuLevel = MENU_LEVEL_3;
					}
					paramLv3 = 0;
				}else if(buttonStatus == BTN_ENTER){
					menuLevel = MENU_LEVEL_1;
					buttonStatus = BTN_IDLE;
					flagdataOld = 1;
				}
			// RESET ENERGY
			}else if(paramLv1 == MENU_ENERGY){
				if(buttonStatus == BTN_UP){
					paramLv3++;
					handleTreshold(&paramLv3, 1, 0); // 0: RESET, 1: NON-RESET
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_ENTER){
					menuLevel = MENU_LEVEL_1;
					flagdataOld = 1;
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_SET){
					// SAVE DATA SLAVE TO VARIABLE VOLATILE
					buttonEnergyActive = (uint8_t)paramLv3;
					buttonEnergyReactive = (uint8_t)paramLv3;
					flagdataOld = 1;
					buttonStatus = BTN_IDLE;
					menuLevel = MENU_LEVEL_1;
					paramLv3 = 0;
				}
			}
		}
		buttonTrigger = 0;
	}
	displayLoop(menuLevel,paramLv1,paramLv2,paramLv3);
	// LEVEL DISPLAY
}

uint8_t convertRawBtnToFloat(float * buffer, uint8_t * data, float max){
	*buffer = ((float)data[0]*1000) + ((float)data[1]*100) + ((float)data[2]*10) + ((float)data[3]);
	if(*buffer > (float)max){
		return 0;
	}else{
		return 1;
	}
}

void floatTodisplay(uint8_t * bufferDisplay, float dataFloat){
	uint16_t data16 = (uint16_t)(dataFloat);
	bufferDisplay[3] = data16%10; data16 /= 10;
	bufferDisplay[2] = data16%10; data16 /= 10;
	bufferDisplay[1] = data16%10; data16 /= 10;
	bufferDisplay[0] = data16%10; data16 /= 10;
}

void handleTreshold(uint32_t * val, uint8_t max, uint8_t min){
	if(*val < min)*val = min;
	if(*val > max)*val = min;
}

void displayLoop(){
	uint8_t dataPrintChar[8];
	// ---------------------------------------STATE DISPLAY ----------------------------------------------
	ht1622ClearSegment();
	if(menuLevel == MENU_LEVEL_0){
		float energyBuffer;
		if((paramLv1 == DISPLAY_CURRENT_RMS)||(paramLv1 == DISPLAY_VOLTAGE_RMS)||(paramLv1 == DISPLAY_APPARENT_POWER)||(paramLv1 == DISPLAY_VOLTAGE_RMS_DIV)||(paramLv1 == DISPLAY_ACTIVE_POWER)||(paramLv1 == DISPLAY_POWER_FACTOR)){
			energyBuffer = (float)(energyActiveA_uint + energyActiveB_uint + energyActiveC_uint)/1000;
			ht1622UpdateRamFloat(ACTIVE_ENERGY, NINE_DIGIT, NONE, energyBuffer);
		}else if(paramLv1 == DISPLAY_REACTIVE_POWER){
			energyBuffer = (float)(energyReactiveA_uint + energyReactiveB_uint + energyReactiveC_uint)/1000;
			ht1622UpdateRamFloat(REACTIVE_ENERGY, NINE_DIGIT, NONE, energyBuffer);
		}
		if(paramLv1 == DISPLAY_CURRENT_RMS){
			ht1622UpdateRamFloat(CURRENT_RMS, FOUR_DIGIT, 1, rmsCurrentA);
			ht1622UpdateRamFloat(CURRENT_RMS, FOUR_DIGIT, 2, rmsCurrentB);
			ht1622UpdateRamFloat(CURRENT_RMS, FOUR_DIGIT, 3, rmsCurrentC);
		}else if(paramLv1 == DISPLAY_VOLTAGE_RMS){
			ht1622UpdateRamFloat(VOLTAGE_RMS, FOUR_DIGIT, 1, rmsVoltageA);
			ht1622UpdateRamFloat(VOLTAGE_RMS, FOUR_DIGIT, 2, rmsVoltageB);
			ht1622UpdateRamFloat(VOLTAGE_RMS, FOUR_DIGIT, 3, rmsVoltageC);
		}else if(paramLv1 == DISPLAY_VOLTAGE_RMS_DIV){
			ht1622UpdateRamFloat(VOLTAGE_RMS_DIV, FOUR_DIGIT, 1, rmsVoltageAB);
			ht1622UpdateRamFloat(VOLTAGE_RMS_DIV, FOUR_DIGIT, 2, rmsVoltageBC);
			ht1622UpdateRamFloat(VOLTAGE_RMS_DIV, FOUR_DIGIT, 3, rmsVoltageCA);
		}else if(paramLv1 == DISPLAY_ACTIVE_POWER){
			ht1622UpdateRamFloat(ACTIVE_POWER, FOUR_DIGIT, 1, powerActiveA);
			ht1622UpdateRamFloat(ACTIVE_POWER, FOUR_DIGIT, 2, powerActiveB);
			ht1622UpdateRamFloat(ACTIVE_POWER, FOUR_DIGIT, 3, powerActiveC);
		}else if(paramLv1 == DISPLAY_REACTIVE_POWER){
			ht1622UpdateRamFloat(REACTIVE_POWER, FOUR_DIGIT, 1, powerReactiveA);
			ht1622UpdateRamFloat(REACTIVE_POWER, FOUR_DIGIT, 2, powerReactiveB);
			ht1622UpdateRamFloat(REACTIVE_POWER, FOUR_DIGIT, 3, powerReactiveC);
		}else if(paramLv1 == DISPLAY_APPARENT_POWER){
			ht1622UpdateRamFloat(APPARENT_POWER, FOUR_DIGIT, 1, powerApparentA);
			ht1622UpdateRamFloat(APPARENT_POWER, FOUR_DIGIT, 2, powerApparentB);
			ht1622UpdateRamFloat(APPARENT_POWER, FOUR_DIGIT, 3, powerApparentC);
		}else if(paramLv1 == DISPLAY_POWER_FACTOR){
			ht1622UpdateRamFloat(POWER_FACTOR, FOUR_DIGIT, 1, powerFactorA);
			ht1622UpdateRamFloat(POWER_FACTOR, FOUR_DIGIT, 2, powerFactorB);
			ht1622UpdateRamFloat(POWER_FACTOR, FOUR_DIGIT, 3, powerFactorC);
		}
	}
	// ---------------------------------------STATE SETTING LEVEL 1---------------------------------------
	if(menuLevel == MENU_LEVEL_1){
		dataPrintChar[0]='s';dataPrintChar[1]='e';dataPrintChar[2]='t';dataPrintChar[3]='t';								// sett
		ht1622UpdateRamChar(NONE, FOUR_DIGIT, 1, dataPrintChar,4);
		if(paramLv1 == MENU_CURRENT){
			dataPrintChar[0]='g';dataPrintChar[1]='a';dataPrintChar[2]='i';dataPrintChar[3]='n';							// G.Crt
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 2, dataPrintChar,4);
		}else if(paramLv1 == MENU_MODBUS){
			dataPrintChar[0]='s';dataPrintChar[1]='.';dataPrintChar[2]='a';dataPrintChar[3]='d';dataPrintChar[4]='r';		// s.adr
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 2, dataPrintChar,5);
		}else if(paramLv1 == MENU_ENERGY){
			dataPrintChar[0]='e';dataPrintChar[1]='.';dataPrintChar[2]='c';dataPrintChar[3]='l';dataPrintChar[4]='r';		// e.clr
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 2, dataPrintChar,5);
		}
	// ---------------------------------------STATE SETTING LEVEL 3---------------------------------------
	}else if(menuLevel == MENU_LEVEL_3) {
		dataPrintChar[0]='s';dataPrintChar[1]='e';dataPrintChar[2]='t';dataPrintChar[3]='t';								// sett
		ht1622UpdateRamChar(NONE, FOUR_DIGIT, 1, dataPrintChar,4);
		// DISPLAY SET VALUE CURRENT
		if(paramLv1 == MENU_CURRENT){
			dataPrintChar[0]='g';dataPrintChar[1]='a';dataPrintChar[2]='i';dataPrintChar[3]='n';		// G.Crt
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 2, dataPrintChar,4);
			// DISPLAY SETTING DATA BLINKING CURSOR
			if((bufferCalibIndeks==0) && (stateCursor==1)){dataPrintChar[0]='/';}
			else{dataPrintChar[0]=bufferCalib[0] + '0';}
			if((bufferCalibIndeks==1) && (stateCursor==1)){dataPrintChar[1]='/';}
			else{dataPrintChar[1]=bufferCalib[1] + '0';}
			if((bufferCalibIndeks==2) && (stateCursor==1)){dataPrintChar[2]='/';}
			else{dataPrintChar[2]=bufferCalib[2] + '0';}
			if((bufferCalibIndeks==3) && (stateCursor==1)){dataPrintChar[3]='/';}
			else{dataPrintChar[3]=bufferCalib[3] + '0';}
			stateCursor = ~stateCursor;
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 3, dataPrintChar,4);

		// DISPLAY SET VALUE MODBUS
		}else if(paramLv1 == MENU_MODBUS){
			// DISPLAY SETTING DATA BLINKING CURSOR
			if((bufferCalibIndeks==0) && (stateCursor==1)){dataPrintChar[0]='/';}
			else{dataPrintChar[0]=bufferCalib[0] + '0';}
			if((bufferCalibIndeks==1) && (stateCursor==1)){dataPrintChar[1]='/';}
			else{dataPrintChar[1]=bufferCalib[1] + '0';}
			if((bufferCalibIndeks==2) && (stateCursor==1)){dataPrintChar[2]='/';}
			else{dataPrintChar[2]=bufferCalib[2] + '0';}
			dataPrintChar[3] = '/';
			stateCursor = ~stateCursor;
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 3, dataPrintChar,4);
			dataPrintChar[0]='s';dataPrintChar[1]='.';dataPrintChar[2]='a';dataPrintChar[3]='d';dataPrintChar[4]='r';		// S.Adr
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 2, dataPrintChar,5);
		// DISPLAY SET VALUE ENERGY
		}else if(paramLv1 == MENU_ENERGY){
			dataPrintChar[0]='e';dataPrintChar[1]='.';dataPrintChar[2]='c';dataPrintChar[3]='l';dataPrintChar[4]='r';
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 2, dataPrintChar,5);
			if(paramLv3 == RESET){dataPrintChar[0]='y';dataPrintChar[1]='e';dataPrintChar[2]='s';dataPrintChar[3]='/';}
			else if(paramLv3 == NON_RESET){dataPrintChar[0]='n';dataPrintChar[1]='o';dataPrintChar[2]='/';dataPrintChar[3]='/';}
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 3, dataPrintChar,4);
		}
	}else if(menuLevel == MENU_LEVEL_SAVE){
		dataPrintChar[0]='s';dataPrintChar[1]='e';dataPrintChar[2]='t';dataPrintChar[3]='t';								// sett
		ht1622UpdateRamChar(NONE, FOUR_DIGIT, 1, dataPrintChar,4);
		dataPrintChar[0]='s';dataPrintChar[1]='a';dataPrintChar[2]='v';dataPrintChar[3]='e';
		ht1622UpdateRamChar(NONE, FOUR_DIGIT, 2, dataPrintChar,4);
		if(paramLv3 == SAVE){dataPrintChar[0]='y';dataPrintChar[1]='e';dataPrintChar[2]='s';dataPrintChar[3]='/';}
		if(paramLv3 == BACK){dataPrintChar[0]='b';dataPrintChar[1]='a';dataPrintChar[2]='c';dataPrintChar[3]='k';}
		if(paramLv3 == CANCEL){dataPrintChar[0]='c';dataPrintChar[1]='n';dataPrintChar[2]='c';dataPrintChar[3]='l';	}
		ht1622UpdateRamChar(NONE, FOUR_DIGIT, 3, dataPrintChar,4);
	}
	ht1622Print();
}
