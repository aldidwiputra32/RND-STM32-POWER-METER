#include "buttonInterface.h"

uint32_t bufferCalibIndeks = 0;
uint16_t buttonStatus;
uint8_t buttonTrigger = 0;
uint8_t menuLevel = MENU_LEVEL_0;
uint8_t menuParam = MENU_CURRENT;
uint8_t bufferCalib[4];

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

void menuLoop(){
	// STATE MACHINE PROCESSING
	// MODE SAVE SETTING
	if(buttonTrigger){
		if(flagGetDataOld){
			// GET DATA FROM EEPROM DATA
			buttonVoltageOffset = (float)offsetVolt_stm32/1000;
			buttonCurrentOffset = (float)offsetCurr_stm32/1000;
			buttonVoltageGain = (float)gainVolt_stm32/1000;
			buttonCurrentGain = (float)gainCurr_stm32/1000;

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
					// WIRING TYPE
					powerWiringType = (uint16_t)buttonWiringType;

					// GROUP CALIBRATION POWER
					offsetVolt_stm32 = (uint16_t)(buttonVoltageOffset*1000);
					offsetCurr_stm32 = (uint16_t)(buttonCurrentOffset*1000);
					gainVolt_stm32 = (uint16_t)(buttonVoltageGain*1000);
					gainCurr_stm32 = (uint16_t)(buttonCurrentGain*1000);

					// GROUP MODBUS SLAVE ID
					Modbus.slaveAddrSlaveSecond = buttonSlaveID;

					// GROUP ENERGY ACTIVE REACTIVE
					if(buttonEnergyActive == RESET){
						bufferEnergySUM[0] = bufferEnergySUM[1] = bufferEnergySUM[2] = bufferEnergySUM[3] = 0.00;
						energyActiveA_uint = energyActiveB_uint = energyActiveC_uint = energyActiveCombine_uint = 0;
					}
					if(buttonEnergyReactive == RESET){
						bufferEnergySUM[4] = bufferEnergySUM[5] = bufferEnergySUM[6] = bufferEnergySUM[7] = 0.00;
						energyReactiveA_uint = energyReactiveB_uint = energyReactiveC_uint = energyReactiveCombine_uint = 0;
					}
					menuLevel = MENU_LEVEL_0;
					// TRIGGER FOR SAVING DATA TO EEPROM
					stateConfig = 1;
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
				paramLv1 = 0;
				menuLevel = MENU_LEVEL_3;
				buttonStatus = BTN_IDLE;
			// ACTION FOR SAVING DATA
			}else if(buttonStatus == BTN_ENTER){
				menuLevel = MENU_LEVEL_SAVE;
				buttonStatus = BTN_IDLE;
			}
		// ---------------------------------------STATE SETTING LEVEL 3---------------------------------------
		}else if(menuLevel == MENU_LEVEL_3){
			// GAIN CURRENT
			if(paramLv1 == MENU_CURRENT){
				// REFRESH DATA OLD
				if(flagdataOld){
					floatTodisplay(bufferCalib, buttonCurrentGain);
					flagdataOld = 0;
				}
				if(buttonStatus == BTN_UP){
					paramLv3++;
					handleTreshold(&paramLv3, 9, 0);
					bufferCalib[bufferCalibIndeks] = paramLv3;
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_NEXT){
					bufferCalib[bufferCalibIndeks] = paramLv3;
					bufferCalibIndeks++;
					handleTreshold(&bufferCalibIndeks, 3, 0);
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_ENTER){
					bufferCalibIndeks = 0;
					menuLevel = MENU_LEVEL_1;
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_SET){
					uint8_t state = 0;
					if(paramLv1 == MENU_CURRENT){
						state = convertRawBtnToFloat(&buttonCurrentGain, bufferCalib, 65.535);
					}
					if(state){
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
				// REFRESH DATA OLD
				if(flagdataOld){
					uint8_t bufferOld = Modbus.slaveAddrSlaveSecond;
					bufferCalib[2] = bufferOld%10; bufferOld/=10;
					bufferCalib[1] = bufferOld%10; bufferOld/=10;
					bufferCalib[0] = bufferOld%10; bufferOld/=10;
					flagdataOld = 0;
				}
				if(buttonStatus == BTN_UP){
					paramLv3++;
					handleTreshold(&paramLv3, 9, 0);
					bufferCalib[bufferCalibIndeks] = paramLv3;
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_NEXT){
					bufferCalib[bufferCalibIndeks] = paramLv3;
					bufferCalibIndeks++;
					handleTreshold(&bufferCalibIndeks, 2, 0);
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
				}
			// RESET ENERGY
			}else if(paramLv1 == MENU_ENERGY){
				if(buttonStatus == BTN_UP){
					paramLv3++;
					handleTreshold(&paramLv3, 1, 0); // 0: RESET, 1: NON-RESET
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_ENTER){
					menuLevel = MENU_LEVEL_2;
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_SET){
					// SAVE DATA SLAVE TO VARIABLE VOLATILE
//					if(paramLv2 == SUBMENU_E_ACTIVE)buttonEnergyActive = paramLv3;
//					if(paramLv2 == SUBMENU_E_REACTIVE)buttonEnergyReactive = paramLv3;

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
	*buffer = ((float)data[0]*10) + ((float)data[1]) + ((float)data[2]/10) + ((float)data[3]/100);
	if(*buffer > (float)max){
		return 0;
	}else{
		return 1;
	}
}

void floatTodisplay(uint8_t * bufferDisplay, float dataFloat){
	uint16_t data16 = (uint16_t)(dataFloat*100);
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
		if((paramLv1 == DISPLAY_CURRENT_RMS)||(paramLv1 == DISPLAY_VOLTAGE_RMS)||(paramLv1 == DISPLAY_APPARENT_POWER)||(paramLv1 == DISPLAY_VOLTAGE_RMS_DIV)||(paramLv1 == DISPLAY_ACTIVE_POWER)||(paramLv1 == DISPLAY_POWER_FACTOR)){
			ht1622UpdateRamFloat(ACTIVE_ENERGY, NINE_DIGIT, NONE, energyActiveCombine);
		}else if(paramLv1 == DISPLAY_REACTIVE_POWER){
			ht1622UpdateRamFloat(REACTIVE_ENERGY, NINE_DIGIT, NONE, energyReactiveCombine);
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
		ht1622UpdateRamChar(NONE, FOUR_DIGIT, 1, dataPrintChar);
		if(paramLv1 == MENU_CURRENT){
			dataPrintChar[0]='g';dataPrintChar[1]='.';dataPrintChar[2]='c';dataPrintChar[3]='r';dataPrintChar[4]='t';		// G.Crt
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 2, dataPrintChar);
		}else if(paramLv1 == MENU_MODBUS){
			dataPrintChar[0]='s';dataPrintChar[1]='.';dataPrintChar[2]='a';dataPrintChar[3]='d';dataPrintChar[4]='r';		// s.adr
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 2, dataPrintChar);
		}else if(paramLv1 == MENU_ENERGY){
			dataPrintChar[0]='e';dataPrintChar[1]='.';dataPrintChar[2]='c';dataPrintChar[3]='l';dataPrintChar[4]='r';		// e.clr
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 2, dataPrintChar);
		}
	// ---------------------------------------STATE SETTING LEVEL 3---------------------------------------
	}else if(menuLevel == MENU_LEVEL_3) {
		dataPrintChar[0]='s';dataPrintChar[1]='e';dataPrintChar[2]='t';dataPrintChar[3]='t';								// sett
		ht1622UpdateRamChar(NONE, FOUR_DIGIT, 1, dataPrintChar);
		// DISPLAY SET VALUE CURRENT
		if(paramLv1 == MENU_CURRENT){
			dataPrintChar[0]='g';dataPrintChar[1]='.';dataPrintChar[2]='c';dataPrintChar[3]='r';dataPrintChar[4]='t';		// G.Crt
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 2, dataPrintChar);
			// DISPLAY SETTING DATA BLINKING CURSOR
//			if(stateCursor == 1){
//				if(bufferCalibIndeks!=0)dataPrintChar[0]=bufferCalib[0]+'0';
//				if(bufferCalibIndeks!=1)dataPrintChar[1]=bufferCalib[1] + '0';
//				if(bufferCalibIndeks!=3)dataPrintChar[3]=bufferCalib[2] + '0';
//				if(bufferCalibIndeks!=4)dataPrintChar[4]=bufferCalib[3] + '0';
//			}else if(stateCursor == -2){
				dataPrintChar[0]=bufferCalib[0] + '0';
				dataPrintChar[1]=bufferCalib[1] + '0';
				dataPrintChar[3]=bufferCalib[2] + '0';
				dataPrintChar[4]=bufferCalib[3] + '0';
//			}
			dataPrintChar[2]='.';
			stateCursor = ~stateCursor;
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 3, dataPrintChar);

		// DISPLAY SET VALUE MODBUS
		}else if(paramLv1 ==MENU_MODBUS){
			dataPrintChar[0]='s';dataPrintChar[1]='.';dataPrintChar[2]='a';dataPrintChar[3]='d';dataPrintChar[4]='r';		// e.clr
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 2, dataPrintChar);
			// DISPLAY SETTING DATA BLINKING CURSOR
//			if(stateCursor){
//				if(bufferCalibIndeks!=0)dataPrintChar[0]=bufferCalib[0]+'0';
//				if(bufferCalibIndeks!=1)dataPrintChar[1]=bufferCalib[1] + '0';
//				if(bufferCalibIndeks!=3)dataPrintChar[3]=bufferCalib[2] + '0';
//				if(bufferCalibIndeks!=4)dataPrintChar[4]=bufferCalib[3] + '0';
//			}else{
				dataPrintChar[0]=bufferCalib[0] + '0';
				dataPrintChar[1]=bufferCalib[1] + '0';
				dataPrintChar[3]=bufferCalib[2] + '0';
				dataPrintChar[4]=bufferCalib[3] + '0';
//			}
			dataPrintChar[2]='.';
			stateCursor = ~stateCursor;
			ht1622UpdateRamChar(NONE, FOUR_DIGIT,  3, dataPrintChar);
		// DISPLAY SET VALUE ENERGY
		}else if(paramLv1 == MENU_ENERGY){
			dataPrintChar[0]='e';dataPrintChar[1]='.';dataPrintChar[2]='c';dataPrintChar[3]='l';dataPrintChar[4]='r';
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 2, dataPrintChar);
//			if(stateCursor == 1){
//				if(bufferCalibIndeks!=0)dataPrintChar[0]=bufferCalib[0]+'0';
//				if(bufferCalibIndeks!=1)dataPrintChar[1]=bufferCalib[1] + '0';
//				if(bufferCalibIndeks!=3)dataPrintChar[3]=bufferCalib[2] + '0';
//				if(bufferCalibIndeks!=4)dataPrintChar[4]=bufferCalib[3] + '0';
//			}else if(stateCursor == -2){
				dataPrintChar[0]=bufferCalib[0] + '0';
				dataPrintChar[1]=bufferCalib[1] + '0';
				dataPrintChar[3]=bufferCalib[2] + '0';
				dataPrintChar[4]=bufferCalib[3] + '0';
//			}
			dataPrintChar[2]='.';
			stateCursor = ~stateCursor;
			ht1622UpdateRamChar(NONE, FOUR_DIGIT, 3, dataPrintChar);
		}
	}else if(menuLevel == MENU_LEVEL_SAVE){
		if(paramLv3 == SAVE){
		}
		if(paramLv3 == BACK){
		}
		if(paramLv3 == CANCEL){
		}
	}
	ht1622Print();
}

//// WIRING TYPE
//					powerWiringType = (uint16_t)buttonWiringType;
//
//					// GROUP CALIBRATION POWER
//					offsetVolt_stm32 = buttonVoltageOffset;
//					offsetCurr_stm32 = buttonCurrentOffset;
//					gainVolt_stm32 = buttonVoltageGain;
//					gainCurr_stm32 = buttonCurrentGain;
//
//					// GROUP MODBUS SLAVE ID
//					Modbus.slaveAddrSlaveSecond = buttonSlaveID;

//uint8_t buttonStatus = 0;
//uint8_t buttonAntiBounce = 0;


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){
//	if(htim == &htim14){
//		buttonAntiBounce = 0;
//		HAL_TIM_Base_Stop(&htim14);
//	}
//}
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIOPin){
//	// ANTI BOUNCING
//	if(buttonAntiBounce){
//		return;
//	}
//
//	if((GPIOPin == GPIO_PIN_6) || (GPIOPin == GPIO_PIN_4) || (GPIOPin == GPIO_PIN_3) || (GPIOPin == GPIO_PIN_15)){
//		buttonAntiBounce = 1;
//		HAL_TIM_Base_Start(&htim14);
//		// BUTTON NEXT " > "
//		if(GPIOPin == GPIO_PIN_6){buttonStatus = BTN_NEXT;}
//		// BUTTON UP " ^ "
//		else if(GPIOPin == GPIO_PIN_4){buttonStatus = BTN_UP;}
//		// BUTTON SET
//		else if(GPIOPin == GPIO_PIN_3){buttonStatus = BTN_SET;}
//		// BUTTON ENTER =
//		else if(GPIOPin == GPIO_PIN_15){buttonStatus = BTN_ENTER;}
//	}
//}
