#include "buttonInterface.h"

uint32_t bufferCalibIndeks = 0;
uint16_t buttonStatus;
uint8_t buttonTrigger = 0;
uint8_t menuLevel = MENU_LEVEL_0;
uint8_t menuParam = MENU_WIRING_TYPE;
uint8_t bufferCalib[4];

static uint8_t	flagdataOld = 1;
uint8_t 	flagGetDataOld = 0;
uint8_t 	buttonSlaveID;
uint8_t 	buttonWiringType;
uint8_t 	buttonEnergyActive = 1;
uint8_t 	buttonEnergyReactive = 1;
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
				handleTreshold(&paramLv1, 4, 0);
				buttonStatus = BTN_IDLE;
			}else if(buttonStatus == BTN_NEXT){
				if((paramLv1 == MENU_WIRING_TYPE) || (paramLv1 == MENU_MODBUS)){
					menuLevel = MENU_LEVEL_3;
				}else{
					menuLevel = MENU_LEVEL_2;
				}
				buttonStatus = BTN_IDLE;
			// ACTION FOR SAVING DATA
			}else if(buttonStatus == BTN_ENTER){
				menuLevel = MENU_LEVEL_SAVE;
				buttonStatus = BTN_IDLE;
			}
			// BUTTON MANAGEMENT END
		// ---------------------------------------STATE SETTING LEVEL 2---------------------------------------
		}else if(menuLevel == MENU_LEVEL_2){
			// BUTTON MANAGEMENT BEGIN
			if(buttonStatus == BTN_UP){
				paramLv2++;
				handleTreshold(&paramLv2, 1, 0);
				buttonStatus = BTN_IDLE;
			}else if(buttonStatus == BTN_NEXT){
				menuLevel = MENU_LEVEL_3;
				buttonStatus = BTN_IDLE;
			}else if(buttonStatus == BTN_ENTER){
				menuLevel = MENU_LEVEL_1;
				buttonStatus = BTN_IDLE;
			}
			// BUTTON MANAGEMENT END
		// ---------------------------------------STATE SETTING LEVEL 3---------------------------------------
		}else if(menuLevel == MENU_LEVEL_3){
			// WIRING TYPE
			if(paramLv1 == MENU_WIRING_TYPE){
				// REFRESH DATA OLD
				if(flagdataOld){
					paramLv3 = buttonWiringType;
					flagdataOld = 0;
				}
				if(buttonStatus == BTN_UP){
					paramLv3++;
					handleTreshold(&paramLv3, 1, 0);
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_ENTER){
					menuLevel = MENU_LEVEL_1;
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_SET){
					buttonWiringType = paramLv3;
					paramLv3 = 0;
					menuLevel = MENU_LEVEL_1;
					buttonStatus = BTN_IDLE;
					flagdataOld = 1;
				}
			}
			// VOLTAGE & CCURRENT CALIBRATION
			else if((paramLv1 == MENU_VOLTAGE) || (paramLv1 == MENU_CURRENT)){
				if((paramLv2 == SUBMENU_GAIN) || (paramLv2 == SUBMENU_OFFSET)){
					// REFRESH DATA OLD
					if(flagdataOld){
						if(paramLv1 == MENU_VOLTAGE){
							if(paramLv2 == SUBMENU_OFFSET)floatTodisplay(bufferCalib, buttonVoltageOffset);
							if(paramLv2 == SUBMENU_GAIN)floatTodisplay(bufferCalib, buttonVoltageGain);
						}else if(paramLv1 == MENU_CURRENT){
							if(paramLv2 == SUBMENU_OFFSET)floatTodisplay(bufferCalib, buttonCurrentOffset);
							if(paramLv2 == SUBMENU_GAIN)floatTodisplay(bufferCalib, buttonCurrentGain);
						}
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
						menuLevel = MENU_LEVEL_2;
						buttonStatus = BTN_IDLE;
					}else if(buttonStatus == BTN_SET){
						uint8_t state = 0;
						if(paramLv1 == MENU_VOLTAGE){
							if(paramLv2 == SUBMENU_OFFSET){state = convertRawBtnToFloat(&buttonVoltageOffset, bufferCalib, 65.535);}
							else if(paramLv2 == SUBMENU_GAIN){state = convertRawBtnToFloat(&buttonVoltageGain, bufferCalib, 65.535);}
						}else if(paramLv1 == MENU_CURRENT){
							if(paramLv2 == SUBMENU_OFFSET){state = convertRawBtnToFloat(&buttonCurrentOffset, bufferCalib, 65.535);}
							else if(paramLv2 == SUBMENU_GAIN){state = convertRawBtnToFloat(&buttonCurrentGain, bufferCalib, 65.535);}
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
					if(paramLv2 == SUBMENU_E_ACTIVE)buttonEnergyActive = paramLv3;
					if(paramLv2 == SUBMENU_E_REACTIVE)buttonEnergyReactive = paramLv3;

					buttonStatus = BTN_IDLE;
					menuLevel = MENU_LEVEL_1;
					paramLv3 = 0;
				}
			}
		}
		displayLoop(menuLevel,paramLv1,paramLv2,paramLv3);
		buttonTrigger = 0;
	}
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

void displayLoop(uint8_t level, uint8_t param1, uint8_t param2, uint8_t param3){
	// ---------------------------------------STATE DISPLAY ----------------------------------------------
	if(level == MENU_LEVEL_0){
		if((param1 == DISPLAY_CURRENT_RMS) || (param1 == DISPLAY_VOLTAGE_RMS)||(param1 == d))
		if(param1 == DISPLAY_CURRENT_RMS){
			ht1622UpdateRam(CURRENT_RMS, FOUR_DIGIT, 1, rmsCurrentA);
			ht1622UpdateRam(CURRENT_RMS, FOUR_DIGIT, 2, rmsCurrentB);
			ht1622UpdateRam(CURRENT_RMS, FOUR_DIGIT, 3, rmsCurrentC);
			ht1622UpdateRam(ACTIVE_ENERGY, NINE_DIGIT, 3, energyActiveCombine);
		}else if(param1 == DISPLAY_VOLTAGE_RMS){
			ht1622UpdateRam(CURRENT_RMS, FOUR_DIGIT, 1, rmsCurrentA);
			ht1622UpdateRam(CURRENT_RMS, FOUR_DIGIT, 2, rmsCurrentB);
			ht1622UpdateRam(CURRENT_RMS, FOUR_DIGIT, 3, rmsCurrentC);
			ht1622UpdateRam(ACTIVE_ENERGY, NINE_DIGIT, 3, energyActiveCombine);

		}
	}
	// ---------------------------------------STATE SETTING LEVEL 1---------------------------------------
	if(level == MENU_LEVEL_1){
//		serialPrint("\r\n----------MENU_LEVEL_1----------\r\n", 36);
		if(param1 == MENU_WIRING_TYPE){
//			serialPrint("WIRINGTYPE\r\n", 12);
		}else if(param1 == MENU_VOLTAGE){
//			serialPrint("CALIB VOLTAGE\r\n", 15);
		}else if(param1 == MENU_CURRENT){
//			serialPrint("CALIB CURRENT\r\n", 15);
		}else if(param1 == MENU_MODBUS){
//			serialPrint("MODBUS\r\n", 6);
		}else if(param1 == MENU_ENERGY){
//			serialPrint("ENERGY\r\n", 8);
		}
	// ---------------------------------------STATE SETTING LEVEL 2---------------------------------------
	}else if(level == MENU_LEVEL_2){
//		serialPrint("\r\n----------MENU_LEVEL_2----------\r\n", 36);
		// DISPLAY CALIBRATION VOLTAGE
		if(param1 == MENU_VOLTAGE){
			if(param2 == SUBMENU_OFFSET){
//				serialPrint("CALIB VOLTAGE: OFFSET\r\n", 23);
			}else if(param2 == SUBMENU_GAIN){
//				serialPrint("CALIB VOLTAGE: GAIN\r\n", 21);
			}
		}
		// DISPLAY CALIBRATION CURRENT
		else if(param1 == MENU_CURRENT){
			if(param2 == SUBMENU_OFFSET){
//				serialPrint("CALIB CURRENT: OFFSET\r\n", 23);
			}else if(param2 == SUBMENU_GAIN){
//				serialPrint("CALIB CURRENT: GAIN\r\n", 21);
			}
		}
		// DISPLAY RESET ENERGY
		else if(param1 == MENU_ENERGY){
			if(param2 == SUBMENU_E_ACTIVE){
//				serialPrint("ENERGY ACTIVE\r\n", 16);
			}else if(param2 == SUBMENU_E_REACTIVE){
//				serialPrint("ENERGY REACTIVE\r\n", 18);
			}
		}
	// ---------------------------------------STATE SETTING LEVEL 3---------------------------------------
	}else if(level == MENU_LEVEL_3){
//		uint8_t dataPrint[1100];
//		serialPrint("\r\n----------MENU_LEVEL_2----------\r\n", 36);
		// DISPLAY SET VALUE WIRING TYPE
		if(param1 == MENU_WIRING_TYPE){
			if(param3 == SUBMENU_N33){
//				serialPrint("WIRETYPE: N33\r\n", 15);
			}else if(param3 == SUBMENU_N34){
//				serialPrint("WIRETYPE: N34\r\n", 15);
			}
		// DISPLAY SET VALUE VOLTAGE
		}else if(param1 == MENU_VOLTAGE){
			if(param2 == SUBMENU_OFFSET){
//				sprintf(dataPrint,"\r\nCALIB VOLTAGE OFFSET CALC >> %d%d%d%d\r\n",bufferCalib[0],bufferCalib[1],bufferCalib[2],bufferCalib[3]);
//				serialPrint(dataPrint, 50);
			}else if(param2 == SUBMENU_GAIN){
//				sprintf(dataPrint,"\r\nCALIB VOLTAGE GAIN CALC >> %d%d%d%d\r\n",bufferCalib[0],bufferCalib[1],bufferCalib[2],bufferCalib[3]);
//				serialPrint(dataPrint, 50);
			}
		// DISPLAY SET VALUE CURRENT
		}else if(param1 == MENU_CURRENT){
			if(param2 == SUBMENU_OFFSET){
//				sprintf(dataPrint,"\r\nCALIB CURRENT OFFSET CALC >> %d%d%d%d\r\n",bufferCalib[0],bufferCalib[1],bufferCalib[2],bufferCalib[3]);
//				serialPrint(dataPrint, 50);
			}else if(param2 == SUBMENU_GAIN){
//				sprintf(dataPrint,"\r\nCALIB CURRENT GAIN CALC >> %d%d%d%d\r\n",bufferCalib[0],bufferCalib[1],bufferCalib[2],bufferCalib[3]);
//				serialPrint(dataPrint, 50);
			}
		// DISPLAY SET VALUE ENERGY
		}else if(param1 == MENU_ENERGY){
			if(param2 == SUBMENU_E_ACTIVE){
//				sprintf(dataPrint,"\r\nCALIB ENERGY CALC >> %d\r\n",paramLv3);
//				serialPrint(dataPrint, 50);
			}else if(param2 == SUBMENU_E_REACTIVE){
//				sprintf(dataPrint,"\r\nCALIB ENERGY CALC >> %d\r\n",paramLv3);
//				serialPrint(dataPrint, 50);
			}
		// DISPLAY SET VALUE MODBUS
		}else if(param1 == MENU_MODBUS){
//			sprintf(dataPrint,"\r\nCALIB MODBUS CALC >> %d%d%d\r\n",bufferCalib[0],bufferCalib[1],bufferCalib[2]);
//			serialPrint(dataPrint, 50);
		}
	}else if(level == MENU_LEVEL_SAVE){
//		uint8_t dataPrint[1100];
//		serialPrint("\r\n----------MENU_LEVEL_4----------\r\n", 36);
//		sprintf(dataPrint,"offsetVolt:%.2f, offsetCurr:%.2f, gainVolt:%.2f, gainCurr:%.2f, WiringType:%d, slaveAddr:%d",
//				buttonVoltageOffset,buttonCurrentOffset,buttonVoltageGain,buttonCurrentGain,buttonWiringType,buttonSlaveID
//		);
//		serialPrint(dataPrint, 700);
		if(paramLv3 == SAVE){
//			serialPrint("\r\nSAVE\r\n", 8);
		}
		if(paramLv3 == BACK){
//			serialPrint("\r\nBACK\r\n", 8);
		}
		if(paramLv3 == CANCEL){
//			serialPrint("\r\nCNCL\r\n", 8);
		}
	}
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
