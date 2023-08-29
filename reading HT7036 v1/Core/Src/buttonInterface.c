#include "buttonInterface.h"

uint16_t buttonStatus;
uint8_t buttonTrigger = 0;
uint8_t menuLevel = MENU_LEVEL_IDLE;
uint8_t menuParam = MENU_WIRING_TYPE;
uint8_t bufferCalib[4];
uint8_t bufferCalibIndeks = 0;
static uint32_t paramLv1, paramLv2, paramLv3;
static uint8_t buttonSlaveID;
static uint8_t buttonEnergyActive, buttonEnergyReactive;
static float buttonVoltageOffset, buttonVoltageGain, buttonCurrentOffset, buttonCurrentGain;

void menuLoop(){
	// STATE MACHINE PROCESSING
	if(buttonTrigger){
		// ---------------------------------------STATE SETTING LEVEL 1---------------------------------------
		if(menuLevel == MENU_LEVEL_1){
			// BUTTON MANAGEMENT BEGIN
			if(buttonStatus == BTN_UP){
				paramLv1++;
				handleTreshold(&paramLv1, 5, 0);
				buttonStatus = BTN_IDLE;
			}else if(buttonStatus == BTN_NEXT){
				if((paramLv1 == MENU_WIRING_TYPE) || (paramLv1 == MENU_MODBUS)){
					menuLevel = MENU_LEVEL_3;
				}else{
					menuLevel = MENU_LEVEL_2;
				}
				buttonStatus = BTN_IDLE;
			}else if(buttonStatus == BTN_ENTER){
				// ACTION SAVING & SYNCRONIZE DATA
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
			}
			// BUTTON MANAGEMENT END
		// ---------------------------------------STATE SETTING LEVEL 3---------------------------------------
		}else if(menuLevel == MENU_LEVEL_3){
			// WIRING TYPE
			if(paramLv1 == MENU_WIRING_TYPE){
				if(buttonStatus == BTN_UP){
					paramLv3++;
					handleTreshold(&paramLv3, 1, 0);
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_SET){
					menuLevel = MENU_LEVEL_1;
					buttonStatus = BTN_IDLE;
				}
			}
			// VOLTAGE & CCURRENT CALIBRATION
			else if((paramLv1 == MENU_VOLTAGE) || (paramLv1 == MENU_CURRENT)){
				if((paramLv2 == SUBMENU_GAIN) || (paramLv2 == SUBMENU_OFFSET)){
					if(buttonStatus == BTN_UP){
						paramLv3++;
						handleTreshold(&paramLv3, 9, 0);
						bufferCalib[bufferCalibIndeks] = paramLv3;
						buttonStatus = BTN_IDLE;
					}else if(buttonStatus == BTN_NEXT){
						bufferCalibIndeks++;
						handleTreshold(&bufferCalibIndeks, 3, 0);
						buttonStatus = BTN_IDLE;
					}else if(buttonStatus == BTN_SET){
						uint8_t state = 0;
						if(paramLv1 == MENU_VOLTAGE){
							if(paramLv2 == SUBMENU_OFFSET){state = convertRawBtnToFloat(&buttonVoltageOffset, bufferCalib, 0xFFF);}
							else if(paramLv2 == SUBMENU_GAIN){state = convertRawBtnToFloat(&buttonVoltageGain, bufferCalib, 0xFFF);}
						}else if(paramLv1 == MENU_CURRENT){
							if(paramLv2 == SUBMENU_OFFSET){state = convertRawBtnToFloat(&buttonCurrentOffset, bufferCalib, 0xFFF);}
							else if(paramLv2 == SUBMENU_GAIN){state = convertRawBtnToFloat(&buttonCurrentGain, bufferCalib, 0xFFF);}
						}
						if(state){
							menuLevel = MENU_LEVEL_1;
							bufferCalibIndeks = 0;
							memset(bufferCalib, 0, sizeof(bufferCalib));
						}
						else menuLevel = MENU_LEVEL_3;
						buttonStatus = BTN_IDLE;
					}
				}
				// CALIBRATION VOLTAGE GAIN & CURRENT
				if(paramLv1 == MENU_VOLTAGE){}
				// CALIBRATION CURRENT GAIN & CURRENT
				else if(paramLv1 == MENU_CURRENT){}
			}
			// MODBUS: SLAVE ID
			else if(paramLv1 == MENU_MODBUS){
				if(buttonStatus == BTN_UP){
					paramLv3++;
					handleTreshold(&paramLv3, 9, 0);
					bufferCalib[bufferCalibIndeks] = paramLv3;
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_NEXT){
					bufferCalibIndeks++;
					handleTreshold(&bufferCalibIndeks, 2, 0);
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_SET){
					// SAVE DATA SLAVE TO VARIABLE VOLATILE
					buttonSlaveID = (bufferCalib[0]*100) + (bufferCalib[1]*10) + (bufferCalib[2]);
					if(buttonSlaveID > 0xFF){
						menuLevel = MENU_LEVEL_1;
					}else{
						menuLevel = MENU_LEVEL_3;
					}
				}
			}else if(paramLv1 == MENU_ENERGY){
				if(buttonStatus == BTN_UP){
					paramLv3++;
					handleTreshold(&paramLv3, 1, 0);
					buttonStatus = BTN_IDLE;
				}else if(buttonStatus == BTN_SET){
					// SAVE DATA SLAVE TO VARIABLE VOLATILE
					if(paramLv2 == SUBMENU_E_ACTIVE)buttonEnergyActive = paramLv3;
					if(paramLv2 == SUBMENU_E_REACTIVE)buttonEnergyReactive = paramLv3;
					buttonStatus = BTN_IDLE;
					menuLevel = MENU_LEVEL_1;
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

void handleTreshold(uint8_t * val, uint8_t max, uint8_t min){
	if(*val < min)*val = min;
	if(*val > max)*val = min;
}

void displayLoop(uint8_t level, uint8_t param1, uint8_t param2, uint8_t param3){
	if(level == MENU_LEVEL_1){
		if(param1 == MENU_WIRING_TYPE){
			serialPrint("WIRINGTYPE\r\n", 12);
		}else if(param1 == MENU_VOLTAGE){
			serialPrint("CALIB VOLTAGE\r\n", 15);
		}else if(param1 == MENU_CURRENT){
			serialPrint("CALIB CURRENT\r\n", 15);
		}else if(param1 == MENU_MODBUS){
			serialPrint("MODBUS\r\n", 6);
		}else if(param1 == MENU_ENERGY){
			serialPrint("ENERGY\r\n", 8);
		}
	}else if(level == MENU_LEVEL_2){
		if(param1 == MENU_VOLTAGE){
			if(param2 == SUBMENU_OFFSET){
				serialPrint("CALIB VOLTAGE: OFFSET\r\n", 23);
			}else if(param2 == SUBMENU_GAIN){
				serialPrint("CALIB VOLTAGE: GAIN\r\n", 21);
			}
		}
		else if(param1 == MENU_CURRENT){
			if(param2 == SUBMENU_OFFSET){
				serialPrint("CALIB CURRENT: OFFSET\r\n", 23);
			}else if(param2 == SUBMENU_GAIN){
				serialPrint("CALIB CURRENT: GAIN\r\n", 21);
			}
		}
		else if(param1 == MENU_ENERGY){
			if(param2 == SUBMENU_E_ACTIVE){
				serialPrint("ENERGY ACTIVE\r\n", 16);
			}else if(param2 == SUBMENU_E_REACTIVE){
				serialPrint("ENERGY REACTIVE\r\n", 18);
			}
		}
	}else if(level == MENU_LEVEL_3){
		if(param1 == MENU_WIRING_TYPE){
			if(param3 == SUBMENU_N33){
				serialPrint("WIRETYPE: N33\r\n", 15);
			}else if(param3 = SUBMENU_N34){
				serialPrint("WIRETYPE: N34\r\n", 15);
			}
		}else if(param1 == MENU_VOLTAGE){
			if(param2 == SUBMENU_OFFSET){
				serialPrint("CALIB VOLTAGE OFFSET CALC\r\n", 27);
			}else if(param2 == SUBMENU_GAIN){
				serialPrint("CALIB VOLTAGE GAIN CALC\r\n", 25);
			}
		}else if(param1 == MENU_CURRENT){
			if(param2 == SUBMENU_OFFSET){
				serialPrint("CALIB CURRENT OFFSET CALC\r\n", 27);
			}else if(param2 == SUBMENU_GAIN){
				serialPrint("CALIB CURRENT GAIN CALC\r\n", 25);
			}
		}
	}
}

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
