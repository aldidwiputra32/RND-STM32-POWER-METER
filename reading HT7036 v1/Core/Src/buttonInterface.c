#include "buttonInterface.h"

uint8_t buttonStatus;
uint8_t buttonTrigger = 0;
uint8_t menuLevel = MENU_LEVEL_IDLE;
uint8_t menuParam = MENU_WIRING_TYPE;

void menuLoop(){
	// STATE MACHINE PROCESSING
	if(buttonTrigger){
		if(menuLevel == MENU_LEVEL_IDLE){
			// DISPLAY FUNCTION

			// SWITCH FROM DISPLAY MODE TO SETTING MODE
			if(buttonStatus == BTN_NEXT)menuLevel = MENU_LEVEL_0;
		}

		// LEVEL SECURITY
		else if(level == MENU_LEVEL_0){
			uint8_t buttonState
		}


		else if(level == MENU_LEVEL_1){

		}else if(level == MENU_LEVEL_2){

		}else if(level == MENU_LEVEL_3){

		}
		buttonTrigger = 0;
	}

	// LEVEL DISPLAY
}
void displayLoop(){

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
