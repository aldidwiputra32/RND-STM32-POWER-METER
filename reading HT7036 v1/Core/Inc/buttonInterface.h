#include "main.h"
#include "gpio.h"
#include  "tim.h"

#define BTN_IDLE 			0
#define BTN_NEXT 			BTN_Next_Pin
#define BTN_UP 				BTN_Up_Pin
#define BTN_SET 			BTN_Set_Pin
#define BTN_ENTER 			BTN_Enter_Pin
#define MENU_LEVEL_IDLE		200
#define MENU_LEVEL_0 		0
#define MENU_LEVEL_1 		1
#define MENU_LEVEL_2 		2
#define MENU_LEVEL_3 		3
#define MENU_WIRING_TYPE 	0
#define MENU_VOLTAGE 		1
#define MENU_CURRENT 		2
#define MENU_MODBUS 		3
#define MENU_ENERGY 		4
#define MENU_SAVE			5
#define SUBMENU_OFFSET		0
#define SUBMENU_GAIN		1
#define SUBMENU_E_ACTIVE	0
#define SUBMENU_E_REACTIVE	1
#define SUBMENU_N33			0
#define SUBMENU_N34			1
#define PARAM_IDLE			0xFFFFFFFF
#define CALIB				0

void menuLoop();
void displayLoop(uint8_t level, uint8_t param1, uint8_t param2, uint8_t param3);
void handleTreshold(uint8_t * val, uint8_t max, uint8_t min);
uint8_t convertRawBtnToFloat(float * buffer, uint8_t * data, float max);
