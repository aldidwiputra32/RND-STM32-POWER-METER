#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "string.h"
#include "modbusSlave.h"
#include "HT1622.h"
#include "HT7036.h"

// BUTTON DEFINE
#define BTN_IDLE 			0
#define BTN_NEXT 			BTN_Next_Pin
#define BTN_UP 				BTN_Up_Pin
#define BTN_SET 			BTN_Set_Pin
#define BTN_ENTER 			BTN_Enter_Pin

// MENU LCD DEFINE
#define MENU_LEVEL_IDLE		200
#define MENU_LEVEL_0 		0
#define MENU_LEVEL_1 		1
#define MENU_LEVEL_2 		2
#define MENU_LEVEL_3 		3
#define MENU_LEVEL_SAVE		4
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
#define WIRING_TYPE_DEF		0
#define SUBMENU_N33			0
#define SUBMENU_N34			1

// READING SENSOR DEFINE
#define DISPLAY_CURRENT_RMS			11	// A
#define DISPLAY_VOLTAGE_RMS 		12 	// V
#define DISPLAY_VOLTAGE_RMS_DIV		13 	// V
#define DISPLAY_ACTIVE_POWER		14 	// KW
#define DISPLAY_REACTIVE_POWER		15 	// VAR
#define DISPLAY_APPARENT_POWER		16 	// VA
#define DISPLAY_ACTIVE_ENERGY		17 	// KWH
#define DISPLAY_REACTIVE_ENERGY		18 	// KVARH
#define DISPLAY_POWER_FACTOR		19  // none

// STATE LCD DEFINE
#define PARAM_IDLE			0xFFFFFFFF
#define CALIB				0
#define SAVE 				2
#define BACK 				1
#define CANCEL				0
#define RESET 				0
#define NON_RESET			1



void menuLoop();
void displayLoop(uint8_t level, uint8_t param1, uint8_t param2, uint8_t param3);
void handleTreshold(uint32_t * val, uint8_t max, uint8_t min);
uint8_t convertRawBtnToFloat(float * buffer, uint8_t * data, float max);
void floatTodisplay(uint8_t * bufferDisplay, float dataFloat);
