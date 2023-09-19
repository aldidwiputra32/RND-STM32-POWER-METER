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
#define MENU_CURRENT 		0
#define MENU_MODBUS 		1
#define MENU_ENERGY 		2
#define MENU_SAVE			3

// READING SENSOR DEFINE
#define DISPLAY_CURRENT_RMS			0	// A
#define DISPLAY_VOLTAGE_RMS 		1 	// V
#define DISPLAY_VOLTAGE_RMS_DIV		2 	// V
#define DISPLAY_ACTIVE_POWER		3 	// KW
#define DISPLAY_REACTIVE_POWER		4 	// VAR
#define DISPLAY_APPARENT_POWER		5 	// VA
#define DISPLAY_POWER_FACTOR		6	// power factor

// STATE LCD DEFINE
#define PARAM_IDLE			0xFFFFFFFF
#define CALIB				0
#define SAVE 				2
#define BACK 				1
#define CANCEL				0
#define RESET 				0
#define NON_RESET			1



void menuLoop();
void displayLoop();
void handleTreshold(uint32_t * val, uint8_t max, uint8_t min);
uint8_t convertRawBtnToFloat(float * buffer, uint8_t * data, float max);
void floatTodisplay(uint8_t * bufferDisplay, float dataFloat);

