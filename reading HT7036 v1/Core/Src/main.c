/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HT7036.h"
#include "modbusSlave.h"
#include "ee24xx.h"
#include "buttonInterface.h"
#include "HT1622.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INTERVAL_EEPROM 	60000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//------------------------- GROUP VARIABLE POWER METER ---------------------------------
uint8_t addrSensor[] = {
		// RMS REGISTER >> 8 addrs
		r_UaRms,			r_UbRms,			r_UcRms,			r_UtRms,
		r_IaRms,			r_IbRms,			r_IcRms,			r_ItRms,
		// POWER REGISTER >> 12 addrs
		r_Pa,				r_Pb,				r_Pc,				r_Pt,
		r_Qa,				r_Qb,				r_Qc,				r_Qt,
		r_Sa,				r_Sb,				r_Sc,				r_St,
		// POWER FACTOR REGISTER >> 4 addrs
		r_Pfa,				r_Pfb,				r_Pfc,				r_Pft,
		// ENERGY REGISTER >> 8 addrs
		r_Epa,				r_Epb, 				r_Epc,				r_Ept,
		r_Eqa,				r_Eqb,				r_Eqc,				r_Eqt
		// TOTAL REGISTER >> 32
};

uint8_t sizeSensor = sizeof(addrSensor)/sizeof(addrSensor[0]);
uint32_t valueSensor[32];
HAL_StatusTypeDef spiStatus[256];
extern float HFconstVal;
extern float ECVal;
extern float ECDef;
extern uint64_t powerTimer;
uint8_t stateConfig = 0;
uint8_t phase = PHASE_RST;
float valueFloat[32];

// POWER REGISTER
float 	powerActiveA,		powerActiveB,		powerActiveC,		PowerActiveCombine,   // V x i x cos phi
		powerReactiveA,		powerReactiveB,		powerReactiveC,		powerReactiveCombine, // V x i x sin phi
		powerApparentA,		powerApparentB,		powerApparentC,		powerApparentCombine; // V x i
// RMS REGISTER
float 	rmsVoltageA,		rmsVoltageB,		rmsVoltageC,		rmsVoltageVector,
		rmsCurrentA,		rmsCurrentB,		rmsCurrentC,		rmsCurrentVector;
// POWER FACTOR REGISTER
float 	powerFactorA,		powerFactorB,		powerFactorC, 		powerFactorCombine;
// ENERGY REGISTER
float 	energyActiveA,		energyActiveB, 		energyActiveC,		energyActiveCombine,
		energyReactiveA,	energyReactiveB, 	energyReactiveC,	energyReactiveCombine;
//
float 	rmsVoltageAB,		rmsVoltageBC,		rmsVoltageCA;

extern uint64_t energyModbus[8];
extern double bufferEnergySUM[8];
extern uint32_t check;

uint64_t 	energyActiveA_uint,		energyActiveB_uint, 		energyActiveC_uint,		energyActiveCombine_uint,
			energyReactiveA_uint,	energyReactiveB_uint, 		energyReactiveC_uint,	energyReactiveCombine_uint;

float	gainVoltageA = 1,	gainVoltageB = 1,	gainVoltageC = 1,
		gainCurrentA = 1,	gainCurrentB = 1,	gainCurrentC = 1,
		gainVoltage = 1,	gainCurrent = 1;

float 	offsetVoltageA = 0, offsetVoltageB = 0,	offsetVoltageC = 0,
		offsetCurrentA = 0,	offsetCurrentB = 0,	offsetCurrentC = 0,
		offsetVoltage = 0,	offsetCurrent = 0, 	offsetPF_stm32 = 0,
		gainPF_stm32 = 1,	powerCoefActive = 0,powerCoefReactive = 0,
		powerCoefApparent = 0;

uint16_t offsetVolt_ht7036,	offsetCurr_ht7036,	gainVolt_ht7036,	gainCurr_ht7036,
		 offsetVolt_stm32,	offsetCurr_stm32,	gainVolt_stm32,		gainCurr_stm32,
		 calibPF_ht7036,	gainCurrentButton_stm32;

uint32_t powerApparentBitA,	powerApparentBitB,	powerApparentBitC;


//------------------------- GROUP VARIABLE MODBUS ---------------------------------
extern MODBUS Modbus;						//  ADDRESS REGISTER VALUE POWER SENSOR >> 92 Register
uint16_t holdingRegisterAddress[] 	= 	{	3027,  3028,  3029,  3030,  3031,  3032, 				// [v] Vrms ABC (V)
											3035,  3036,											// [x] Vrms Vector (V)
											2999,  3000,  3001,  3002,  3003,  3004,				// [v] Irms ABC (A)
											3009,  3010,											// [x] Irms Vector (A)
											3053,  3054,  3055,  3056,  3057,  3058,  3059,  3060,	// [v] Active power ABC & Total (kW)
											3061,  3062,  3063,  3064,  3065,  3066,  3067,  3068,	// [x] Reactive power ABC & Total (kVAR)
											3069,  3070,  3071,  3072,  3073,  3074,  3075,  3076,	// [x] Apparent Power ABC & Total (kVA)
											3077,  3078,  3079,  3080,  3081,  3082,				// [v] power factor ABC
											3083,  3084,											// [x] power factor combine
											3517,  3518,  3519,  3520,								// [x] active energy A
											3521,  3522,  3523,  3524, 								// [x] active energy B
											3525,  3526,  3527,  3528,								// [x] active energy C
											3203,  3204,  3205,  3206, 								// [v] Active Energy Delivered (Into Load) Total
											3529,  3530,  3531,  3532,								// [x] reactive energy A
											3533,  3534,  3535,  3536,								// [x] reactive energy B
											3537,  3538,  3539,  3540,								// [x] reactive energy C
											3219,  3220,  3221,  3222,  							// [v] Reactive Energy Delivered Total
											3019,  3020,  3021,  3022,  3023,  3024,				// [v] Vrms AB,BC,CA
											21329, 21330, 21331, 21332, 21333, 21334,				// [0] total harmonic distorision (%)
											// ADDRESS REGISTER SLAVE ID POWER SENSOR >> 1 Register
											0X1000,
											// ADDRESS REGISTER PARAMETER CALIBRATION POWER SENSOR FOR SUPER USER >> 12 Register
											0x1001, 0x1002, 0x1003, 								// offset voltage ABC Phase
											0x1004, 0x1005, 0x1006, 								// offset current ABC Phase
											0x1007, 0x1008, 0x1009, 								// gain voltage ABC Phase
											0x100A, 0x100B, 0x100C,									// gain current ABC Phase
											// ADDRESS REGISTER PARAMETER CALIBRATION POWER SENSOR FOR USER >> 12 Register
											0x2001, 0x2002, 0x2003, 								// offset voltage ABC Phase
											0x2004, 0x2005, 0x2006, 								// offset current ABC Phase
											0x2007, 0x2008, 0x2009, 								// gain voltage ABC Phase
											0x200A, 0x200B, 0x200C,									// gain current ABC Phase
											// [RAW DATA] ADDRESS REGISTER PARAMETER CALIBRATION POWER SENSOR FOR SUPER USER >> 4 Register
											0x3001,		 											// offset Voltage super User
											0x3002, 												// offset current super user (2 byte) >> HT7036
											0x3003, 												// gain voltage super user (2 byte) >> HT7036
											0x3004,													// gain current super user (2 byte) >> HT7036
											// ADDRESS REGISTER PARAMETER POWER METER CALIBRTION SUPER USER & USER >> 3 register
											0x4001,													// power phase corrction for super user >> HT7036
											0x4002,													// gain power factor for user >> STM32
											0x4003,													// offset power factpr for user >> STM32
											// ADDRESS REGISTER PARMATER CALIBRATION GAIN CURRENT VIA BUTTON SET & DECODE PARAM GROUP POWER >> 3 register
											0x4004,													// gain current button stm32
											0x4005, 0x4006,											// decode param group power active register >> HT7036
											0x5005, 0x5006,											// decode param group power reactive register >> ht7036
											0x6005,	0x6006											// decode param group power apparent register >> ht7036
};
											// VALUE REGISTER POWER SENSOR
//uint16_t holdingRegisterSize = (uint16_t)sizeof(holdingRegisterAddress)/sizeof(uint16_t);
uint16_t holdingRegisterSize = 131; // 125
uint16_t holdingRegisterValue[131]	= {0}; // 125
extern uint16_t addressModbus;

//------------------------- GROUP VARIABLE EEPROM EXTERNAL 8K ---------------------------------
uint8_t eepromBufferRead[86];
uint8_t eepromBufferWrite[86];
uint32_t eepromTimerDelta = 0;
uint32_t eepromTimer = 0;

//------------------------- GROUP VARIABLE BUTTON INTERFACE ---------------------------------
uint8_t buttonAntiBounce = 0;
extern uint16_t buttonStatus;
extern uint8_t buttonTrigger;
extern uint8_t menuLevel;
extern uint8_t menuParam;
extern uint8_t 	flagGetDataOld;
extern uint32_t paramLv1;
extern uint8_t stateConfigButton;

//------------------------- GROUP VARIABLE DISPLAY 7-SEGMENT ---------------------------------
uint32_t backlightTimer;
uint8_t backlightState=1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void serialPrint(char* text, uint8_t size){
//	HAL_GPIO_WritePin(MODBUS_En_GPIO_Port,MODBUS_En_Pin,GPIO_PIN_RESET);
	HAL_UART_Transmit(&huart2, (uint8_t*)text, size, 100);
//	HAL_GPIO_WritePin(MODBUS_En_GPIO_Port,MODBUS_En_Pin,GPIO_PIN_SET);
}
void powerMeterSetup();
void eepromEncode(
			uint64_t energyActiveA,				// indeks 0 - 7
			uint64_t energyActiveB,				// indeks 8 - 15
			uint64_t energyActiveC,				// indeks 16 - 23
			uint64_t energyReactiveA,			// indeks 24 - 31
			uint64_t energyReactiveB,			// indeks 32 - 39
			uint64_t energyReactiveC,			// indeks 40 - 47
			uint16_t offsetVolt_ht7036,			// indeks 48 - 49
			uint16_t offsetCurr_ht7036,			// indeks 50 - 51
			uint16_t gainVolt_ht7036,			// indeks 52 - 53
			uint16_t gainCurr_ht7036,			// indeks 54 - 55
			uint16_t offsetVolt_stm32,			// indeks 56 - 57
			uint16_t offsetCurr_stm32,			// indeks 58 - 59
			uint16_t gainVolt_stm32,			// indeks 60 - 62
			uint16_t gainCurr_stm32,			// indeks 62 - 63
			uint16_t slaveAddress,				// indeks 64 - 65
			uint16_t calibPF_ht7036,			// indeks 66 - 67
			uint16_t gainPF_stm32,				// indeks 68 - 69
			uint16_t offsetPF_stm32,			// indeks 70 - 71
			uint16_t gainCurrentButton_stm32,	// indeks 72 - 73
			uint32_t powerCoefActive,			// indeks 74 - 77
			uint32_t powerCoefReactive,			// indeks 78 - 81
			uint32_t powerCoefApparent			// indeks 82 - 85
);
uint16_t byteLow32(uint32_t buf){return (uint16_t)((buf & 0x0000FFFF));}
uint16_t byteHigh32(uint32_t buf){return (uint16_t)((buf & 0xFFFF0000) >> 16);}
void modbusValueUpdate();
void powerCalibLoop();
void powerSplitValue();
void powerHandleCalib();
void eepromLoop();
void eepromLoad();
void powerHandleTresholdGroup();
void powerHandleTreshold(float * data, float max, float min);
void backlightHandle();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_Delay(1000);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  // MODBUS SETUP
  ModbusBegin(
		  &Modbus,
		  &huart2,
		  0,
		  0x00,
		  0x01,
		  holdingRegisterAddress,
		  holdingRegisterValue,
		  holdingRegisterSize,
		  MODBUS_En_GPIO_Port,
		  MODBUS_En_Pin
  );
  // START EEPROM EXTERNAL
  ee24_init(&hi2c2, 0, 0, 0);
  eepromLoad();
  // START HT1622
  ht1622_init();
  clean_all();
  // SETUP POWER METER
  powerMeterSetup();
  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // POWER GROUP FUNCTION (TIMER, READ SENSOR, DATA PROCESSING, CALIB HANDLING) >> ASYNCRONOUS TASK
	  powerMultiReadSensor(addrSensor, valueSensor, valueFloat, 32);
	  powerSplitValue();
	  powerCalibLoop();
	  // LCD CUSTOM GROUP FUNCTION
	  menuLoop();
	  backlightHandle();
	  // EEPROM GROUP FUNCTION
	  eepromLoop();
	  // MODBUS UPDATE
	  modbusValueUpdate();
	  HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL10;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void backlightHandle(){
	if(backlightState && (HAL_GetTick() - backlightTimer > 60000)){
		HAL_GPIO_WritePin(BACKLIGHT_En_GPIO_Port, BACKLIGHT_En_Pin, GPIO_PIN_RESET);
		backlightState = 0;
	}
}

void powerMeterSetup(){
	uint8_t address[] = {
			  // OFFSET CURRENT HT7036
			  w_IaRmsoffse,
			  w_IbRmsoffse,
			  w_IcRmsoffse,
			  // OFFSET VOLTAGE HT7036
			  w_UaRmsoffse,
			  w_UbRmsoffse,
			  w_UcRmsoffse,
			  // GAIN VOLTAGE HT7036
	  		  w_UgainA,
			  w_UgainB,
			  w_UgainC,
			  // GAIN CURRENT HT7036
			  w_IgainA,
			  w_IgainB,
			  w_IgainC
	};
	uint32_t addressData[] = {
			  // OFFSET CURRENT HT7036
			  offsetCurr_ht7036,
			  offsetCurr_ht7036,
			  offsetCurr_ht7036,
			  // GAIN CURRENT HT7036
			  offsetVolt_ht7036,
			  offsetVolt_ht7036,
			  offsetVolt_ht7036,
			  // GAIN VOLTAGE HT7036
			  gainVolt_ht7036,
			  gainVolt_ht7036,
			  gainVolt_ht7036,
			  // GAIN CURRENT HT7036
			  gainCurr_ht7036,
			  gainCurr_ht7036,
			  gainCurr_ht7036,
	};
	powerRestoreCalib();
	powerSetup(address,addressData,spiStatus,12);
	powerMultiReadSensor(addrSensor, valueSensor, valueFloat, 32);
	powerSplitValue();
	// INITIAL TIMER SAMPLING POWER & EEPROM
	powerTimer = eepromTimer = HAL_GetTick();
}

void modbusValueUpdate(){
	float bufferFloat;
	uint32_t bufferUnsign32;
	uint64_t bufferUnsign64;
	uint8_t address = 0;
	uint16_t byteHigh,byteLow;
	for(uint8_t indeks=0; indeks<32; indeks++){
		// RMS GROUP SENSOR >> if(indeks>=0 && indeks<8)
		if(indeks>=0 && indeks<8){
			bufferUnsign32 = floatToInt32(&valueFloat[indeks]);
			byteHigh = byteHigh32(bufferUnsign32);
			byteLow = byteLow32(bufferUnsign32);
			Modbus.holdingRegisterValue[address++] = byteHigh;
			Modbus.holdingRegisterValue[address++] = byteLow;
		}
		// POWER GROUP SENSOR >> if(indeks>=8 && indeks<20)
		if(indeks>=8 && indeks<20){
			bufferFloat = valueFloat[indeks]/1000;
			bufferUnsign32 = floatToInt32(&bufferFloat);
			byteHigh = byteHigh32(bufferUnsign32);
			byteLow = byteLow32(bufferUnsign32);
			Modbus.holdingRegisterValue[address++] = byteHigh;
			Modbus.holdingRegisterValue[address++] = byteLow;
		}
		// POWER FACTOR GROUP SENSOR >> if(indeks>=20 && indeks<24)
		if(indeks>=20 && indeks<24){
			bufferUnsign32 = floatToInt32(&valueFloat[indeks]);
			byteHigh = byteHigh32(bufferUnsign32);
			byteLow = byteLow32(bufferUnsign32);
			Modbus.holdingRegisterValue[address++] = byteHigh;
			Modbus.holdingRegisterValue[address++] = byteLow;
		}
 		// ENERGY GROUOP SENSOR >> if(indeks>=24 && indeks<32)
		if(indeks>=24 && indeks<32){
			bufferUnsign64 = energyModbus[indeks-24];
			// haNDLING VALUE FOR SMART LOAD
			//if(bufferUnsign64 == 0)bufferUnsign64 = ZERO_VAL_ENERGY;
			Modbus.holdingRegisterValue[address++] = byte64High1(bufferUnsign64);
			Modbus.holdingRegisterValue[address++] = byte64High2(bufferUnsign64);
			Modbus.holdingRegisterValue[address++] = byte64Low1(bufferUnsign64);
			Modbus.holdingRegisterValue[address++] = byte64Low2(bufferUnsign64);
		}
	}
	// VOLTAGE DIFFERENTIAL GROUP
	// TOTAL HARMONIC DISTORTION
	bufferFloat = rmsVoltageAB;
	bufferUnsign32 = floatToInt32(&bufferFloat);
	Modbus.holdingRegisterValue[address++] = byteHigh32(bufferUnsign32);
	Modbus.holdingRegisterValue[address++] = byteLow32(bufferUnsign32);
	bufferFloat = rmsVoltageBC;
	bufferUnsign32 = floatToInt32(&bufferFloat);
	Modbus.holdingRegisterValue[address++] = byteHigh32(bufferUnsign32);
	Modbus.holdingRegisterValue[address++] = byteLow32(bufferUnsign32);
	bufferFloat = rmsVoltageCA;
	bufferUnsign32 = floatToInt32(&bufferFloat);
	Modbus.holdingRegisterValue[address++] = byteHigh32(bufferUnsign32);
	Modbus.holdingRegisterValue[address++] = byteLow32(bufferUnsign32);
	// bufferFloat = ZERO_VAL;
	for(uint8_t i=0;i<3;i++){
		// Modbus.holdingRegisterValue[address++] = byteHigh32(floatToInt32(&bufferFloat));
		// Modbus.holdingRegisterValue[address++] = byteLow32(floatToInt32(&bufferFloat));
		Modbus.holdingRegisterValue[address++] = 0;
		Modbus.holdingRegisterValue[address++] = 0;
	}
}

void powerCalibLoop(){
	if(Modbus.trigState){
		uint16_t addressSlave;
		uint8_t addressIndeks;
		uint16_t dataCalib16;
		uint32_t dataCalib32;
		// FUCTION CODE WRITE REGISTER
		if(Modbus.functionCode == 0x06){
			uint8_t splitSensorIndeks[] = {
					0, 1, 2,	// Address Write Voltage Offset	<< 0x1001(45), 0x1002(46), 0x1003(47)
					4, 5, 6,	// Address Write Current Offset	<< 0x1004(48), 0x1005(49), 0x1006(50)
					0, 1, 2,	// Address Write Voltage Offset	<< 0x1001(45), 0x1002(46), 0x1003(47)
					4, 5, 6		// Address Write Current Offset	<< 0x1004(48), 0x1005(49), 0x1006(50)
			};
			// SCAN ADDRESS
			for(uint8_t indeks=0;indeks<12;indeks++){
				if(addressModbus == Modbus.holdingRegisterAddress[93+indeks])addressIndeks = splitSensorIndeks[indeks];
				else __NOP();
			}

			// --------------------------------------------------CALIBRATION HT7036 FOR SUPER USER------------------------------------------------------------------
			// FILTER REGISTER OFFSET VOLTAGE RMS
			if((addressModbus == 0x1001) || (addressModbus == 0x1002) || (addressModbus == 0x1003)){
				// GET DATA MODBUS FOR ZEROIING VALUE
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				dataCalib16 = Modbus.holdingRegisterValue[addressSlave];
				dataCalib32 = (uint32_t)dataCalib16;
				stateConfig = Modbus.trigState;
				// GET ADDRESS REGISTER PARAMETER CALIBRATION [RAW DATA]
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x3001, Modbus.holdingRegisterSize);
				if(addressModbus == 0x1001){offsetVolt_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(VRMS_OFFSET, w_UaRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],offsetVolt_ht7036);phase=PHASE_A;}
				else if(addressModbus == 0x1002){offsetVolt_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(VRMS_OFFSET, w_UbRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],offsetVolt_ht7036);phase=PHASE_B;}
				else if(addressModbus == 0x1003){offsetVolt_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(VRMS_OFFSET, w_UcRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],offsetVolt_ht7036);phase=PHASE_C;}
			}
			// FILTER REGISTER OFFSET CURRENT RMS
			else if((addressModbus == 0x1004) || (addressModbus == 0x1005) || (addressModbus == 0x1006)){
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				dataCalib16 = Modbus.holdingRegisterValue[addressSlave];
				dataCalib32 = (uint32_t)dataCalib16;
				stateConfig = Modbus.trigState;
				// GET ADDRESS REGISTER PARAMETER CALIBRATION [RAW DATA]
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x3002, Modbus.holdingRegisterSize);
				if(addressModbus == 0x1004){offsetCurr_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(IRMS_OFFSET, w_IaRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],offsetCurr_ht7036);phase=PHASE_A;}
				else if(addressModbus == 0x1005){offsetCurr_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(IRMS_OFFSET, w_IbRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],offsetCurr_ht7036);phase=PHASE_B;}
				else if(addressModbus == 0x1006){offsetCurr_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(IRMS_OFFSET, w_IcRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],offsetCurr_ht7036);phase=PHASE_C;}
			}
			// FILTER REGISTER GAIN VOLTAGE RMS
			else if((addressModbus == 0x1007) || (addressModbus == 0x1008) || (addressModbus == 0x1009)){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				dataCalib16 = Modbus.holdingRegisterValue[addressSlave];
				dataCalib32 = (uint32_t)dataCalib16;
				// GET ADDRESS REGISTER PARAMETER CALIBRATION [RAW DATA]
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x3003, Modbus.holdingRegisterSize);
				if(dataCalib16 != 0){
					if(addressModbus == 0x1007){gainVolt_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(VRMS_GAIN, w_UgainA, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],gainVolt_ht7036);phase=PHASE_A;}
					else if(addressModbus == 0x1008){gainVolt_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(VRMS_GAIN, w_UgainB, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],gainVolt_ht7036);phase=PHASE_B;}
					else if(addressModbus == 0x1009){gainVolt_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(VRMS_GAIN, w_UgainC, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],gainVolt_ht7036);phase=PHASE_C;}
				}else __NOP();
			}
			// FILTER REGISTER GAIN CURRENT RMS
			else if((addressModbus == 0x100A) || (addressModbus == 0x100B) || (addressModbus == 0x100C)){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				dataCalib16 = Modbus.holdingRegisterValue[addressSlave];
				dataCalib32 = (uint32_t)dataCalib16;
				// GET ADDRESS REGISTER PARAMETER CALIBRATION [RAW DATA]
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x3004, Modbus.holdingRegisterSize);
				if(addressSlave !=0){
					if(addressModbus == 0x100A){gainCurr_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(IRMS_GAIN, w_IgainA, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],gainCurr_ht7036);phase=PHASE_A;}
					else if(addressModbus == 0x100B){gainCurr_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(IRMS_GAIN, w_IgainB, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],gainCurr_ht7036);phase=PHASE_B;}
					else if(addressModbus == 0x100C){gainCurr_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(IRMS_GAIN, w_IgainC, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],gainCurr_ht7036);phase=PHASE_C;}
				}else __NOP();
			}
			// FILTER POWER FACTOR CALIBRATION
			else if(addressModbus == 0x4001){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				calibPF_ht7036 = Modbus.holdingRegisterValue[addressSlave];
				dataCalib32 = (uint32_t)calibPF_ht7036;
				// WRITE REGISTER VALUE
				powerSingleRecalib(PF_CALIB, w_PhSregApq1, &dataCalib32, w_PhSregApq1,  &spiStatus[0], check);  // variable check just buffer not important
				powerSingleRecalib(PF_CALIB, w_PhSregBpq1, &dataCalib32, w_PhSregBpq1,  &spiStatus[0], check);  // variable check just buffer not important
				powerSingleRecalib(PF_CALIB, w_PhSregCpq1, &dataCalib32, w_PhSregCpq1,  &spiStatus[0], check);  // variable check just buffer not important
			}

			// --------------------------------------------------CALIBRATION STM32 FOR USER------------------------------------------------------------------
			// FILTER REGISTER OFFSET VOLTAGE RMS
			else if ((addressModbus == 0x2001) || (addressModbus == 0x2002) || (addressModbus == 0x2003)){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				offsetVolt_stm32 = dataCalib16 = Modbus.holdingRegisterValue[addressSlave];
				if(addressModbus == 0x2001){offsetVoltageA = (float)dataCalib16/1000;phase=PHASE_A;}
				if(addressModbus == 0x2002){offsetVoltageB = (float)dataCalib16/1000;phase=PHASE_B;}
				if(addressModbus == 0x2003){offsetVoltageC = (float)dataCalib16/1000;phase=PHASE_C;}
			}
			// FILTER REGISTER OFFSET CURRENT RMS
			else if ((addressModbus == 0x2004) || (addressModbus == 0x2005) || (addressModbus == 0x2006)){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				offsetCurr_stm32 = dataCalib16 = Modbus.holdingRegisterValue[addressSlave];
				if(addressModbus == 0x2004){offsetCurrentA = (float)dataCalib16/1000;phase=PHASE_A;}
				if(addressModbus == 0x2005){offsetCurrentB = (float)dataCalib16/1000;phase=PHASE_B;}
				if(addressModbus == 0x2006){offsetCurrentC = (float)dataCalib16/1000;phase=PHASE_C;}
			}
			// FILTER REGISTER GAIN VOLTAGE RMS
			else if ((addressModbus == 0x2007) || (addressModbus == 0x2008) || (addressModbus == 0x2009)){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				gainVolt_stm32 = dataCalib16 = Modbus.holdingRegisterValue[addressSlave];
				if(addressModbus == 0x2007){gainVoltageA = (float)dataCalib16/1000;phase=PHASE_A;}
				if(addressModbus == 0x2008){gainVoltageB = (float)dataCalib16/1000;phase=PHASE_B;}
				if(addressModbus == 0x2009){gainVoltageC = (float)dataCalib16/1000;phase=PHASE_C;}
			}
			// FILTER REGISTER GAIN CURRENT RMS >> NOT 1000 MULTIPLICATION COZ USER REQUIREMENT
			else if ((addressModbus == 0x200A) || (addressModbus == 0x200B) || (addressModbus == 0x200C)){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				gainCurr_stm32 = dataCalib16 = Modbus.holdingRegisterValue[addressSlave];
				if(addressModbus == 0x200A){gainCurrentA = (float)dataCalib16/1000;phase=PHASE_A;}
				if(addressModbus == 0x200B){gainCurrentB = (float)dataCalib16/1000;phase=PHASE_B;}
				if(addressModbus == 0x200C){gainCurrentC = (float)dataCalib16/1000;phase=PHASE_C;}
			}
			// FIITER REGISTER FOR OFFSET GAIN POWER FACTOR STM32
			else if((addressModbus == 0x4002) || (addressModbus == 0x4003)){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				int16_t bufferInt16 = (int16_t)Modbus.holdingRegisterValue[addressSlave];
				if(addressModbus == 0x4002){gainPF_stm32 = (float)bufferInt16 / 10000;}	// 0x40002
				else{offsetPF_stm32 = (float)bufferInt16 / 10000;}						// 0x40003
			}
			// FILTER REGISTER FOR GAIN CURRENT VIA BUTTON SET
			else if(addressModbus == 0x4004){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				gainCurrentButton_stm32 = Modbus.holdingRegisterValue[addressSlave];
			}
		}
		// --------------------------------------------------HANDLE RESET VALUE REGISTER[ENERGY]------------------------------------------------------------------
		if(Modbus.functionCode == 0x10){
			// GETTING DATA ENERGY
			uint16_t buffer16[4],addressModbusBuffer,addressSlaveArray[4];
			uint64_t buffer64;
			if((addressModbus == 3517) || (addressModbus == 3521) || (addressModbus == 3525) || (addressModbus == 3203) || (addressModbus == 3529) || (addressModbus == 3533) || (addressModbus == 3537) || (addressModbus == 3219)){
				stateConfig = Modbus.trigState;
				addressModbusBuffer = addressModbus;
				for(uint8_t i=0;i<4;i++){
					addressSlaveArray[i] = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbusBuffer+i, Modbus.holdingRegisterSize);
					buffer16[i] = Modbus.holdingRegisterValueRX[addressSlaveArray[i]];
				}
				uint8_t buffer8[8] = {
						byte16High(buffer16[0]),
						byte16Low(buffer16[0]),
						byte16High(buffer16[1]),
						byte16Low(buffer16[1]),
						byte16High(buffer16[2]),
						byte16Low(buffer16[2]),
						byte16High(buffer16[3]),
						byte16Low(buffer16[3]),
				};
				uint8Touint64(&buffer64, buffer8);

				// ACTIVE ENERGY
				if(addressModbus == 3517){energyActiveA_uint = buffer64; bufferEnergySUM[0] = (double)energyActiveA_uint;}
				if(addressModbus == 3521){energyActiveB_uint = buffer64; bufferEnergySUM[1] = (double)energyActiveB_uint;}
				if(addressModbus == 3525){energyActiveC_uint = buffer64; bufferEnergySUM[2] =  (double)energyActiveC_uint;}
				if(addressModbus == 3203){
					energyActiveA_uint = (uint64_t)buffer64/3; bufferEnergySUM[0] = (double)energyActiveA_uint;
					energyActiveB_uint = (uint64_t)buffer64/3; bufferEnergySUM[1] = (double)energyActiveB_uint;
					energyActiveC_uint = (uint64_t)buffer64/3; bufferEnergySUM[2] = (double)energyActiveC_uint;
					energyActiveCombine_uint = energyActiveA_uint + energyActiveB_uint + energyActiveC_uint;
					bufferEnergySUM[3] = (double)energyActiveCombine_uint;
				}
				// 	REACTIVE ENERGY
				if(addressModbus == 3529){energyReactiveA_uint = buffer64;bufferEnergySUM[4] = (double)energyReactiveA_uint;}
				if(addressModbus == 3533){energyReactiveB_uint = buffer64; bufferEnergySUM[5] = (double)energyReactiveB_uint;}
				if(addressModbus == 3537){energyReactiveC_uint = buffer64; bufferEnergySUM[6] = (double)energyReactiveC_uint;}
				if(addressModbus == 3219){
					energyReactiveA_uint = (uint64_t)buffer64/3; bufferEnergySUM[4] = (double)energyReactiveA_uint;
					energyReactiveB_uint = (uint64_t)buffer64/3; bufferEnergySUM[5] = (double)energyReactiveB_uint;
					energyReactiveC_uint = (uint64_t)buffer64/3; bufferEnergySUM[6] = (double)energyReactiveC_uint;
					energyReactiveCombine_uint = energyReactiveA_uint + energyReactiveB_uint + energyReactiveC_uint;
					bufferEnergySUM[7] = (double)energyReactiveCombine_uint;
				}
			}
			// handling modbus power active
			else if(addressModbus == 0x4005){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks( Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				powerCoefActive =(float)(uint16ToUint32(Modbus.holdingRegisterValue[addressSlave], Modbus.holdingRegisterValue[addressSlave+1]))/1000000000; // deivide by 10 miliar
			}
			// handling modbus power reactive
			else if(addressModbus == 0x5005){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks( Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				powerCoefReactive =(float)(uint16ToUint32(Modbus.holdingRegisterValue[addressSlave], Modbus.holdingRegisterValue[addressSlave+1]))/1000000000; // deivide by 10 miliar
			}
			// handling modbus power apparent
			else if(addressModbus == 0x6005){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks( Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				powerCoefApparent = (float)(uint16ToUint32(Modbus.holdingRegisterValue[addressSlave], Modbus.holdingRegisterValue[addressSlave+1]))/1000000000; // deivide by 10 miliar
			}
		}else __NOP();
		Modbus.trigState = 0;
	}else __NOP();
}

void eepromLoad(){
	uint8_t indeksAddress = 0;
	uint8_t buffer8[8];
	uint16_t bufferUint16;
	int16_t bufferInt16;
	uint32_t bufferUint32;
	// GET DATA FROM EEPROM EXTERNAL
	ee24_read(0, (uint8_t*)eepromBufferRead, sizeof(eepromBufferRead), 1000);// for(uint8_t indeks=0;indeks<64;indeks++)ee24VirtualRead(&eepromBufferRead[indeks], 0, 1024, indeks);
	// DECODE DATA
	for(uint8_t indeks=0;indeks<86;indeks++){
		// DECODE ACTIVE ENERGY PHASE A >> valueuint64 [0];
		if(indeks>=0 && indeks<8)buffer8[indeks] = eepromBufferRead[indeks];
		if(indeks == 7){
			uint8Touint64(&energyActiveA_uint, buffer8);
			if(energyActiveA_uint == 0xFFFFFFFFFFFFFFFF)energyActiveA_uint = ENERGY_ACTIVE_A_DEF;
			indeksAddress = 0;
		}
		// DECODE ACTIVE ENERGY PHASE B
		if(indeks>=8 && indeks<16)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks == 15){
			uint8Touint64(&energyActiveB_uint, buffer8);
			if(energyActiveB_uint == 0xFFFFFFFFFFFFFFFF)energyActiveB_uint = ENERGY_ACTIVE_B_DEF;
			indeksAddress = 0;
		}
		// DECODE ACTIVE ENERGY PHASE C
		if(indeks>=16 && indeks<24)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks == 23){
			uint8Touint64(&energyActiveC_uint, buffer8);
			if(energyActiveC_uint == 0xFFFFFFFFFFFFFFFF)energyActiveC_uint = ENERGY_ACTIVE_C_DEF;
			indeksAddress = 0;
		}
		// DECODE REACTIVE ENERGY PHASE A
		if(indeks>=24 && indeks<32)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks == 31){
			uint8Touint64(&energyReactiveA_uint, buffer8);
			if(energyReactiveA_uint == 0xFFFFFFFFFFFFFFFF)energyReactiveA_uint = ENERGY_REACTIVE_A_DEF;
			indeksAddress = 0;
		}
		// DECODE REACRIVE ENERGY PHASE B
		if(indeks>=32 && indeks<40)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks == 39){
			uint8Touint64(&energyReactiveB_uint, buffer8);
			if(energyReactiveB_uint == 0xFFFFFFFFFFFFFFFF)energyReactiveB_uint = ENERGY_REACTIVE_B_DEF;
			indeksAddress = 0;
		}
		// DECODE REACTIVE ENERGY PHASE C
		if(indeks>=40 && indeks<48)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks == 47){
			uint8Touint64(&energyReactiveC_uint, buffer8);
			if(energyReactiveC_uint == 0xFFFFFFFFFFFFFFFF)energyReactiveC_uint = ENERGY_REACTIVE_C_DEF;
			indeksAddress = 0;
		}
		// DECODE OFFSET VOLTAGE SUPER USER [HT7036]
		if(indeks>=48 && indeks<50)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==49){
			offsetVolt_ht7036 = uint8ToUint16(buffer8[0], buffer8[1]);
			if(offsetVolt_ht7036 == 0xFFFF)offsetVolt_ht7036 = OFFSET_VOLT_HT_DEF;
			indeksAddress = 0;
		}
		// DECODE OFFSET CURRENT SUPER USER [HT7036]
		if(indeks>=50 && indeks<52)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==51){
			offsetCurr_ht7036 = uint8ToUint16(buffer8[0], buffer8[1]);
			if(offsetCurr_ht7036 == 0xFFFF)offsetCurr_ht7036 = OFFSET_CURR_HT_DEF;
			indeksAddress = 0;
		}
		// DECODE GAIN VOLTAGE SUPER USER [HT7036]
		if(indeks>=52 && indeks<54)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==53){
			gainVolt_ht7036 = uint8ToUint16(buffer8[0], buffer8[1]);
			if(gainVolt_ht7036 == 0xFFFF)gainVolt_ht7036 = GAIN_VOLT_HT_DEF;
			indeksAddress = 0;
		}
		// DECODE GAIN CURRANT SUPER USER [HT7036]
		if(indeks>=54 && indeks<56)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==55){
			gainCurr_ht7036 = uint8ToUint16(buffer8[0], buffer8[1]);
			if(gainCurr_ht7036 == 0xFFFF)gainCurr_ht7036 = GAIN_CURR_HT_DEF;
			indeksAddress = 0;
		}
		// DECODE OFFSET VOLTAGE USER [STM32]
		if(indeks>=56 && indeks<58)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==57){
			offsetVolt_stm32 = uint8ToUint16(buffer8[0], buffer8[1]);
			if(offsetVolt_stm32 == 0xFFFF)offsetVolt_stm32 = OFFSET_VOLT_STM_DEF;
			indeksAddress = 0;
		}
		// DECODE OFFSET CURRENT USER [STM32]
		if(indeks>=58 && indeks<60)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==59){
			offsetCurr_stm32 = uint8ToUint16(buffer8[0], buffer8[1]);
			if(offsetCurr_stm32 == 0xFFFF)offsetCurr_stm32 = OFFSET_CURR_STM_DEF;
			indeksAddress = 0;}
		// DECODE GAIN VOLTAGE USER [STM32]
		if(indeks>=60 && indeks<62)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==61){
			gainVolt_stm32 = uint8ToUint16(buffer8[0], buffer8[1]);
			if(gainVolt_stm32 == 0xFFFF)gainVolt_stm32 = GAIN_VOLT_STM_DEF;
			indeksAddress = 0;
		}
		// DECODE GAIN CURRANT USER [STM32]
		if(indeks>=62 && indeks<64)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==63){
			gainCurr_stm32 = uint8ToUint16(buffer8[0], buffer8[1]);
			if(gainCurr_stm32 == 0xFFFF)gainCurr_stm32 = GAIN_CURR_STM_DEF;
			indeksAddress = 0;
		}
		// DECODE SLAVE ADDRESS MODBUS
		if(indeks>=64 && indeks<66)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==65){
			bufferUint16 = uint8ToUint16(buffer8[0], buffer8[1]);
			if(bufferUint16 == 0xFFFF){
				Modbus.slaveAddrSlaveSecond = SLAVEID_DEF;
			}else{
				Modbus.slaveAddrSlaveSecond = (uint8_t)bufferUint16;
			}
			indeksAddress = 0;
		}
		// DECODE POWER FACTOR CALIBRATION SUPER USER
		if (indeks>=66 && indeks<68)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==67){
			bufferUint16 = uint8ToUint16(buffer8[0], buffer8[1]);
			if(bufferUint16 == 0xFFFF){
				calibPF_ht7036 = (float)PHASE_CORRECTION_ONE;
			}else{
				calibPF_ht7036 = bufferUint16;
			}
			indeksAddress = 0;
		}
		// DECODE GAIN POWER FACTOR CALIBRATION >> STM32
		if (indeks>=68 && indeks<70)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==69){
			bufferUint16 = uint8ToUint16(buffer8[0], buffer8[1]);
			bufferInt16 = (int16_t)bufferUint16;
			if(bufferUint16 == 0xFFFF){
				gainPF_stm32 = (float)GAIN_PF_DEF;
			}else{
				gainPF_stm32 = (float)bufferInt16/10000;
			}
			indeksAddress = 0;
		}
		// DECODE OFFSET POWER FACTOR CALIBRATION
		if(indeks>=70 && indeks<72)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==71){
			bufferUint16 = uint8ToUint16(buffer8[0], buffer8[1]);
			bufferInt16 = (int16_t)bufferUint16;
			if(bufferUint16 == 0xFFFF){
				offsetPF_stm32 = (float)GAIN_PF_DEF;
			}else{
				offsetPF_stm32 = (float)bufferInt16/10000;
			}
			indeksAddress = 0;
		}
		// DECODE GAIN CURRENT VIA BUTTTON SET
		if(indeks>=72 && indeks<74)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==73){
			bufferUint16 = uint8ToUint16(buffer8[0], buffer8[1]);
			if(bufferUint16 == 0xFFFF){
				gainCurrentButton_stm32 = GAIN_BUTTON_DEF;
			}else{
				gainCurrentButton_stm32 = bufferUint16;
			}
			indeksAddress = 0;
		}
		// DECODE POWER ACTIVE COEFFICIENT
		if(indeks>=74 && indeks<78)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==77){
			bufferUint32 = uint16ToUint32(uint8ToUint16(buffer8[0],buffer8[1]),uint8ToUint16(buffer8[2],buffer8[3]));
			if(bufferUint32 == 0xFFFFFFFF){powerCoefActive = POWER_COEF_DEF;}
			else{powerCoefActive = (float)bufferUint32/1000000000;}
			indeksAddress = 0;
		}
		// DECODE POWER REACTIVE COEFFICIENT
		if(indeks>=78 && indeks<82)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==81){
			bufferUint32 = uint16ToUint32(uint8ToUint16(buffer8[0],buffer8[1]),uint8ToUint16(buffer8[2],buffer8[3]));
			if(bufferUint32 == 0xFFFFFFFF){powerCoefReactive = POWER_COEF_DEF;}
			else{powerCoefReactive = (float)bufferUint32/1000000000;}
			indeksAddress = 0;
		}
		// DECODE POWER APPARENT COEFFICIENT
		if(indeks>=82 && indeks<86)buffer8[indeksAddress++] = eepromBufferRead[indeks];
		if(indeks==85){
			bufferUint32 = uint16ToUint32(uint8ToUint16(buffer8[0],buffer8[1]),uint8ToUint16(buffer8[2],buffer8[3]));
			if(bufferUint32 == 0xFFFFFFFF){powerCoefApparent = POWER_COEF_DEF;}
			else{powerCoefApparent = (float)bufferUint32/1000000000;}
			indeksAddress = 0;
		}
	}
	// SYNCRON FROM DATA EEPROM TO ENERGY[BUFFER ARRAY]
	bufferEnergySUM[0] = (double)energyActiveA_uint;
	bufferEnergySUM[1] = (double)energyActiveB_uint;
	bufferEnergySUM[2] = (double)energyActiveC_uint;
	bufferEnergySUM[3] = (double)(energyActiveA_uint+energyActiveB_uint+energyActiveC_uint);
	bufferEnergySUM[4] = (double)energyReactiveA_uint;
	bufferEnergySUM[5] = (double)energyReactiveB_uint;
	bufferEnergySUM[6] = (double)energyReactiveC_uint;
	bufferEnergySUM[7] = (double)(energyReactiveA_uint+energyReactiveB_uint+energyReactiveC_uint);
	// SYNCRON FROM DATA EEPROM TO PARAMETER CALIBRATION
	offsetVoltage = (float)offsetVolt_stm32/1000;
	offsetCurrent = (float)offsetCurr_stm32/1000;
	gainVoltage = (float)gainVolt_stm32/1000;
	gainCurrent = (float)gainCurr_stm32/1000;
	// SYNCRON FROM DATA EEPROM TO MODBUS REGISTER
	uint16_t addressSlave[3];
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x3001, Modbus.holdingRegisterSize);		// offset Voltage HT7036
	Modbus.holdingRegisterValue[addressSlave[0]] = offsetVolt_ht7036;
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x3002, Modbus.holdingRegisterSize);		// offset Current HT7036
	Modbus.holdingRegisterValue[addressSlave[0]] = offsetCurr_ht7036;
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x3003, Modbus.holdingRegisterSize);		// gain Voltage HT7036
	Modbus.holdingRegisterValue[addressSlave[0]] = gainVolt_ht7036;
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x3004, Modbus.holdingRegisterSize);		// gain Current HT7036
	Modbus.holdingRegisterValue[addressSlave[0]] = gainCurr_ht7036;
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2001, Modbus.holdingRegisterSize);		// offset Voltage STM32
	addressSlave[1] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2002, Modbus.holdingRegisterSize);
	addressSlave[2] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2003, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[addressSlave[0]] = Modbus.holdingRegisterValue[addressSlave[1]] = Modbus.holdingRegisterValue[addressSlave[2]] = offsetVolt_stm32;
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2004, Modbus.holdingRegisterSize);		// offset Current STM32
	addressSlave[1] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2005, Modbus.holdingRegisterSize);
	addressSlave[2] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2006, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[addressSlave[0]] = Modbus.holdingRegisterValue[addressSlave[1]] = Modbus.holdingRegisterValue[addressSlave[2]] = offsetCurr_stm32;
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2007, Modbus.holdingRegisterSize);		// gain Voltage STM32
	addressSlave[1] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2008, Modbus.holdingRegisterSize);
	addressSlave[2] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2009, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[addressSlave[0]] = Modbus.holdingRegisterValue[addressSlave[1]] = Modbus.holdingRegisterValue[addressSlave[2]] = gainVolt_stm32;
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x200A, Modbus.holdingRegisterSize);		// gain Current STM32
	addressSlave[1] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x200B, Modbus.holdingRegisterSize);
	addressSlave[2] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x200C, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[addressSlave[0]] = Modbus.holdingRegisterValue[addressSlave[1]] = Modbus.holdingRegisterValue[addressSlave[2]] = gainCurr_stm32;
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x1000, Modbus.holdingRegisterSize);		// modbus slave id
	Modbus.holdingRegisterValue[addressSlave[0]] = Modbus.slaveAddrSlaveSecond;
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x4001, Modbus.holdingRegisterSize);		// power factor calibration
	Modbus.holdingRegisterValue[addressSlave[0]] = calibPF_ht7036;
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x4002, Modbus.holdingRegisterSize);		// gain power factor user >> stm32
	Modbus.holdingRegisterValue[addressSlave[0]] = (uint16_t)(gainPF_stm32*10000);
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x4003, Modbus.holdingRegisterSize);		// offset power factor user >> stm32
	Modbus.holdingRegisterValue[addressSlave[0]] = (uint16_t)(offsetPF_stm32*10000);
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x4004, Modbus.holdingRegisterSize);		// gain current user via button set >> stm32
	Modbus.holdingRegisterValue[addressSlave[0]] = gainCurrentButton_stm32;
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x4005, Modbus.holdingRegisterSize);		// decode param power active coef
	addressSlave[1] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x4006, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[addressSlave[0]] = byte32High((uint32_t)(powerCoefActive*1000000000));
	Modbus.holdingRegisterValue[addressSlave[1]] = byte32Low((uint32_t)(powerCoefActive*1000000000));
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x5005, Modbus.holdingRegisterSize);		// decode param power reactive coef
	addressSlave[1] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x5006, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[addressSlave[0]] = byte32High((uint32_t)(powerCoefReactive*1000000000));
	Modbus.holdingRegisterValue[addressSlave[1]] = byte32Low((uint32_t)(powerCoefReactive*1000000000));
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x6005, Modbus.holdingRegisterSize);		// decode param power apparent coef
	addressSlave[1] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x6006, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[addressSlave[0]] = byte32High((uint32_t)(powerCoefApparent*1000000000));
	Modbus.holdingRegisterValue[addressSlave[1]] = byte32Low((uint32_t)(powerCoefApparent*1000000000));
}

void eepromLoop(){
	eepromTimerDelta = HAL_GetTick() - eepromTimer;
	if(stateConfig || (eepromTimerDelta > INTERVAL_EEPROM)){
		eepromTimer = HAL_GetTick();
		// UPDATE VALUE SETTING PARAMETER VIA BUTTON
		if(stateConfigButton){
			// slave address
			uint8_t address;
			address = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x1000, Modbus.holdingRegisterSize);
			Modbus.holdingRegisterValue[address] = Modbus.slaveAddrSlaveSecond;
			// gain Current A B C  >> 0x200A 0x200B 0x200C
			address = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x4004, Modbus.holdingRegisterSize);
			Modbus.holdingRegisterValue[address] = gainCurrentButton_stm32;
			stateConfigButton = 0;
		}
		// RE-READING POWER METER FOR NEW CONFIG
		powerMultiReadSensor(addrSensor, valueSensor, valueFloat, 32);
		powerSplitValue();
		phase = PHASE_RST;
		stateConfig = 0;

		// ENCODE DATA & WRITE EEPROM
		eepromEncode(
				energyActiveA_uint,				energyActiveB_uint,						energyActiveC_uint,
				energyReactiveA_uint,			energyReactiveB_uint,					energyReactiveC_uint,
				offsetVolt_ht7036,				offsetCurr_ht7036,						gainVolt_ht7036,							gainCurr_ht7036,
				offsetVolt_stm32,				offsetCurr_stm32,						gainVolt_stm32,								gainCurr_stm32,
				Modbus.slaveAddrSlaveSecond, 	calibPF_ht7036,							(uint16_t)(gainPF_stm32*10000), 			(uint16_t)(offsetPF_stm32*10000),
				gainCurrentButton_stm32,		(uint32_t)(powerCoefActive*1000000000),	(uint32_t)(powerCoefReactive*1000000000),	(uint32_t)(powerCoefApparent*1000000000)
		);
		ee24_write(0, (uint8_t*)eepromBufferWrite, sizeof(eepromBufferWrite), 1000);
	}
}

void eepromEncode(
			uint64_t energyActiveA,				// indeks 0 - 7
			uint64_t energyActiveB,				// indeks 8 - 15
			uint64_t energyActiveC,				// indeks 16 - 23
			uint64_t energyReactiveA,			// indeks 24 - 31
			uint64_t energyReactiveB,			// indeks 32 - 39
			uint64_t energyReactiveC,			// indeks 40 - 47
			uint16_t offsetVolt_ht7036,			// indeks 48 - 49
			uint16_t offsetCurr_ht7036,			// indeks 50 - 51
			uint16_t gainVolt_ht7036,			// indeks 52 - 53
			uint16_t gainCurr_ht7036,			// indeks 54 - 55
			uint16_t offsetVolt_stm32,			// indeks 56 - 57
			uint16_t offsetCurr_stm32,			// indeks 58 - 59
			uint16_t gainVolt_stm32,			// indeks 60 - 62
			uint16_t gainCurr_stm32,			// indeks 62 - 63
			uint16_t slaveAddress,				// indeks 64 - 65
			uint16_t calibPF_ht7036,			// indeks 66 - 67
			uint16_t gainPF_stm32,				// indeks 68 - 69
			uint16_t offsetPF_stm32,			// indeks 70 - 71
			uint16_t gainCurrentButton_stm32,	// indeks 72 - 73
			uint32_t powerCoefActive,			// indeks 74 - 77
			uint32_t powerCoefReactive,			// indeks 78 - 81
			uint32_t powerCoefApparent			// indeks 82 - 85
	){
	uint8_t buffer8[8];
	uint8_t indeksBuffer=0;
	// ENCCODE ENERGY ACTIVE A
	uint64ToUint8(buffer8, energyActiveA);
	for(uint8_t indeks=0;indeks<8;indeks++){eepromBufferWrite[indeks] = buffer8[indeksBuffer++];}
	// ENCCODE ENERGY ACTIVE B
	uint64ToUint8(buffer8, energyActiveB);
	indeksBuffer = 0;
	for(uint8_t indeks=8;indeks<16;indeks++){eepromBufferWrite[indeks] = buffer8[indeksBuffer++];}
	// ENCCODE ENERGY ACTIVE C
	uint64ToUint8(buffer8, energyActiveC);
	indeksBuffer = 0;
	for(uint8_t indeks=16;indeks<24;indeks++){eepromBufferWrite[indeks] = buffer8[indeksBuffer++];}
	// ENCCODE ENERGY REACTIVE A
	uint64ToUint8(buffer8, energyReactiveA);
	indeksBuffer = 0;
	for(uint8_t indeks=24;indeks<32;indeks++){eepromBufferWrite[indeks] = buffer8[indeksBuffer++];}
	// ENCCODE ENERGY REACTIVE B
	uint64ToUint8(buffer8, energyReactiveB);
	indeksBuffer = 0;
	for(uint8_t indeks=32;indeks<40;indeks++){eepromBufferWrite[indeks] = buffer8[indeksBuffer++];}
	// ENCCODE ENERGY REACTIVE C
	uint64ToUint8(buffer8, energyReactiveC);
	indeksBuffer = 0;
	for(uint8_t indeks=40;indeks<48;indeks++){eepromBufferWrite[indeks] = buffer8[indeksBuffer++];}
	// ENCODE OFFSET VOLT HT3036
	eepromBufferWrite[48] = byte16High(offsetVolt_ht7036);
	eepromBufferWrite[49] = byte16Low(offsetVolt_ht7036);
	// ENCODE OFFSET CURRENT  HT3036
	eepromBufferWrite[50] = byte16High(offsetCurr_ht7036);
	eepromBufferWrite[51] = byte16Low(offsetCurr_ht7036);
	// ENCODE GAIN VOLT HT3036
	eepromBufferWrite[52] = byte16High(gainVolt_ht7036);
	eepromBufferWrite[53] = byte16Low(gainVolt_ht7036);
	// ENCODE GAIN CURRENT  HT3036
	eepromBufferWrite[54] = byte16High(gainCurr_ht7036);
	eepromBufferWrite[55] = byte16Low(gainCurr_ht7036);
	// ENCODE OFFSET VOLT STM32
	eepromBufferWrite[56] = byte16High(offsetVolt_stm32);
	eepromBufferWrite[57] = byte16Low(offsetVolt_stm32);
	// ENCODE OFFSET CURRENT STM32
	eepromBufferWrite[58] = byte16High(offsetCurr_stm32);
	eepromBufferWrite[59] = byte16Low(offsetCurr_stm32);
	// ENCODE GAIN VOLT STM32
	eepromBufferWrite[60] = byte16High(gainVolt_stm32);
	eepromBufferWrite[61] = byte16Low(gainVolt_stm32);
	// ENCODE GAIN CURRENT  STM32
	eepromBufferWrite[62] = byte16High(gainCurr_stm32);
	eepromBufferWrite[63] = byte16Low(gainCurr_stm32);
	// ENCODE SLAVE ADDRESS MODBUS
	eepromBufferWrite[64] = byte16High(slaveAddress);
	eepromBufferWrite[65] = byte16Low(slaveAddress);
	// WIRING TYPE
	eepromBufferWrite[66] = byte16High(calibPF_ht7036);
	eepromBufferWrite[67] = byte16Low(calibPF_ht7036);
	// GAIN POWER FACTOR
	eepromBufferWrite[68] = byte16High(gainPF_stm32);
	eepromBufferWrite[69] = byte16Low(gainPF_stm32);
	// OFFSET POWER FACTOR
	eepromBufferWrite[70] = byte16High(offsetPF_stm32);
	eepromBufferWrite[71] = byte16Low(offsetPF_stm32);
	// gain button stm32
	eepromBufferWrite[72] = byte16High(gainCurrentButton_stm32);
	eepromBufferWrite[73] = byte16Low(gainCurrentButton_stm32);
	// decode param power coef active
	eepromBufferWrite[74] = byte16High(byte32High(powerCoefActive));
	eepromBufferWrite[75] = byte16Low(byte32High(powerCoefActive));
	eepromBufferWrite[76] = byte16High(byte32Low(powerCoefActive));
	eepromBufferWrite[77] = byte16Low(byte32Low(powerCoefActive));
	// decode param power coef reactive
	eepromBufferWrite[78] = byte16High(byte32High(powerCoefReactive));
	eepromBufferWrite[79] = byte16Low(byte32High(powerCoefReactive));
	eepromBufferWrite[80] = byte16High(byte32Low(powerCoefReactive));
	eepromBufferWrite[81] = byte16Low(byte32Low(powerCoefReactive));
	// decode param power coef apparnet
	eepromBufferWrite[82] = byte16High(byte32High(powerCoefApparent));
	eepromBufferWrite[83] = byte16Low(byte32High(powerCoefApparent));
	eepromBufferWrite[84] = byte16High(byte32Low(powerCoefApparent));
	eepromBufferWrite[85] = byte16Low(byte32Low(powerCoefApparent));
}

void powerSplitValue(){

	// GETTING DATA FROM FLOAT ARRAY TO FLOAT32 (GENERAL GROUP SENSOR) >> DECODE
	rmsVoltageA = valueFloat[0];			rmsVoltageB = valueFloat[1];			rmsVoltageC = valueFloat[2];		rmsVoltageVector = valueFloat[3];
	rmsCurrentA = valueFloat[4];			rmsCurrentB = valueFloat[5];			rmsCurrentC = valueFloat[6];		rmsCurrentVector = valueFloat[7];
	powerActiveA = valueFloat[8];			powerActiveB = valueFloat[9];			powerActiveC = valueFloat[10];		PowerActiveCombine = valueFloat[11];
	powerReactiveA = valueFloat[12];		powerReactiveB = valueFloat[13];		powerReactiveC = valueFloat[14];	powerReactiveCombine = valueFloat[15];
	powerApparentA = valueFloat[16];		powerApparentB = valueFloat[17];		powerApparentC = valueFloat[18];	powerApparentCombine = valueFloat[19];
	powerFactorA = valueFloat[20];			powerFactorB = valueFloat[21];			powerFactorC = valueFloat[22];		powerFactorCombine = valueFloat[23];
	energyActiveA = valueFloat[24];			energyActiveB = valueFloat[25];			energyActiveC = valueFloat[26];		energyActiveCombine = valueFloat[27];
	energyReactiveA = valueFloat[28];		energyReactiveB = valueFloat[29];		energyReactiveC = valueFloat[30];	energyReactiveCombine = valueFloat[31];
	// GETTING DATA FROM FLOAT ARRAY TO FLOAT32 (ENERGY GROUP SENSOR) >> DECODE
	energyActiveA_uint = energyModbus[0];	energyActiveB_uint = energyModbus[1];	energyActiveC_uint = energyModbus[2];
	energyReactiveA_uint = energyModbus[4];	energyReactiveB_uint = energyModbus[5];	energyReactiveC_uint = energyModbus[6];
	energyModbus[3] = energyActiveA_uint + energyActiveB_uint + energyActiveC_uint;
	energyModbus[7] = energyReactiveA_uint + energyReactiveB_uint + energyReactiveC_uint;
	// DATA PROCESSING
	powerHandleCalib();
	powerHandleTresholdGroup();
	// CALCULATE AND GET VOLTAGE DIFFRENCE
	rmsVoltageAB = calcVoltDif(rmsVoltageA, rmsVoltageB);
	rmsVoltageBC = calcVoltDif(rmsVoltageB, rmsVoltageC);
	rmsVoltageCA = calcVoltDif(rmsVoltageC, rmsVoltageA);
	// GETTING DATA FROM FLOAT32 TO FLOAT ARRAY (GENERAL GROUP SENSOR) >> ENCODE
	valueFloat[0] = rmsVoltageA;					valueFloat[1] = rmsVoltageB;					valueFloat[2] = rmsVoltageC;				valueFloat[3] = rmsVoltageVector;
	valueFloat[4] = rmsCurrentA;					valueFloat[5] = rmsCurrentB;					valueFloat[6] = rmsCurrentC;				valueFloat[7] = rmsCurrentVector;
	valueFloat[8] = powerActiveA;					valueFloat[9] = powerActiveB;					valueFloat[10] = powerActiveC;				valueFloat[11] = (powerActiveA + powerActiveB + powerActiveC);
	valueFloat[12] = powerReactiveA;				valueFloat[13] = powerReactiveB;				valueFloat[14] = powerReactiveC;			valueFloat[15] = (powerReactiveA + powerReactiveB + powerReactiveC);
	valueFloat[16] = powerApparentA;				valueFloat[17] = powerApparentB;				valueFloat[18] = powerApparentC;			valueFloat[19] = powerApparentCombine; // CALCULATE MANUAL APPARENT VALUE
	valueFloat[20] = powerFactorA;					valueFloat[21] = powerFactorB;					valueFloat[22] = powerFactorC;				valueFloat[23] = powerFactorCombine;
	valueFloat[24] = energyActiveA;					valueFloat[25] = energyActiveB;					valueFloat[26] = energyActiveC;				valueFloat[27] = energyActiveCombine;
	valueFloat[28] = energyReactiveA;				valueFloat[29] = energyReactiveB;				valueFloat[30] = energyReactiveC;			valueFloat[31] = energyReactiveCombine;
}

void powerHandleTresholdGroup(){
	// HANDLE TRESHOLD FOR ZEROIING
	// RMS VOLAGE
	powerHandleTreshold(&rmsVoltageA,0.5,-0.5);
	powerHandleTreshold(&rmsVoltageB,0.5,-0.5);
	powerHandleTreshold(&rmsVoltageC,0.5,-0.5);
	powerHandleTreshold(&rmsVoltageVector,0.5,-0.5);
	// RMS VOLTAGE DIV
	powerHandleTreshold(&rmsVoltageAB,0.5,-0.5);
	powerHandleTreshold(&rmsVoltageBC,0.5,-0.5);
	powerHandleTreshold(&rmsVoltageCA,0.5,-0.5);
	// RMS CURRENT
	powerHandleTreshold(&rmsCurrentA,0.009,-0.009);
	powerHandleTreshold(&rmsCurrentB,0.009,-0.009);
	powerHandleTreshold(&rmsCurrentC,0.009,-0.009);
	powerHandleTreshold(&rmsCurrentVector,0.009,-0.009);
	// POWER ACTIVE
	powerHandleTreshold(&powerActiveA,0.0005,-0.0005);
	powerHandleTreshold(&powerActiveB,0.0005,-0.0005);
	powerHandleTreshold(&powerActiveC,0.0005,-0.0005);
	powerHandleTreshold(&PowerActiveCombine,0.0005,-0.0005);
	// POWER REACTIVE
	powerHandleTreshold(&powerReactiveA,0.0005,-0.0005);
	powerHandleTreshold(&powerReactiveB,0.0005,-0.0005);
	powerHandleTreshold(&powerReactiveC,0.0005,-0.0005);
	powerHandleTreshold(&powerReactiveCombine,0.0005,-0.0005);
	// POWER APPARENT
	powerHandleTreshold(&powerApparentA,0.0005,-0.0005);
	powerHandleTreshold(&powerApparentB,0.0005,-0.0005);
	powerHandleTreshold(&powerApparentC,0.0005,-0.0005);
	powerHandleTreshold(&powerApparentCombine,0.0005,-0.0005);
	// NOT USED >> REPLACE ON ModbusValueUPdate();
	// ENERGY ACTIVE TOTAL
	// if(energyActiveCombine == 0)energyActiveCombine = ZERO_VAL;
	// ENERGY REACTIVR TOTAL
	// if(energyReactiveCombine == 0)energyReactiveCombine = ZERO_VAL;
}

void powerHandleTreshold(float * data, float max, float min){
	//if((*data<max)&&(*data>min))*data=ZERO_VAL;
	if((*data<max)&&(*data>min))*data=0;
}

void powerHandleCalib(){
	if(phase != PHASE_RST){
		// CALIB PHASE A
		if(phase == PHASE_A){
			if(addressModbus == 0x2001)offsetVoltage = offsetVoltageA;
			if(addressModbus == 0x2004)offsetCurrent = offsetCurrentA;
			if(addressModbus == 0x2007)gainVoltage = gainVoltageA;
			if(addressModbus == 0x200A)gainCurrent = gainCurrentA;
		}
		// CALIB PHASE B
		if(phase == PHASE_B){
			if(addressModbus == 0x2002)offsetVoltage = offsetVoltageB;
			if(addressModbus == 0x2005)offsetCurrent = offsetCurrentB;
			if(addressModbus == 0x2008)gainVoltage = gainVoltageB;
			if(addressModbus == 0x200B)gainCurrent = gainCurrentB;
		}
		// CALIB PHASE C
		if(phase == PHASE_C){
			if(addressModbus == 0x2003)offsetVoltage = offsetVoltageC;
			if(addressModbus == 0x2006)offsetCurrent = offsetCurrentC;
			if(addressModbus == 0x2009)gainVoltage = gainVoltageC;
			if(addressModbus == 0x200C)gainCurrent = gainCurrentC;
		}
	}
	// CALIBRAITION VOLTAGE
	rmsVoltageA = rmsVoltageA*gainVoltage + offsetVoltage;
	rmsVoltageB = rmsVoltageB*gainVoltage + offsetVoltage;
	rmsVoltageC = rmsVoltageC*gainVoltage + offsetVoltage;

	// CALIBRATION CURRENT
	rmsCurrentA = rmsCurrentA*gainCurrent*gainCurrentButton_stm32 + offsetCurrent;
	rmsCurrentB = rmsCurrentB*gainCurrent*gainCurrentButton_stm32 + offsetCurrent;
	rmsCurrentC = rmsCurrentC*gainCurrent*gainCurrentButton_stm32 + offsetCurrent;

	// MODIFY BEGIN
	valueFloat[0] = rmsVoltageA;
	valueFloat[1] = rmsVoltageB;
	valueFloat[2] = rmsVoltageC;

	valueFloat[4] = rmsCurrentA;
	valueFloat[5] = rmsCurrentB;
	valueFloat[6] = rmsCurrentC;
	// MODIFY END
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){
	if(htim == &htim14){
		// SWITCH FROM DISPLAY MODE TO SETTING MODE
		if(buttonStatus == BTN_NEXT){
			uint8_t timerCount = 0;
			uint32_t timer = HAL_GetTick();
			while(HAL_GPIO_ReadPin(BTN_Next_GPIO_Port, BTN_Next_Pin) == GPIO_PIN_RESET){
				if(HAL_GetTick()-timer >= 1000){
					timerCount += 1;
					timer = HAL_GetTick();
				}
				if(timerCount >= 3){
					flagGetDataOld = 1;
					menuLevel = MENU_LEVEL_1;
					paramLv1 = 0;
					buttonStatus = BTN_IDLE;
					break;
				}
			}
		}
		buttonAntiBounce = 0;
		HAL_TIM_Base_Stop_IT(&htim14);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIOPin){
	// ANTI BOUNCING
	if(buttonAntiBounce){
		return;
	}
	// TRIGGER BUTTON
	if((GPIOPin == BTN_Next_Pin) || (GPIOPin == BTN_Up_Pin) || (GPIOPin == BTN_Set_Pin) || (GPIOPin == BTN_Enter_Pin)){
		// HANDLE BACKLIGHT
		backlightState = 1;
		backlightTimer = HAL_GetTick();
		HAL_GPIO_WritePin(BACKLIGHT_En_GPIO_Port, BACKLIGHT_En_Pin, GPIO_PIN_SET);

		buttonAntiBounce = 1;
		buttonTrigger=1;
		HAL_TIM_Base_Start_IT(&htim14);
		// BUTTON NEXT " >> "
		if(GPIOPin == BTN_Next_Pin){buttonStatus = BTN_NEXT;}
		// BUTTON UP " ^ "
		else if(GPIOPin == BTN_Up_Pin){buttonStatus = BTN_UP;}
		// BUTTON SET
		else if(GPIOPin == BTN_Set_Pin){buttonStatus = BTN_SET;}
		// BUTTON ENTER
		else if(GPIOPin == BTN_Enter_Pin){buttonStatus = BTN_ENTER;}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* A. NOTE
 * 	Group Power 			>> 4 byte Sign 		>> division 100
 * 	Group RMS				>> 2 byte unsgin	>> division 100
 * 	Gruop power factor 		>> 2 byte unsign	>> division 100
 * 	Group energy 			>> 4 byte unsign 	>> division 100
 *
 * B. MAPPING REGISTER EXISTING
 * 		V A-N(VAB)				>> 3027, 3028		>> (int32_t)float32
 * 		V B-N(VBC)				>> 3029, 3030		>> (int32_t)float32
 * 		V C-N(VCA)				>> 3031, 3032		>> (int32_t)float32
 * 		I A						>> 2999, 3000		>> (int32_t)float32
 * 		I B						>> 3001, 3002		>> (int32_t)float32
 * 		I C						>> 3003, 3004		>> (int32_t)float32
 * 		active Pow A			>> 3053, 3054		>> (int32_t)float32
 * 		active Pow B			>> 3055, 3056		>> (int32_t)float32
 * 		active Pow C			>> 3057, 3058		>> (int32_t)float32
 * 		active Pow Tot			>> 3059, 3060		>> (int32_t)float32
 *X		THD V-A					>> 21329, 21330		>> (int32_t)float32 // total harmonic distorision (%)
 *X		THD V-B					>> 21331, 21332		>> (int32_t)float32
 *X		THD V-C					>> 21333, 21334		>> (int32_t)float32
 * 		PF A					>> 3077, 3078		>> (int32_t)4Q FP PF // power factor
 *	 	PF B					>> 3079, 3080		>> (int32_t)4Q FP PF
 *	 	PF C 					>> 3081, 3082		>> (int32_t)4Q FP PF
 * 		Reactive Energy	(VARH) 	>> 3219, 3220, 3221, 3222		>> (int64_t)int64	// Reactive Energy Delivered
 * 		Energy (WH)				>> 3203, 3204, 3205, 3206		>> (int64_t)int64	// Active Energy Delivered (Into Load)
 *	 	V A-B 					>> 3019, 3020		>> (int32_t)float32 // ((V A + V B)/2)*sqr(1/2)  | 1,4142135623730950488016887242097 >> akar2 dari 2
 *	 	V B-C 					>> 3021, 3022		>> (int32_t)float32	//
 *		V C-A 					>> 3023, 3024		>> (int32_t)float32
 */
/*
--------------group sensor--------------

| Address | Description               | size   |
| ------- | ------------------------- | ------ |
| 0       | active energy A           | 8 byte |
| 8       | active energy B           | 8 byte |
| 16      | active energy C           | 8 byte |
| 24      | reactive energy A         | 8 byte |
| 32      | reactive energy B         | 8 byte |
| 40      | reactive energy C         | 8 byte |

-------------group calibration------------------

| Address | Description               | size   |
| ------- | ------------------------- | ------ |
| 48      | offset Voltage super User | 2 byte |
| 50      | offset current super user | 2 byte |
| 52      | gain voltage super user   | 2 byte |
| 54      | gain current super user   | 2 byte |
| 56      | offset Voltage User       | 2 byte |
| 58      | offset current user       | 2 byte |
| 60      | gain voltage user         | 2 byte |
| 62      | gain current user         | 2 byte |

-------group Other Sensor---------

| Address | Description               | size   |
| ------- | ------------------------- | ------ |
| 64      | Slave ID                  | 2 byte |
| 66      | Wiring type               | 2 byte |


 * -------------schema write and read------------------
 * encode buffer(grouping data) >> 64 byte
 * decode buffer(split data)
 * data Frame buffer(type:uint8_t[64 indeks array]) in EEPROM >> Based on table "group sensor & group calibration"
 */
