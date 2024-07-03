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
#include "memory.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INTERVAL_EEPROM 	120000
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
HAL_StatusTypeDef spiStatus[16];
extern float HFconstVal;
extern float ECVal;
extern float ECDef;
extern uint64_t powerTimer;
uint64_t rstPowerTimer;
uint64_t rstPowerTimerDelta;
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
// LINE VOLTAGE
float 	rmsVoltageAB,		rmsVoltageBC,		rmsVoltageCA;

// HANDLING OFFSET
uint64_t offsetEnergyActive, offsetEnergyReactive;

extern uint64_t energyModbus[8];
extern double bufferEnergySUM[8];
extern uint32_t check;

uint64_t 	energyActiveA_uint,		energyActiveB_uint, 		energyActiveC_uint,		energyActiveCombine_uint,
			energyReactiveA_uint,	energyReactiveB_uint, 		energyReactiveC_uint,	energyReactiveCombine_uint;



float	gainVoltageA = 1,	gainVoltageB = 1,	gainVoltageC = 1,
		gainCurrentA = 1,	gainCurrentB = 1,	gainCurrentC = 1,
		gainVoltage = 1,	gainCurrent = 1;

float 	offsetVoltageA = 0, 	offsetVoltageB = 0,		offsetVoltageC = 0,
		offsetCurrentA = 0,		offsetCurrentB = 0,		offsetCurrentC = 0,
		offsetVoltage = 0,		offsetCurrent = 0, 		offsetPF_stm32 = 0,
		gainPF_stm32 = 0,
		powerCoefActiveA = 0,	powerCoefActiveB = 0,	powerCoefActiveC = 0,
		powerCoefReactiveA = 0,	powerCoefReactiveB = 0,	powerCoefReactiveC = 0,
		powerCoefApparentA = 0,	powerCoefApparentB = 0,	powerCoefApparentC = 0;

uint16_t offsetVolt_ht7036,	offsetCurr_ht7036,	gainVolt_ht7036,	gainCurr_ht7036,
		 gainVolt_stm32,		gainCurr_stm32,
		 calibPF_ht7036,	gainCurrentButton_stm32,
		 idHt7036;

int16_t offsetVolt_stm32, offsetCurr_stm32;

uint32_t powerApparentBitA,	powerApparentBitB,	powerApparentBitC;


//------------------------- GROUP VARIABLE MODBUS ---------------------------------
extern MODBUS Modbus;						// ADDRESS REGISTER VALUE POWER SENSOR >> 92 Register
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
											// [RAW DATA] ADDRESS REGISTER PARAMETER CALIBRATION POWER SENSOR FOR SUPER USER >> 5 Register
											0x3000,													// ID IC HT7036
											0x3001,		 											// offset Voltage super User
											0x3002, 												// offset current super user (2 byte) >> HT7036
											0x3003, 												// gain voltage super user (2 byte) >> HT7036
											0x3004,													// gain current super user (2 byte) >> HT7036
											// ADDRESS REGISTER PARAMETER POWER METER CALIBRTION SUPER USER & USER >> 3 register
											0x4001,													// power phase corrction for super user >> HT7036
											0x4002,													// gain power factor for user >> STM32
											0x4003,													// offset power factpr for user >> STM32
											// ADDRESS REGISTER PARMATER CALIBRATION GAIN CURRENT VIA BUTTON SET & DECODE PARAM GROUP POWER >> 19 register
											0x4004,													// gain current button stm32
											0x4005, 0x4006, 0x4007, 0x4008, 0x4009, 0x400A,			// decode param group power active register abc >> stm32
											0x5005, 0x5006, 0x5007, 0x5008, 0x5009, 0x500A,			// decode param group power reactive register abc >> stm32
											0x6005, 0x6006, 0x6007, 0x6008, 0x6009, 0x600A			// decode param group power apparent register abc >> stm32
};
											// VALUE REGISTER POWER SENSOR
//uint16_t holdingRegisterSize = (uint16_t)sizeof(holdingRegisterAddress)/sizeof(uint16_t);
uint16_t holdingRegisterSize = 144;
uint16_t holdingRegisterValue[144]	= {0};
extern uint16_t addressModbus;

//------------------------- GROUP VARIABLE EEPROM EXTERNAL 8K ---------------------------------
uint8_t eepromSize = 136;
uint8_t eepromBufferRead[136]; // before modify >> 134
uint8_t eepromBufferWrite[136]; // before modify >> 134
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
//---------------------------- GROUP VARIABLE FLASH MEMORY -----------------------------------
extern uint32_t flashAddrVirtual;
uint32_t bufferFlash[5];
uint8_t timerFlash=0;
// TESTING BEGIN
//uint32_t bufferWrite[5];
// TESTING END

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
		int16_t offsetVolt_stm32,			// indeks 56 - 57
		int16_t offsetCurr_stm32,			// indeks 58 - 59
		uint16_t slaveAddress,				// indeks 60 - 61
		uint16_t calibPF_ht7036,			// indeks 62 - 63
		uint16_t gainPF_stm32,				// indeks 64 - 65
		uint16_t offsetPF_stm32,			// indeks 66 - 67
		uint16_t gainCurrentButton_stm32,	// indeks 68 - 69
		uint32_t powerCoefActiveA,			// indeks 70 - 73
		uint32_t powerCoefActiveB,			// indeks 74 - 77
		uint32_t powerCoefActiveC,			// indeks 78 - 81
		uint32_t powerCoefReactiveA,		// indeks 82 - 85
		uint32_t powerCoefReactiveB,		// indeks 86 - 89
		uint32_t powerCoefReactiveC,		// indeks 90 - 93
		uint32_t powerCoefApparentA,		// indeks 94 - 97
		uint32_t powerCoefApparentB,		// indeks 98 - 101
		uint32_t powerCoefApparentC,		// indeks 102 - 105
		uint16_t gainVoltageA,				// indeks 106 - 107
		uint16_t gainVoltageB,				// indeks 108 - 109
		uint16_t gainVoltageC,				// indeks 110 - 111
		uint16_t gainCurrentA,				// indeks 112 - 113
		uint16_t gainCurrentB,				// indeks 114 - 115
		uint16_t gainCurrentC,				// indeks 116 - 117
		uint64_t offsetEnergyActive,		// indeks 118 - 125
		uint64_t offsetEnergyReactive		// indeks 126 - 133
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
void decodeGain(uint8_t eepromBufferRead[], float* gainData, int index, float defaultValue);
void handlingPowerInit();
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
  // CLEAR MEMORY
  ee24_eraseChip();
  for(;;);
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
	  // HANDLING INIT VALUE
	  handlingPowerInit();
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
	// powerRestoreCalib();
	powerSetup(address,addressData,spiStatus,12);
	powerMultiReadSensor(addrSensor, valueSensor, valueFloat, 32);
	powerSplitValue();
	// INITIAL TIMER SAMPLING POWER & EEPROM
	powerTimer = eepromTimer = rstPowerTimer =  HAL_GetTick();
}

void modbusValueUpdate(){
	float bufferFloat;
	uint32_t bufferUnsign32;
	uint64_t bufferUnsign64;
	uint8_t address = 0;
	uint16_t byteHigh,byteLow,address16;
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
	// ID IC HT7-36
	address16 = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x3000, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[address16] = idHt7036;
}

void powerCalibLoop(){
	if(Modbus.trigState){
		uint16_t addressSlave;
		uint8_t addressIndeks;
		uint16_t dataCalib16;
		uint32_t dataCalib32;
		int16_t bufferInt16;
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
			// -----------------------------------------------------------SLAVE ADDRESS-------------------------------------------------------------------------
			if(addressModbus == 0x1000){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				Modbus.slaveAddrSlaveSecond = Modbus.holdingRegisterValue[addressSlave];
			}
			// --------------------------------------------------CALIBRATION HT7036 FOR SUPER USER------------------------------------------------------------------
			// FILTER REGISTER OFFSET VOLTAGE RMS
			if((addressModbus == 0x1001) || (addressModbus == 0x1002) || (addressModbus == 0x1003) || (addressModbus == 0x1004) || (addressModbus == 0x1005) || (addressModbus == 0x1006) || (addressModbus == 0x1007) || (addressModbus == 0x1008) || (addressModbus == 0x1009) || (addressModbus == 0x100A) || (addressModbus == 0x100B) || (addressModbus == 0x100C)){
				// GET DATA MODBUS
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				dataCalib16 = Modbus.holdingRegisterValue[addressSlave];
				dataCalib32 = (uint32_t)dataCalib16;
				stateConfig = Modbus.trigState;
			}

			if((addressModbus == 0x1001) || (addressModbus == 0x1002) || (addressModbus == 0x1003)){
				// GET ADDRESS REGISTER PARAMETER CALIBRATION [RAW DATA]
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x3001, Modbus.holdingRegisterSize);
				if(addressModbus == 0x1001){offsetVolt_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(VRMS_OFFSET, w_UaRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],offsetVolt_ht7036);phase=PHASE_A;}
				else if(addressModbus == 0x1002){offsetVolt_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(VRMS_OFFSET, w_UbRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],offsetVolt_ht7036);phase=PHASE_B;}
				else if(addressModbus == 0x1003){offsetVolt_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(VRMS_OFFSET, w_UcRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],offsetVolt_ht7036);phase=PHASE_C;}
			}
			// FILTER REGISTER OFFSET CURRENT RMS
			else if((addressModbus == 0x1004) || (addressModbus == 0x1005) || (addressModbus == 0x1006)){
				// GET ADDRESS REGISTER PARAMETER CALIBRATION [RAW DATA]
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x3002, Modbus.holdingRegisterSize);
				if(addressModbus == 0x1004){offsetCurr_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(IRMS_OFFSET, w_IaRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],offsetCurr_ht7036);phase=PHASE_A;}
				else if(addressModbus == 0x1005){offsetCurr_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(IRMS_OFFSET, w_IbRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],offsetCurr_ht7036);phase=PHASE_B;}
				else if(addressModbus == 0x1006){offsetCurr_ht7036 = Modbus.holdingRegisterValue[addressSlave] = powerSingleRecalib(IRMS_OFFSET, w_IcRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0],offsetCurr_ht7036);phase=PHASE_C;}
			}
			// FILTER REGISTER GAIN VOLTAGE RMS
			else if((addressModbus == 0x1007) || (addressModbus == 0x1008) || (addressModbus == 0x1009)){
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
				offsetVolt_stm32 = bufferInt16 = (int16_t)Modbus.holdingRegisterValue[addressSlave];
				if(addressModbus == 0x2001){offsetVoltageA = (float)bufferInt16/1000;phase=PHASE_A;}
				if(addressModbus == 0x2002){offsetVoltageB = (float)bufferInt16/1000;phase=PHASE_B;}
				if(addressModbus == 0x2003){offsetVoltageC = (float)bufferInt16/1000;phase=PHASE_C;}
			}
			// FILTER REGISTER OFFSET CURRENT RMS
			else if ((addressModbus == 0x2004) || (addressModbus == 0x2005) || (addressModbus == 0x2006)){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				offsetCurr_stm32 = bufferInt16 = (int16_t)Modbus.holdingRegisterValue[addressSlave];
				if(addressModbus == 0x2004){offsetCurrentA = (float)bufferInt16/1000;phase=PHASE_A;}
				if(addressModbus == 0x2005){offsetCurrentB = (float)bufferInt16/1000;phase=PHASE_B;}
				if(addressModbus == 0x2006){offsetCurrentC = (float)bufferInt16/1000;phase=PHASE_C;}
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
				bufferInt16 = (int16_t)Modbus.holdingRegisterValue[addressSlave];
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

			// scan for address array modbus
			addressSlave = modbusGetIndeks( Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);

			// trigger for trigstate >> EEPROM handling
			if(		(addressModbus == 3517) || (addressModbus == 3521) ||
					(addressModbus == 3525) || (addressModbus == 3203) ||
					(addressModbus == 3529) || (addressModbus == 3533) ||
					(addressModbus == 3537) || (addressModbus == 3219) ||
					(addressModbus == 0x4005) || (addressModbus == 0x4006) ||
					(addressModbus == 0x4007) || (addressModbus == 0x4008) ||
					(addressModbus == 0x4009) || (addressModbus == 0x400A) ||
					(addressModbus == 0x5005) || (addressModbus == 0x5006) ||
					(addressModbus == 0x5007) || (addressModbus == 0x5008) ||
					(addressModbus == 0x5009) || (addressModbus == 0x500A) ||
					(addressModbus == 0x6005) || (addressModbus == 0x6006) ||
					(addressModbus == 0x6007) || (addressModbus == 0x6008) ||
					(addressModbus == 0x6009) || (addressModbus == 0x600A) ||
					(addressModbus == 0x2007) || (addressModbus == 0x2008) ||
					(addressModbus == 0x2009) || (addressModbus == 0x200A) ||
					(addressModbus == 0x200B) || (addressModbus == 0x200C)){
				stateConfig = Modbus.trigState;
			}

			if((addressModbus == 3517) || (addressModbus == 3521) || (addressModbus == 3525) || (addressModbus == 3203) || (addressModbus == 3529) || (addressModbus == 3533) || (addressModbus == 3537) || (addressModbus == 3219)){
				addressModbusBuffer = addressModbus;
				for(uint8_t i=0;i<4;i++){
					addressSlaveArray[i] = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbusBuffer+i, Modbus.holdingRegisterSize);
					buffer16[i] = Modbus.holdingRegisterValueRX[i];
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
			}else{
				// handling modbus power active A(0x4005, 0x4006), B(0x4007, 0x4008), C(0x4009, 0x400A)
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x4005, Modbus.holdingRegisterSize);
				powerCoefActiveA =(float)(uint16ToUint32(Modbus.holdingRegisterValue[addressSlave], Modbus.holdingRegisterValue[addressSlave+1]))/1000000000;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x4007, Modbus.holdingRegisterSize);
				powerCoefActiveB =(float)(uint16ToUint32(Modbus.holdingRegisterValue[addressSlave], Modbus.holdingRegisterValue[addressSlave+1]))/1000000000;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x4009, Modbus.holdingRegisterSize);
				powerCoefActiveC =(float)(uint16ToUint32(Modbus.holdingRegisterValue[addressSlave], Modbus.holdingRegisterValue[addressSlave+1]))/1000000000;

				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x5005, Modbus.holdingRegisterSize);
				powerCoefReactiveA =(float)(uint16ToUint32(Modbus.holdingRegisterValue[addressSlave], Modbus.holdingRegisterValue[addressSlave+1]))/1000000000;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x5007, Modbus.holdingRegisterSize);
				powerCoefReactiveB =(float)(uint16ToUint32(Modbus.holdingRegisterValue[addressSlave], Modbus.holdingRegisterValue[addressSlave+1]))/1000000000;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x5009, Modbus.holdingRegisterSize);
				powerCoefReactiveC =(float)(uint16ToUint32(Modbus.holdingRegisterValue[addressSlave], Modbus.holdingRegisterValue[addressSlave+1]))/1000000000;

				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x6005, Modbus.holdingRegisterSize);
				powerCoefApparentA =(float)(uint16ToUint32(Modbus.holdingRegisterValue[addressSlave], Modbus.holdingRegisterValue[addressSlave+1]))/1000000000;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x6007, Modbus.holdingRegisterSize);
				powerCoefApparentB =(float)(uint16ToUint32(Modbus.holdingRegisterValue[addressSlave], Modbus.holdingRegisterValue[addressSlave+1]))/1000000000;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x6009, Modbus.holdingRegisterSize);
				powerCoefApparentC =(float)(uint16ToUint32(Modbus.holdingRegisterValue[addressSlave], Modbus.holdingRegisterValue[addressSlave+1]))/1000000000;

				// handling gain voltage & current  a b c
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2007, Modbus.holdingRegisterSize);
				gainVoltageA =(float)Modbus.holdingRegisterValue[addressSlave]/1000;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2008, Modbus.holdingRegisterSize);
				gainVoltageB =(float)Modbus.holdingRegisterValue[addressSlave]/1000;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2009, Modbus.holdingRegisterSize);
				gainVoltageC =(float)Modbus.holdingRegisterValue[addressSlave]/1000;

				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x200A, Modbus.holdingRegisterSize);
				gainCurrentA =(float)Modbus.holdingRegisterValue[addressSlave]/1000;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x200B, Modbus.holdingRegisterSize);
				gainCurrentB =(float)Modbus.holdingRegisterValue[addressSlave]/1000;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x200C, Modbus.holdingRegisterSize);
				gainCurrentC =(float)Modbus.holdingRegisterValue[addressSlave]/1000;
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
	uint64_t bufferUint64;
	// GET DATA FROM EEPROM EXTERNAL
	ee24_read(0, (uint8_t*)eepromBufferRead, sizeof(eepromBufferRead), 1000);// for(uint8_t indeks=0;indeks<64;indeks++)ee24VirtualRead(&eepromBufferRead[indeks], 0, 1024, indeks);
	// GET DATA FROM FLASH MEMORY
	pmMemoryRead(bufferFlash);
	// CHECKING CRC EEPROM PROCESS
	uint8_t stateMemory = 0;
	uint16_t bufferCrc = modbusCreateCRC(eepromBufferRead, 134);
	// CRC IS NOT AVAILABLE >> FIRST TIME
	if((eepromBufferRead[134]==0xFF) && (eepromBufferRead[135]==0xFF)){stateMemory = 1;}
	// COMPARING BYTE CRC EEPROM WITH GENERATE BYTE CRC >> CRC IS CORRECT
	else if((eepromBufferRead[134]==byteLow(bufferCrc) && (eepromBufferRead[135]==byteHigh(bufferCrc)))){stateMemory = 1;}
	// CRC IS NOT CORRECT
	else{stateMemory = 0;}

	// GET DATA IF EEPROM NOT CORRUPT >> SOURCE DATA EXTERNAL EEPROM
	if(stateMemory){
		// DECODE DATA PROCESS
		for(uint8_t indeks=0;indeks<135;indeks++){
			// DECODE ACTIVE ENERGY PHASE A >> valueuint64 [0];
			if(indeks>=0 && indeks<8)buffer8[indeks] = eepromBufferRead[indeks];
			// DECODE ACTIVE ENERGY PHASE B
			if(indeks>=8 && indeks<16)buffer8[indeksAddress++] = eepromBufferRead[indeks];
			// DECODE ACTIVE ENERGY PHASE C
			if(indeks>=16 && indeks<24)buffer8[indeksAddress++] = eepromBufferRead[indeks];
			// DECODE REACTIVE ENERGY PHASE A
			if(indeks>=24 && indeks<32)buffer8[indeksAddress++] = eepromBufferRead[indeks];
			// DECODE REACRIVE ENERGY PHASE B
			if(indeks>=32 && indeks<40)buffer8[indeksAddress++] = eepromBufferRead[indeks];
			// DECODE REACTIVE ENERGY PHASE C
			if(indeks>=40 && indeks<48)buffer8[indeksAddress++] = eepromBufferRead[indeks];
			// CONVERT 8 BIT TO 64 BIT
			if(indeks == 7 || indeks == 15 || indeks == 23 || indeks == 31 || indeks == 39 || indeks == 47){
				uint8Touint64(&bufferUint64, buffer8);
			}
			if(indeks == 7){
				energyActiveA_uint = bufferUint64;
				if(energyActiveA_uint == 0xFFFFFFFFFFFFFFFF)energyActiveA_uint = ENERGY_ACTIVE_A_DEF;
				indeksAddress = 0;
			}
			if(indeks == 15){
				energyActiveB_uint = bufferUint64;
				if(energyActiveB_uint == 0xFFFFFFFFFFFFFFFF)energyActiveB_uint = ENERGY_ACTIVE_B_DEF;
				indeksAddress = 0;
			}
			if(indeks == 23){
				energyActiveC_uint = bufferUint64;
				if(energyActiveC_uint == 0xFFFFFFFFFFFFFFFF)energyActiveC_uint = ENERGY_ACTIVE_C_DEF;
				indeksAddress = 0;
			}
			if(indeks == 31){
				energyReactiveA_uint = bufferUint64;
				if(energyReactiveA_uint == 0xFFFFFFFFFFFFFFFF)energyReactiveA_uint = ENERGY_REACTIVE_A_DEF;
				indeksAddress = 0;
			}
			if(indeks == 39){
				energyReactiveB_uint = bufferUint64;
				if(energyReactiveB_uint == 0xFFFFFFFFFFFFFFFF)energyReactiveB_uint = ENERGY_REACTIVE_B_DEF;
				indeksAddress = 0;
			}
			if(indeks == 47){
				energyReactiveC_uint = bufferUint64;
				if(energyReactiveC_uint == 0xFFFFFFFFFFFFFFFF)energyReactiveC_uint = ENERGY_REACTIVE_C_DEF;
				indeksAddress = 0;
			}
			// GET DATE FROM EEPROM
			if(	(indeks>=48 && indeks<50) || (indeks>=50 && indeks<52) ||
				(indeks>=52 && indeks<54) || (indeks>=54 && indeks<56) ||
				(indeks>=56 && indeks<58) || (indeks>=58 && indeks<60) ||
				(indeks>=60 && indeks<62) || (indeks>=62 && indeks<64) ||
				(indeks>=64 && indeks<66) || (indeks>=66 && indeks<68) ||
				(indeks>=68 && indeks<70) ){
					buffer8[indeksAddress++] = eepromBufferRead[indeks];
			}
			// DECODE GROOUP CAALIBRATION
			if((indeks==49) || (indeks==51) || (indeks==53) || (indeks==55) || (indeks==57) || (indeks==59) || (indeks==61) || (indeks==63) || (indeks==65) || (indeks==67) || (indeks==69)){
				bufferUint16 = uint8ToUint16(buffer8[0], buffer8[1]);
				indeksAddress = 0;
			}

			// DECODE OFFSET VOLTAGE SUPER USER [HT7036]
			if(indeks==49){
				if(bufferUint16 == 0xFFFF){offsetVolt_ht7036 = OFFSET_VOLT_HT_DEF;}
				else{offsetVolt_ht7036 = bufferUint16;}
			}
			// DECODE OFFSET CURRENT SUPER USER [HT7036]
			if(indeks==51){
				if(bufferUint16 == 0xFFFF){offsetCurr_ht7036 = OFFSET_CURR_HT_DEF;}
				else{offsetCurr_ht7036 = bufferUint16;}
			}
			// DECODE GAIN VOLTAGE SUPER USER [HT7036]
			if(indeks==53){
				if(bufferUint16 == 0xFFFF){gainVolt_ht7036 = GAIN_VOLT_HT_DEF;}
				else{gainVolt_ht7036 = bufferUint16;}
			}
			// DECODE GAIN CURRANT SUPER USER [HT7036]
			if(indeks==55){
				if(bufferUint16 == 0xFFFF){gainCurr_ht7036 = GAIN_CURR_HT_DEF;}
				else{gainCurr_ht7036 = bufferUint16;}
			}
			// DECODE OFFSET VOLTAGE USER [STM32]
			if(indeks==57){
				if(bufferUint16 == 0xFFFF){offsetVolt_stm32 = OFFSET_VOLT_STM_DEF;}
				else{offsetVolt_stm32 = bufferUint16;}
			}
			// DECODE OFFSET CURRENT USER [STM32]
			if(indeks==59){
				if(bufferUint16 == 0xFFFF){offsetCurr_stm32 = OFFSET_CURR_STM_DEF;}
				else{offsetCurr_stm32 = bufferUint16;}
			}
			// DECODE SLAVE ADDRESS MODBUS
			if(indeks==61){
				if(bufferUint16 == 0xFFFF){	Modbus.slaveAddrSlaveSecond = SLAVEID_DEF;}
				else{Modbus.slaveAddrSlaveSecond = (uint8_t)bufferUint16;}
			}
			// DECODE POWER FACTOR CALIBRATION SUPER USER
			if(indeks==63){
				if(bufferUint16 == 0xFFFF){calibPF_ht7036 = (float)PHASE_CORRECTION_ONE;}
				else{calibPF_ht7036 = bufferUint16;}
			}
			// DECODE GAIN POWER FACTOR CALIBRATION >> STM32
			if(indeks==65){
				bufferInt16 = (int16_t)bufferUint16;
				if(bufferUint16 == 0xFFFF){gainPF_stm32 = (float)GAIN_PF_DEF;}
				else{gainPF_stm32 = (float)bufferInt16/10000;}
			}
			// DECODE OFFSET POWER FACTOR CALIBRATION
			if(indeks==67){
				bufferInt16 = (int16_t)bufferUint16;
				if(bufferUint16 == 0xFFFF){offsetPF_stm32 = (float)OFFSET_PF_DEF;}
				else{offsetPF_stm32 = (float)bufferInt16/10000;}
			}
			// DECODE GAIN CURRENT VIA BUTTTON SET
			if(indeks==69){
				if(bufferUint16 == 0xFFFF){gainCurrentButton_stm32 = GAIN_BUTTON_DEF;}
				else{gainCurrentButton_stm32 = bufferUint16;}
			}

			// GET DATA FROM EEPROM
			if(	(indeks>=70 && indeks<74) || (indeks>=74 && indeks<78) ||
				(indeks>=78 && indeks<82) || (indeks>=82 && indeks<86) ||
				(indeks>=86 && indeks<90) || (indeks>=90 && indeks<94) ||
				(indeks>=94 && indeks<98) || (indeks>=98 && indeks<102)||
				(indeks>=102 && indeks<106)){
				buffer8[indeksAddress++] = eepromBufferRead[indeks];
			}
			// DECODE GROUP POWER COEFICIENT
			if((indeks==73) || (indeks==77) || (indeks==81) || (indeks==85) || (indeks==89) || (indeks==93) || (indeks==97) || (indeks==101) || (indeks==105)){
				bufferUint32 = uint16ToUint32(uint8ToUint16(buffer8[0],buffer8[1]),uint8ToUint16(buffer8[2],buffer8[3]));
				indeksAddress = 0;
			}
			// DECODE POWER ACTIVE COEFFICIENT A
			if(indeks==73){
				if(bufferUint32 == 0xFFFFFFFF){powerCoefActiveA = POWER_ACTIVE_A_COEF_DEF;}
				else{powerCoefActiveA = (float)bufferUint32/1000000000;}
			}
			// DECODE POWER ACTIVE COEFFICIENT B
			if(indeks==77){
				if(bufferUint32 == 0xFFFFFFFF){powerCoefActiveB = POWER_ACTIVE_B_COEF_DEF;}
				else{powerCoefActiveB = (float)bufferUint32/1000000000;}
			}
			// DECODE POWER ACTIVE COEFFICIENT C
			if(indeks==81){
				if(bufferUint32 == 0xFFFFFFFF){powerCoefActiveC = POWER_ACTIVE_C_COEF_DEF;}
				else{powerCoefActiveC = (float)bufferUint32/1000000000;}
			}
			// DECODE POWER REACTIVE COEFFICIENT A
			if(indeks==85){
				if(bufferUint32 == 0xFFFFFFFF){powerCoefReactiveA = POWER_REACTIVE_A_COEF_DEF;}
				else{powerCoefReactiveA = (float)bufferUint32/1000000000;}
			}
			// DECODE POWER REACTIVE COEFFICIENT B
			if(indeks==89){
				if(bufferUint32 == 0xFFFFFFFF){powerCoefReactiveB = POWER_REACTIVE_B_COEF_DEF;}
				else{powerCoefReactiveB = (float)bufferUint32/1000000000;}
			}
			// DECODE POWER REACTIVE COEFFICIENT C
			if(indeks==93){
				if(bufferUint32 == 0xFFFFFFFF){powerCoefReactiveC = POWER_REACTIVE_C_COEF_DEF;}
				else{powerCoefReactiveC = (float)bufferUint32/1000000000;}
			}
			// DECODE POWER APPARENT COEFFICIENT A
			if(indeks==97){
				if(bufferUint32 == 0xFFFFFFFF){powerCoefApparentA = POWER_APPARENT_A_COEF_DEF;}
				else{powerCoefApparentA = (float)bufferUint32/1000000000;}
			}
			// DECODE POWER APPARENT COEFFICIENT B
			if(indeks==101){
				if(bufferUint32 == 0xFFFFFFFF){powerCoefApparentB= POWER_APPARENT_B_COEF_DEF;}
				else{powerCoefApparentB = (float)bufferUint32/1000000000;}
			}
			// DECODE POWER APPARENT COEFFICIENT C
			if(indeks==105){
				if(bufferUint32 == 0xFFFFFFFF){powerCoefApparentC = POWER_APPARENT_C_COEF_DEF;}
				else{powerCoefApparentC = (float)bufferUint32/1000000000;}
			}
			// GET DATA FROM EEPROM
			if(		(indeks>=106 && indeks<108) || (indeks>=108 && indeks<110) ||
					(indeks>=110 && indeks<112) || (indeks>=112 && indeks<114) ||
					(indeks>=114 && indeks<116) || (indeks>=116 && indeks<118)){
				buffer8[indeksAddress++] = eepromBufferRead[indeks];
			}
			// DECODE GROUP GAIN RMS VOLTAGE AND CURRENT
			if((indeks==107) || (indeks==109) || (indeks==111) || (indeks==113) || (indeks==113) || (indeks==115) || (indeks==117)){
				bufferUint16 = uint8ToUint16(buffer8[0], buffer8[1]);
				indeksAddress = 0;
			}
			// DECODDE GAIN VOLTAGE A
			if(indeks==107){
				if(bufferUint16 == 0xFFFF){	gainVoltageA = GAIN_VOLT_STM_DEF_A;}
				else{gainVoltageA = (float)bufferUint16/1000;}
			}
			// DECODDE GAIN VOLTAGE B
			if(indeks==109){
				if(bufferUint16 == 0xFFFF){	gainVoltageB = GAIN_VOLT_STM_DEF_B;}
				else{gainVoltageB = (float)bufferUint16/1000;}
			}
			// DECODDE GAIN VOLTAGE C
			if(indeks==111){
				if(bufferUint16 == 0xFFFF){	gainVoltageC = GAIN_VOLT_STM_DEF_C;}
				else{gainVoltageC = (float)bufferUint16/1000;}
			}
			// DECODDE GAIN CURRENT A
			if(indeks==113){
				if(bufferUint16 == 0xFFFF){	gainCurrentA = GAIN_CURR_STM_DEF_A;}
				else{gainCurrentA = (float)bufferUint16/1000;}
			}
			// DECODDE GAIN CURRENT B
			if(indeks==115){
				if(bufferUint16 == 0xFFFF){	gainCurrentB = GAIN_CURR_STM_DEF_B;}
				else{gainCurrentB = (float)bufferUint16/1000;}
			}
			// DECODDE GAIN CURRENT C
			if(indeks==117){
				if(bufferUint16 == 0xFFFF){	gainCurrentC = GAIN_CURR_STM_DEF_C;}
				else{gainCurrentC = (float)bufferUint16/1000;}
			}
			// DECODE GAIN OFFSET ENERGY ACTIVE FOR AUTO RESET
			if(indeks>=118 && indeks<126){buffer8[indeksAddress++] = eepromBufferRead[indeks];
				if(indeks == 125){
					uint8Touint64(&offsetEnergyActive, buffer8);
					if(offsetEnergyActive == 0xFFFFFFFFFFFFFFFF)offsetEnergyActive = 0;
					indeksAddress = 0;
				}
			}
			// DECODE GAIN OFFSET ENERGY REACTIVE FOR AUTO RESET
			if(indeks>=126 && indeks<134){buffer8[indeksAddress++] = eepromBufferRead[indeks];
				if(indeks == 133){
					uint8Touint64(&offsetEnergyReactive, buffer8);
					if(offsetEnergyReactive == 0xFFFFFFFFFFFFFFFF)offsetEnergyReactive = 0;
					indeksAddress = 0;
				}
			}
		}
	// GET DATA IF EEPROM CORRUPT >> SOURCE DATA FROM INTERNAL FLASH MEMORY
	}else{
		// GET OTHER DATA FROM DEFAULT VALUE
		offsetEnergyReactive = 0; offsetEnergyActive = 0;
		gainCurrentA=GAIN_CURR_STM_DEF_A; gainCurrentB=GAIN_CURR_STM_DEF_B; gainCurrentC=GAIN_CURR_STM_DEF_C;
		gainVoltageA=GAIN_VOLT_STM_DEF_A; gainVoltageB=GAIN_VOLT_STM_DEF_B; gainVoltageC=GAIN_VOLT_STM_DEF_C;
		powerCoefActiveA=POWER_ACTIVE_A_COEF_DEF; powerCoefActiveB=POWER_ACTIVE_B_COEF_DEF; powerCoefActiveC=POWER_ACTIVE_C_COEF_DEF;
		powerCoefReactiveA=POWER_REACTIVE_A_COEF_DEF; powerCoefReactiveB=POWER_REACTIVE_B_COEF_DEF; powerCoefReactiveC=POWER_REACTIVE_C_COEF_DEF;
		powerCoefApparentA=POWER_APPARENT_A_COEF_DEF; powerCoefApparentB=POWER_APPARENT_B_COEF_DEF; powerCoefApparentC=POWER_APPARENT_C_COEF_DEF;
		offsetPF_stm32=(float)OFFSET_PF_DEF; gainPF_stm32=(float)GAIN_PF_DEF; calibPF_ht7036=(float)PHASE_CORRECTION_ONE;
		offsetCurr_stm32=OFFSET_CURR_STM_DEF; offsetVolt_stm32=OFFSET_VOLT_STM_DEF;
		gainCurr_ht7036=GAIN_CURR_HT_DEF; gainVolt_ht7036=GAIN_VOLT_HT_DEF;
		offsetCurr_ht7036=OFFSET_CURR_HT_DEF; offsetVolt_ht7036=OFFSET_VOLT_HT_DEF;
		// DECODE DATA FROM FLASH MEMORY & GET DATA MEMORY
		pmDecode(bufferFlash);
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
	// SYNCRON FROM DATA EEPROM TO MODBUS REGISTER
	uint16_t addressSlave[3];
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x3001, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[addressSlave[0]] = offsetVolt_ht7036;												// offset Voltage HT7036 >> 0x3001
	Modbus.holdingRegisterValue[addressSlave[0]+1] = offsetCurr_ht7036; 											// offset Current HT7036 >> 0x3002
	Modbus.holdingRegisterValue[addressSlave[0]+2] = gainVolt_ht7036;												// gain Voltage HT7036 >> 0x3003
	Modbus.holdingRegisterValue[addressSlave[0]+2] = gainCurr_ht7036;												// gain Current HT7036 >> 0x3004
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2001, Modbus.holdingRegisterSize);			// offset Voltage STM32 >> 0x2001, 0x2002, 0x2003
	Modbus.holdingRegisterValue[addressSlave[0]] = Modbus.holdingRegisterValue[addressSlave[0]+1] = Modbus.holdingRegisterValue[addressSlave[0]+2] = offsetVolt_stm32;
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2004, Modbus.holdingRegisterSize);			// offset Current STM32 >> 0x2004, 0x2005, 0x2006
	Modbus.holdingRegisterValue[addressSlave[0]] = Modbus.holdingRegisterValue[addressSlave[0]+1] = Modbus.holdingRegisterValue[addressSlave[0]+2] = offsetCurr_stm32;
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x2007, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[addressSlave[0]] = (uint16_t)(gainVoltageA*1000);									// gain Voltage STM32 >> 0x2007
	Modbus.holdingRegisterValue[addressSlave[0]+1] = (uint16_t)(gainVoltageB*1000);									// gain Voltage STM32 >> 0x2008
	Modbus.holdingRegisterValue[addressSlave[0]+2] = (uint16_t)(gainVoltageC*1000);									// gain Voltage STM32 >> 0x2009
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x200A, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[addressSlave[0]] = (uint16_t)(gainCurrentA*1000);									// gain Current STM32 >> 0x200A
	Modbus.holdingRegisterValue[addressSlave[0]+1] = (uint16_t)(gainCurrentB*1000);									// gain Current STM32 >> 0x200B
	Modbus.holdingRegisterValue[addressSlave[0]+2] = (uint16_t)(gainCurrentC*1000);									// gain Current STM32 >> 0x200C
	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x1000, Modbus.holdingRegisterSize);			// modbus slave id
	Modbus.holdingRegisterValue[addressSlave[0]] = Modbus.slaveAddrSlaveSecond;

	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x4001, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[addressSlave[0]] = calibPF_ht7036;													// power factor calibration >> 0x4001
	Modbus.holdingRegisterValue[addressSlave[0]+1] = (uint16_t)(gainPF_stm32*10000);								// gain power factor stm32 >> 0x4002
	Modbus.holdingRegisterValue[addressSlave[0]+2] = (uint16_t)(offsetPF_stm32*10000);								// offset power factor stm32 >> 0x4003
	Modbus.holdingRegisterValue[addressSlave[0]+3] = gainCurrentButton_stm32;										// gain current user via button >> 0x4004
	Modbus.holdingRegisterValue[addressSlave[0]+4] = byte32High((uint32_t)(powerCoefActiveA*1000000000));			// decode param power active coef A >> 0x4005, 0x4006
	Modbus.holdingRegisterValue[addressSlave[0]+5] = byte32Low((uint32_t)(powerCoefActiveA*1000000000));
	Modbus.holdingRegisterValue[addressSlave[0]+6] = byte32High((uint32_t)(powerCoefActiveB*1000000000));			// decode param power active coef B >> 0x4007, 0x4008
	Modbus.holdingRegisterValue[addressSlave[0]+7] = byte32Low((uint32_t)(powerCoefActiveB*1000000000));
	Modbus.holdingRegisterValue[addressSlave[0]+8] = byte32High((uint32_t)(powerCoefActiveC*1000000000));			// decode param power active coef C >> 0x4009, 0x400A
	Modbus.holdingRegisterValue[addressSlave[0]+9] = byte32Low((uint32_t)(powerCoefActiveC*1000000000));

	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x5005, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[addressSlave[0]] = byte32High((uint32_t)(powerCoefReactiveA*1000000000));			// decode param power reactive coef A >> 0x5005, 0x5006
	Modbus.holdingRegisterValue[addressSlave[0]+1] = byte32Low((uint32_t)(powerCoefReactiveA*1000000000));
	Modbus.holdingRegisterValue[addressSlave[0]+2] = byte32High((uint32_t)(powerCoefReactiveB*1000000000));			// decode param power reactive coef B >> 0x5007, 0x5008
	Modbus.holdingRegisterValue[addressSlave[0]+3] = byte32Low((uint32_t)(powerCoefReactiveB*1000000000));
	Modbus.holdingRegisterValue[addressSlave[0]+4] = byte32High((uint32_t)(powerCoefReactiveC*1000000000));			// decode param power reactive coef C >> 0x5009, 0x500A
	Modbus.holdingRegisterValue[addressSlave[0]+5] = byte32Low((uint32_t)(powerCoefReactiveC*1000000000));

	addressSlave[0] = modbusGetIndeks(Modbus.holdingRegisterAddress, 0x6005, Modbus.holdingRegisterSize);
	Modbus.holdingRegisterValue[addressSlave[0]] = byte32High((uint32_t)(powerCoefApparentA*1000000000));			// decode param power apparent coef A >> 0x6005, 0x6006
	Modbus.holdingRegisterValue[addressSlave[0]+1] = byte32Low((uint32_t)(powerCoefApparentA*1000000000));
	Modbus.holdingRegisterValue[addressSlave[0]+2] = byte32High((uint32_t)(powerCoefApparentB*1000000000));			// decode param power apparent coef B >> 0x6007, 0x6008
	Modbus.holdingRegisterValue[addressSlave[0]+3] = byte32Low((uint32_t)(powerCoefApparentB*1000000000));
	Modbus.holdingRegisterValue[addressSlave[0]+4] = byte32High((uint32_t)(powerCoefApparentC*1000000000));			// decode param power apparent coef C >> 0x6009, 0x600A
	Modbus.holdingRegisterValue[addressSlave[0]+5] = byte32Low((uint32_t)(powerCoefApparentC*1000000000));
}

void eepromLoop(){
	eepromTimerDelta = HAL_GetTick() - eepromTimer;
	// ROUTINE WRITE EEPROM
	if(stateConfig || (eepromTimerDelta > INTERVAL_EEPROM)){
		eepromTimer = HAL_GetTick();
		timerFlash += 1;
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
		// ENCODE DATA & WRITE EEPROM
		eepromEncode(
				energyActiveA_uint,							energyActiveB_uint,							energyActiveC_uint,
				energyReactiveA_uint,						energyReactiveB_uint,						energyReactiveC_uint,
				offsetVolt_ht7036,							offsetCurr_ht7036,							gainVolt_ht7036,
				gainCurr_ht7036,							offsetVolt_stm32,							offsetCurr_stm32,
				Modbus.slaveAddrSlaveSecond, 				calibPF_ht7036,								(uint16_t)(gainPF_stm32*10000), 			(uint16_t)(offsetPF_stm32*10000),
				gainCurrentButton_stm32,
				(uint32_t)(powerCoefActiveA*1000000000),	(uint32_t)(powerCoefActiveB*1000000000),	(uint32_t)(powerCoefActiveC*1000000000),
				(uint32_t)(powerCoefReactiveA*1000000000),	(uint32_t)(powerCoefReactiveB*1000000000),	(uint32_t)(powerCoefReactiveC*1000000000),
				(uint32_t)(powerCoefApparentA*1000000000),	(uint32_t)(powerCoefApparentB*1000000000),	(uint32_t)(powerCoefApparentC*1000000000),
				(uint16_t)(gainVoltageA*1000),				(uint16_t)(gainVoltageB*1000),				(uint16_t)(gainVoltageC*1000),
				(uint16_t)(gainCurrentA*1000),				(uint16_t)(gainCurrentB*1000),				(uint16_t)(gainCurrentC*1000),
				offsetEnergyActive,							offsetEnergyReactive
		);
		// GENERATE CRC FOR EEPROM >> MODIFY BEGIN
		uint16_t bufferCrc = modbusCreateCRC(eepromBufferWrite, 134); // mean value 134 >> total data of EEPROM
		// CONVERT 32 TO 8 BIT ARRAY >> MODIFY END
		eepromBufferWrite[134] = byteLow(bufferCrc);
		eepromBufferWrite[135] = byteHigh(bufferCrc);

		ee24_write(0, (uint8_t*)eepromBufferWrite, sizeof(eepromBufferWrite), 1000);
		// CONFIRM READING DATA
		uint8_t stateEeprom = 1;
		uint8_t checkStateEeprom = 1;
		for(uint8_t indeksCheck=0; indeksCheck<5; indeksCheck++){ // CHECKING UNTIL 5 TRY
			ee24_read(0, (uint8_t*)eepromBufferRead, sizeof(eepromBufferRead), 1000);
			for(uint8_t indeks=0;indeks<eepromSize;indeks++){
				checkStateEeprom = eepromBufferRead[indeks] == eepromBufferWrite[indeks];
				stateEeprom = stateEeprom && checkStateEeprom;
			}
			if(stateEeprom == 0){
				ee24_write(0, (uint8_t*)eepromBufferWrite, sizeof(eepromBufferWrite), 1000);
				HAL_Delay(10);
			}else{
				break;
			}
		}
	}
	// ROUTINE WRITE FLASH
	if((stateConfig) || (timerFlash >= 60)){
		timerFlash = 0;
		pmEncode(bufferFlash);
		pmMemoryWrite(bufferFlash);
	}
	// RESET TRIGGER STATE CONFIG VIA BUTTON
	stateConfig = 0;
}

void encodeUint64ToEeprom(uint8_t startIndex, uint64_t value) {
    uint8_t buffer8[8];
    uint64ToUint8(buffer8, value);
    for (uint8_t i = 0; i < 8; i++) {
        eepromBufferWrite[startIndex + i] = buffer8[i];
    }
}

void encodeUint32ToEeprom(uint8_t startIndex, uint32_t value) {
    eepromBufferWrite[startIndex] = byte16High(byte32High(value));
    eepromBufferWrite[startIndex + 1] = byte16Low(byte32High(value));
    eepromBufferWrite[startIndex + 2] = byte16High(byte32Low(value));
    eepromBufferWrite[startIndex + 3] = byte16Low(byte32Low(value));
}

void encodeUint16ToEeprom(uint8_t startIndex, uint16_t value) {
    eepromBufferWrite[startIndex] = byte16High(value);
    eepromBufferWrite[startIndex + 1] = byte16Low(value);
}

void eepromEncode(
        uint64_t energyActiveA, uint64_t energyActiveB, uint64_t energyActiveC,
        uint64_t energyReactiveA, uint64_t energyReactiveB, uint64_t energyReactiveC,
        uint16_t offsetVolt_ht7036, uint16_t offsetCurr_ht7036,
        uint16_t gainVolt_ht7036, uint16_t gainCurr_ht7036,
        int16_t offsetVolt_stm32, int16_t offsetCurr_stm32,
        uint16_t slaveAddress, uint16_t calibPF_ht7036,
        uint16_t gainPF_stm32, uint16_t offsetPF_stm32,
        uint16_t gainCurrentButton_stm32,
        uint32_t powerCoefActiveA, uint32_t powerCoefActiveB, uint32_t powerCoefActiveC,
        uint32_t powerCoefReactiveA, uint32_t powerCoefReactiveB, uint32_t powerCoefReactiveC,
        uint32_t powerCoefApparentA, uint32_t powerCoefApparentB, uint32_t powerCoefApparentC,
        uint16_t gainVoltageA, uint16_t gainVoltageB, uint16_t gainVoltageC,
        uint16_t gainCurrentA, uint16_t gainCurrentB, uint16_t gainCurrentC,
        uint64_t offsetEnergyActive, uint64_t offsetEnergyReactive) {

    encodeUint64ToEeprom(0, energyActiveA);
    encodeUint64ToEeprom(8, energyActiveB);
    encodeUint64ToEeprom(16, energyActiveC);
    encodeUint64ToEeprom(24, energyReactiveA);
    encodeUint64ToEeprom(32, energyReactiveB);
    encodeUint64ToEeprom(40, energyReactiveC);

    encodeUint16ToEeprom(48, offsetVolt_ht7036);
    encodeUint16ToEeprom(50, offsetCurr_ht7036);
    encodeUint16ToEeprom(52, gainVolt_ht7036);
    encodeUint16ToEeprom(54, gainCurr_ht7036);

    encodeUint16ToEeprom(56, (uint16_t)offsetVolt_stm32);
    encodeUint16ToEeprom(58, (uint16_t)offsetCurr_stm32);
    encodeUint16ToEeprom(60, slaveAddress);
    encodeUint16ToEeprom(62, calibPF_ht7036);
    encodeUint16ToEeprom(64, gainPF_stm32);
    encodeUint16ToEeprom(66, offsetPF_stm32);
    encodeUint16ToEeprom(68, gainCurrentButton_stm32);

    encodeUint32ToEeprom(70, powerCoefActiveA);
    encodeUint32ToEeprom(74, powerCoefActiveB);
    encodeUint32ToEeprom(78, powerCoefActiveC);
    encodeUint32ToEeprom(82, powerCoefReactiveA);
    encodeUint32ToEeprom(86, powerCoefReactiveB);
    encodeUint32ToEeprom(90, powerCoefReactiveC);
    encodeUint32ToEeprom(94, powerCoefApparentA);
    encodeUint32ToEeprom(98, powerCoefApparentB);
    encodeUint32ToEeprom(102, powerCoefApparentC);

    encodeUint16ToEeprom(106, gainVoltageA);
    encodeUint16ToEeprom(108, gainVoltageB);
    encodeUint16ToEeprom(110, gainVoltageC);

    encodeUint16ToEeprom(112, gainCurrentA);
    encodeUint16ToEeprom(114, gainCurrentB);
    encodeUint16ToEeprom(116, gainCurrentC);

    encodeUint64ToEeprom(118, offsetEnergyActive);
    encodeUint64ToEeprom(126, offsetEnergyReactive);
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
	powerHandleTreshold(&rmsVoltageA,0.8,-0.8);
	powerHandleTreshold(&rmsVoltageB,0.8,-0.8);
	powerHandleTreshold(&rmsVoltageC,0.8,-0.8);
	powerHandleTreshold(&rmsVoltageVector,0.8,-0.8);
	// RMS VOLTAGE DIV
	powerHandleTreshold(&rmsVoltageAB,0.5,-0.5);
	powerHandleTreshold(&rmsVoltageBC,0.5,-0.5);
	powerHandleTreshold(&rmsVoltageCA,0.5,-0.5);
	// RMS CURRENT
	// powerHandleTreshold(&rmsCurrentA,0.009,-0.009);
	// powerHandleTreshold(&rmsCurrentB,0.009,-0.009);
	// powerHandleTreshold(&rmsCurrentC,0.009,-0.009);
	// powerHandleTreshold(&rmsCurrentVector,0.009,-0.009);
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
	rmsVoltageA = rmsVoltageA*gainVoltageA + offsetVoltage;
	rmsVoltageB = rmsVoltageB*gainVoltageB + offsetVoltage;
	rmsVoltageC = rmsVoltageC*gainVoltageC + offsetVoltage;

	// HANDLING TRESHOLD CURRENT 5 mA >> 0.005 A
	powerHandleTreshold(&rmsCurrentA,0.005,-0.005);
	powerHandleTreshold(&rmsCurrentB,0.005,-0.005);
	powerHandleTreshold(&rmsCurrentC,0.005,-0.005);
	powerHandleTreshold(&rmsCurrentVector,0.005,-0.005);

	// CALIBRATION CURRENT
	rmsCurrentA = rmsCurrentA*gainCurrentA*gainCurrentButton_stm32 + offsetCurrent;
	rmsCurrentB = rmsCurrentB*gainCurrentB*gainCurrentButton_stm32 + offsetCurrent;
	rmsCurrentC = rmsCurrentC*gainCurrentC*gainCurrentButton_stm32 + offsetCurrent;

	// HANDLING POWER APPARENT
	if(powerCoefApparentA == 0)powerApparentA = rmsVoltageA * rmsCurrentA;
	if(powerCoefApparentB == 0)powerApparentB = rmsVoltageB * rmsCurrentB;
	if(powerCoefApparentC == 0)powerApparentC = rmsVoltageC * rmsCurrentC;

	// HANDLING POWER FACTOR
	if(gainPF_stm32 == 0){
		float bufferActive, bufferApparent;
		bufferActive = powerActiveA; bufferApparent = powerApparentA;
		handleAbsolute(&bufferActive); handleAbsolute(&bufferApparent);
		if(bufferApparent == 0){
			powerFactorA = 1;
		}else{
			powerFactorA = bufferActive / bufferApparent;
			if(powerFactorA > 1) powerFactorA = 1;
			if(powerFactorA < 0) powerFactorA = 0;

		}
		if((bufferActive < 0.001) && (bufferApparent < 0.001))powerFactorA = 1;

		bufferActive = powerActiveB; bufferApparent = powerApparentB;
		handleAbsolute(&bufferActive); handleAbsolute(&bufferApparent);
		if(bufferApparent == 0){
			powerFactorB = 1;
		}else{
			powerFactorB = bufferActive / bufferApparent;
			if(powerFactorB > 1) powerFactorB = 1;
			if(powerFactorB < 0) powerFactorB = 0;
		}
		if((bufferActive < 0.001) && (bufferApparent < 0.001))powerFactorB = 1;

		bufferActive = powerActiveC; bufferApparent = powerApparentC;
		handleAbsolute(&bufferActive); handleAbsolute(&bufferApparent);
		if(bufferApparent == 0){
			powerFactorC = 1;
		}else{
			powerFactorC = bufferActive / bufferApparent;
			if(powerFactorC > 1) powerFactorC = 1;
			if(powerFactorC < 0) powerFactorC = 0;
		}
		if((bufferActive < 0.001) && (bufferApparent < 0.001))powerFactorC = 1;

	}
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

void decodeGain(uint8_t eepromBufferRead[], float* gainData, int index, float defaultValue){
    static int bufferIndex = 0;
    // macro encodefing sie >> 2
    if (index % 2 == 0) {
        uint16_t bufferUint16 = uint8ToUint16(eepromBufferRead[index], eepromBufferRead[index + 1]);
        if (bufferUint16 == 0xFFFF) {
            gainData[bufferIndex] = defaultValue;
        } else {
            gainData[bufferIndex] = (float)bufferUint16 / 1000;
        }
        bufferIndex++;
    }
}
void handlingPowerInit(){
	uint8_t statusEvent = 0;
	uint8_t statusRoutine = 0;
	uint8_t statusTotal = 0;

	// CHECKING STATUS EVENT
	if(((rmsVoltageA < 0.1) && (rmsVoltageA > -0.1)) && (rmsCurrentA != 0)){
		statusEvent = 1;
	}
	else if(((rmsVoltageB < 0.1) && (rmsVoltageB > -0.1)) && (rmsCurrentB != 0)){
		statusEvent = 1;
	}
	else if(((rmsVoltageC < 0.1) && (rmsVoltageC > -0.1)) && (rmsCurrentC != 0)){
		statusEvent = 1;
	}

	// CHECKING STATUS ROUTINE
	rstPowerTimerDelta = HAL_GetTick() - rstPowerTimer;
	if(rstPowerTimerDelta >= 1800000){
		rstPowerTimer = HAL_GetTick();
		statusRoutine = 1;
	}

	// CALCULATE STATUS
	statusTotal = statusEvent || statusRoutine;

	// TRIGGERR DATA
	if(statusTotal == 1){
		// RE-SETUP POWER METER
		powerMeterSetup();
		HAL_Delay(1000);

		// RESET STATUS EVENT & ROUTINE
		statusEvent = 0;
		statusRoutine = 0;
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
