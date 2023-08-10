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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HT7036.h"
#include "modbusSlave.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//------------------------- GROUP VARIABLE POWER METER ---------------------------------
uint8_t addrSensor[] = {
		// POWER REGISTER >> 12 addrs
		r_Pa,				r_Pb,				r_Pc,				r_Pt,
		r_Qa,				r_Qb,				r_Qc,				r_Qt,
		r_Sa,				r_Sb,				r_Sc,				r_St,
		// RMS REGISTER >> 8 addrs
		r_UaRms,			r_UbRms,			r_UcRms,			r_UtRms,
		r_IaRms,			r_IbRms,			r_IcRms,			r_ItRms,
		// POWER FACTOR REGISTER >> 4 addrs
		r_Pfa,				r_Pfb,				r_Pfc,				r_Pft,
		// ENERGY REGISTER >> 4 addrs
		r_Epa,				r_Epb, 				r_Epc,				r_Ept
		// TOTAL REGISTER >> 28
};

uint8_t sizeSensor = sizeof(addrSensor)/sizeof(addrSensor[0]);
uint64_t powerTimer = 0;
uint64_t powerTimerDelta = 0;
uint32_t valueSensor[28];
HAL_StatusTypeDef spiStatus[256];
extern float HFconstVal;
extern float ECVal;
uint8_t stateConfig = 0;
uint8_t phase = PHASE_RST;
float valueFloat[28];

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
float 	energyActiveA,		energyActiveB, 		energyActiveC,		energyActiveCombine;

//------------------------- GROUP VARIABLE MODBUS ---------------------------------
/* A. NOTE
 * 	Group Power 			>> 4 byte Sign 		>> division 100
 * 	Group RMS				>> 2 byte unsgin	>> division 100
 * 	Gruop power factor 		>> 2 byte unsign	>> division 100
 * 	Group energy 			>> 4 byte unsign 	>> division 100
 *
 * B. MAPPING REGISTER EXISTING
 * 	V A-N(VAB)				>> 2027
 * 	V B-N(VBC)				>> 3029
 * 	V C-N(VCA)				>> 3031
 * 	I A						>> 2999
 * 	I B						>> 3001
 * 	I C						>> 3003
 * 	Pow A					>> 3053
 * 	Pow B 					>> 3055
 * 	Pow C 					>> 3057
 * 	Pow Tot					>> 3059
 * 	THD V-A					>> 21329
 * 	THD V-B					>> 21331
 * 	THD V-C					>> 21333
 * 	PF A					>> 3077
 * 	PF B					>> 3079
 * 	PF C 					>> 3081
 * 	Reactive Energy	(VARH) 	>> 3219
 * 	Energy (WH)				>> 3203
 * 	V A-B 					>> 3019
 * 	V B-C 					>> 3021
 * 	V C-A 					>> 3023
 */
extern MODBUS Modbus;						//  ADDRESS REGISTER VALUE POWER SENSOR
uint16_t holdingRegisterAddress[] 	= 	{	// power register 4 byte
											0x0000,	0x0001,	0x0002, 0x0003,
											0x0004, 0x0005, 0x0006, 0x0007,
											0x0008, 0x0009, 0x000A, 0x000B,
											0x000C, 0x000D, 0x000E, 0x000F,
											0x0010, 0x0011, 0x0012, 0x0013,
											0x0014, 0x0015, 0x0016, 0x0017,
											// rms register 2 byte
											0x0018, 0x0019, 0x001A, 0x001B,
											0x001C, 0x001D, 0x001E, 0x001F,
											// power factor register 2 byte
											0x0020, 0x0021, 0x0022, 0x0023,
											// energy register 4 byte
											0x0024, 0x0025, 0x0026, 0x0027,
											0x0028, 0x0029, 0x002A, 0x002B,
											// parameter calibraiton
											0x002C, 0x002D, 0x002E, 		// offset voltage ABC Phase
											0x002F, 0x0030, 0x0031, 		// offset current ABC Phase
											0x0032, 0x0033, 0x0034, 		// gain voltage ABC Phase
											0x0035, 0x0036, 0x0037,			// gain current ABC Phase
											// ADDRESS REGISTER SLAVE ID POWER SENSOR
											0X1000,
											// ADDRESS REGISTER PARAMETER CALIBRATION POWER SENSOR
											0x1001, 0x1002, 0x1003, 		// offset voltage ABC Phase
											0x1004, 0x1005, 0x1006, 		// offset current ABC Phase
											0x1007, 0x1008, 0x1009, 		// gain voltage ABC Phase
											0x100A, 0x100B, 0x100C			// gain current ABC Phase
										};
											// VALUE REGISTER POWER SENSOR
uint16_t holdingRegisterValue[]		= 	{	0x0000,	0x0000,	0x0000, 0x0000,
											0x0000, 0x0000, 0x0000, 0x0000,
											0x0000, 0x0000, 0x0000, 0x0000,
											0x0000, 0x0000, 0x0000, 0x0000,
											0x0000, 0x0000, 0x0000, 0x0000,
											0x0000, 0x0000, 0x0000, 0x0000,
											0x0000, 0x0000, 0x0000, 0x0000,
											0x0000, 0x0000, 0x0000, 0x0000,
											0x0000, 0x0000, 0x0000, 0x0000,
											0x0000, 0x0000, 0x0000, 0x0000,
											0x0000, 0x0000, 0x0000, 0x0000,
											0x0000, 0x0000, 0x0000,
											0x0000, 0x0000, 0x0000,
											0x0000, 0x0000, 0x0000,
											0x0000, 0x0000, 0x0000,
											// VALUE REGISTER SLAVE ID POWER SENSOR
											0x0000,
											// VALUE  REGISTER PARAMETER CALIBRATION POWER SENSOR
											0x0000, 0x0000, 0x0000, 		// offset voltage
											0x0000, 0x0000, 0x0000, 		// offset current
											0x0000, 0x0000, 0x0000, 		// gain voltage
											0x0000, 0x0000, 0x0000			// gain current
										};
uint16_t holdingRegisterSize		= (uint16_t)sizeof(holdingRegisterAddress)/sizeof(uint16_t);
extern uint16_t addressModbus;
//------------------------- GROUP VARIABLE TESTING ---------------------------------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void serialPrint(char* text, uint8_t size){HAL_UART_Transmit(&huart2, (uint8_t*)text, size, 100);}
void powerMeterSetup();
uint16_t byteLow32(uint32_t buf){return (uint16_t)((buf & 0x0000FFFF));}
uint16_t byteHigh32(uint32_t buf){return (uint16_t)((buf & 0xFFFF0000) >> 16);}
void modbusValueUpdate();
void powerCalibLoop();
void splitValueSensor();
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // SETUP POWER METER
  powerMeterSetup();

  // INITIAL TIMER SAMPLING POWER
  powerTimer = HAL_GetTick();

  // MODBUS SETUP
  ModbusBegin(
		  &Modbus,
		  &huart2,
		  0,
		  0x00,
		  0x01,
		  holdingRegisterAddress,
		  holdingRegisterValue,
		  &holdingRegisterSize,
		  MODBUS_En_GPIO_Port,
		  MODBUS_En_Pin
  );
  // START MODBUS HANDLE
  modbusReceive(&Modbus);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  powerTimerDelta = HAL_GetTick() - powerTimer;
	  powerTimer = HAL_GetTick();
	  powerMultiReadSensor(addrSensor, valueSensor, valueFloat, 28);
	  splitValueSensor();
	  if(stateConfig){
		  if((phase==PHASE_A) && (valueSensor[8]>0))ECVal = calcMeterConstant(valueSensor[8], HFconstVal, rmsVoltageA*rmsCurrentA);
		  if((phase==PHASE_B) && (valueSensor[9]>0))ECVal = calcMeterConstant(valueSensor[9], HFconstVal, rmsVoltageB*rmsCurrentB);
		  if((phase==PHASE_C) && (valueSensor[10]>0))ECVal = calcMeterConstant(valueSensor[10], HFconstVal, rmsVoltageC*rmsCurrentC);
		  powerMultiReadSensor(addrSensor, valueSensor, valueFloat, 28);
		  phase=PHASE_RST;
		  stateConfig = 0;
	  }
	  modbusValueUpdate();
	  powerCalibLoop();
	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void){
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
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
void powerMeterSetup(){
	uint8_t address[] = {
//			  w_IaRmsoffse,
//			  w_IbRmsoffse,
//			  w_IcRmsoffse,
//			  w_UaRmsoffse,
//			  w_UbRmsoffse,
//			  w_UcRmsoffse,
//	  		  w_UgainA,
//			  w_UgainB,
//			  w_UgainC,
			  w_IgainA,
			  w_IgainB,
			  w_IgainC
	};
	uint32_t addressData[] = {
//			  7,
//			  7,
//			  7,
//			  8,
//			  8,
//			  8,
//			  62011,
//			  62011,
//			  62011,
			  49853,
			  49853,
			  49853
	};
	powerRestoreCalib();
	powerSetup(address,addressData,spiStatus,3);
	powerMultiReadSensor(addrSensor, valueSensor, valueFloat, 69);
//	splitValueSensor();
}

void modbusValueUpdate(){
	uint32_t bufferUnsign32;
	int32_t bufferSign32;
	uint16_t bufferUnsign16;
	uint8_t address = 0;
	for(int indeks=0;indeks<28;indeks++){
		// CONFERT FLOAT DATA TO INTEGER SIGN/UNSGIN & 2BYTE/4BYTE
		// GROUP POWER
		if(indeks >= 0 && indeks <12){
			bufferSign32  = (int32_t)(valueFloat[indeks] * 100);
			bufferUnsign32 = (uint32_t)bufferSign32;
			Modbus.holdingRegisterValue[address++] = byteHigh32(bufferUnsign32);
			Modbus.holdingRegisterValue[address++] = byteLow32(bufferUnsign32);
		}
		// GROUP RMS
		if(indeks>=12 && indeks<20){
			bufferUnsign16 = (uint16_t)(valueFloat[indeks] * 100);
			Modbus.holdingRegisterValue[address++] = bufferUnsign16;
		}
		// GROUP POWER FACTOR
		if(indeks>=20 && indeks<24){
			bufferUnsign16 = (uint16_t)(valueFloat[indeks]*100);
			Modbus.holdingRegisterValue[address++] = bufferUnsign16;
		}
		// GROUP ENERGY
		if(indeks>=24 && indeks<28){
			bufferSign32 = (int32_t)(valueFloat[indeks]*100);
			bufferUnsign32 = (uint32_t)bufferSign32;
			Modbus.holdingRegisterValue[address++] = byteHigh32(bufferUnsign32);
			Modbus.holdingRegisterValue[address++] = byteHigh32(bufferUnsign32);
		}
	}
}

void powerCalibLoop(){
	if(Modbus.trigState){
		uint8_t addressSlave;
		uint8_t addressIndeks;
		uint16_t dataCalib16;
		uint32_t dataCalib32;
		// FUCTION CODE WRITE REGISTER
		if(Modbus.functionCode == 0x06){
			uint8_t splitSensorIndeks[] = {
					12, 13, 14,	// Address Write Voltage Offset	<< 0x1001(45), 0x1002(46), 0x1003(47)
					16, 17, 18,	// Address Write Current Offset	<< 0x1004(48), 0x1005(49), 0x1006(50)
					12, 13, 14,	// Address Write Voltage Gain	<< 0x1007(51), 0x1008(52), 0x1009(53)
					16, 17, 18	// Address Write CUrrent Gain	<< 0x100A(54), 0x100B(55), 0x100C(56)
			};
			for(uint8_t indeks=0;indeks<12;indeks++){
				if(addressModbus == Modbus.holdingRegisterAddress[57+indeks])addressIndeks = splitSensorIndeks[indeks];
				else __NOP();
			}
			// FILTER REGISTER OFFSET VOLTAGE RMS
			if((addressModbus == 0x1001) || (addressModbus == 0x1002) || (addressModbus == 0x1003)){
				// GET DATA MODBUS FOR ZEROIING VALUE
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				dataCalib16 = Modbus.holdingRegisterValue[addressSlave];
				dataCalib32 = (uint32_t)dataCalib16;
				if(addressModbus == 0x1001)Modbus.holdingRegisterValue[addressSlave-13] = powerSingleRecalib(VRMS_OFFSET, w_UaRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0]);
				else if(addressModbus == 0x1002)Modbus.holdingRegisterValue[addressSlave-13] = powerSingleRecalib(VRMS_OFFSET, w_UbRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0]);
				else if(addressModbus == 0x1003)Modbus.holdingRegisterValue[addressSlave-13] = powerSingleRecalib(VRMS_OFFSET, w_UcRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0]);
			}
			// FILTER REGISTER OFFSET CURRENT RMS
			else if((addressModbus == 0x1004) || (addressModbus == 0x1005) || (addressModbus == 0x1006)){
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				dataCalib16 = Modbus.holdingRegisterValue[addressSlave];
				dataCalib32 = (uint32_t)dataCalib16;
				if(addressModbus == 0x1004)Modbus.holdingRegisterValue[addressSlave-13] = powerSingleRecalib(IRMS_OFFSET, w_IaRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0]);
				else if(addressModbus == 0x1005)Modbus.holdingRegisterValue[addressSlave-13] = powerSingleRecalib(IRMS_OFFSET, w_IbRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0]);
				else if(addressModbus == 0x1006)Modbus.holdingRegisterValue[addressSlave-13] = powerSingleRecalib(IRMS_OFFSET, w_IcRmsoffse, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0]);
			}
			// FILTER REGISTER GAIN VOLTAGE RMS
			else if((addressModbus == 0x1007) || (addressModbus == 0x1008) || (addressModbus == 0x1009)){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				dataCalib16 = Modbus.holdingRegisterValue[addressSlave];
				dataCalib32 = (uint32_t)dataCalib16;
				if(dataCalib16 != 0){
					if(addressModbus == 0x1007){Modbus.holdingRegisterValue[addressSlave-13] = powerSingleRecalib(VRMS_GAIN, w_UgainA, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0]);phase=PHASE_A;}
					else if(addressModbus == 0x1008){Modbus.holdingRegisterValue[addressSlave-13] = powerSingleRecalib(VRMS_GAIN, w_UgainB, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0]);phase=PHASE_B;}
					else if(addressModbus == 0x1009){Modbus.holdingRegisterValue[addressSlave-13] = powerSingleRecalib(VRMS_GAIN, w_UgainC, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0]);phase=PHASE_C;}
				}else __NOP();
			}
			// FILTER REGISTER GAIN CURRENT RMS
			else if((addressModbus == 0x100A) || (addressModbus == 0x100B) || (addressModbus == 0x100C)){
				stateConfig = Modbus.trigState;
				addressSlave = modbusGetIndeks(Modbus.holdingRegisterAddress, addressModbus, Modbus.holdingRegisterSize);
				dataCalib16 = Modbus.holdingRegisterValue[addressSlave];
				dataCalib32 = (uint32_t)dataCalib16;
				if(addressSlave !=0){
					if(addressModbus == 0x100A){Modbus.holdingRegisterValue[addressSlave-13] = powerSingleRecalib(IRMS_GAIN, w_IgainA, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0]);phase=PHASE_A;}
					else if(addressModbus == 0x100B){Modbus.holdingRegisterValue[addressSlave-13] = powerSingleRecalib(IRMS_GAIN, w_IgainB, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0]);phase=PHASE_B;}
					else if(addressModbus == 0x100C){Modbus.holdingRegisterValue[addressSlave-13] = powerSingleRecalib(IRMS_GAIN, w_IgainC, &dataCalib32, addrSensor[addressIndeks], &spiStatus[0]);phase=PHASE_C;}
				}else __NOP();
			}
		}
		if(Modbus.functionCode == 0x10){
			__NOP();
		}else __NOP();
		Modbus.trigState = 0;
	}else __NOP();
}

void splitValueSensor(){
	powerActiveA = valueFloat[0];		powerActiveB = valueFloat[1];		powerActiveC = valueFloat[2];		PowerActiveCombine = valueFloat[3];
	powerReactiveA = valueFloat[4];		powerReactiveB = valueFloat[5];		powerReactiveC = valueFloat[6];		powerReactiveCombine = valueFloat[7];
	powerApparentA = valueFloat[8];		powerApparentB = valueFloat[9];		powerApparentC = valueFloat[10];	powerApparentCombine = valueFloat[11];
	rmsVoltageA = valueFloat[12];		rmsVoltageB = valueFloat[13];		rmsVoltageC = valueFloat[14];		rmsVoltageVector = valueFloat[15];
	rmsCurrentA = valueFloat[16];		rmsCurrentB = valueFloat[17];		rmsCurrentC = valueFloat[18];		rmsCurrentVector = valueFloat[19];
	powerFactorA = valueFloat[20];		powerFactorB = valueFloat[21];		powerFactorC = valueFloat[22];		powerFactorCombine = valueFloat[23];
	energyActiveA = valueFloat[24];		energyActiveB = valueFloat[25];		energyActiveC = valueFloat[26];		energyActiveCombine = valueFloat[27];
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
