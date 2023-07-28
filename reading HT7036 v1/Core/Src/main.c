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
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HT7036.h"
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
uint8_t addrSensor[] = {
		// POWER REGISTER >> 20 addrs
		r_Pa,				r_Pb,				r_Pc,				r_Pt,
		r_Qa,				r_Qb,				r_Qc,				r_Qt,
		r_Sa,				r_Sb,				r_Sc,				r_St,
		r_LinePa,			r_LinePb,			r_LinePc,			r_LinePt,
		r_LineQa,			r_LineQb,			r_LineQc,			r_LineQt,
		// RMS REGISTER >> 14 addrs
		r_UaRms,			r_UbRms,			r_UcRms,			r_UtRms,
		r_IaRms,			r_IbRms,			r_IcRms,			r_ItRms,
		r_LineUaRrms,		r_LineUbRrms, 		r_LineUcRrms,
		r_LineIaRrms, 		r_LineIbRrms,		r_LineIcRrms,
		// POWER FACTOR REGISTER >> 4 addrs
		r_Pfa,				r_Pfb,				r_Pfc,				r_Pft,
		// ENERGY REGISTER >> 8 addrs
		r_Epa,				r_Epb, 				r_Epc,				r_Ept,
		r_Eqa,				r_Eqb,				r_Eqc,				r_Eqt
		// TOTAL REGISTER >> 46
};
uint8_t sizeSensor = sizeof(addrSensor)/sizeof(addrSensor[0]);
uint32_t valueSensor[46];
float valueFloat[46];

// POWER REGISTER
float 	powerActiveA,		powerActiveB,		powerActiveC,		PowerActiveCombine,
		powerReactiveA,		powerReactiveB,		powerReactiveC,		powerReactiveCombine,
		powerApparentA,		powerApparentB,		powerApparentC,		powerApparentCombine,
		powerActiveWaveA,	powerActiveWaveB,	powerActiveWaveC,	powerActiveWaveSum,
		powerReactiveWaveA,	powerReactiveWaveB,	powerReactiveWaveC,	powerReactiveWaveCombine;

// RMS REGISTER
float 	rmsVoltageA,		rmsVoltageB,		rmsVoltageC,		rmsVoltageVector,
		rmsCurrentA,		rmsCurrentB,		rmsCurrentC,		rmsCurrentVector,
		rmsVoltageWaveA,	rmsVoltageWaveB,	rmsVoltageWaveC,
		rmsCurrentWaveA,	rmsCurrentWaveB,	rmsCurrentWaveC;

// POWER FACTOR REGISTER
float 	powerFactorA,		powerFactorB,		powerFactorC, 		powerFactorCombine;

// ENERGY REGISTER
float 	energyActiveA,		energyActiveB, 		energyActiveC,		energyActiveCombine,
		energyReactiveA,	energyReactiveB, 	energyReactiveC, 	energyReactiveCombine;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  uint32_t dataRX[46];


//  dataRX[0] = powerCalculateCalib(VRMS_GAIN, 589824, 12);

  // TESTING
//  spiCommandSpecial(w_calib, BYTE_ENABLE);
//  spiCommandSpecial(w_read_calib, BYTE_ENABLE);
//  spiWrite24(w_ModeCfg, 0xF9FE);
//  spiCommandSpecial(w_calib, BYTE_DISABLE);
//  spiCommandSpecial(w_read_calib, BYTE_DISABLE);
//  dataRX[0] = spiRead24(r_UaRms);
//  dataRX[1] = spiRead24(r_UbRms);
//  dataRX[2] = spiRead24(r_UcRms);
//
//  powerInit();
//  dataRX[0] = spiRead24(deviceId);
  // SET CALIBRATION

  int addressSize = 3;
  HAL_StatusTypeDef addressStatus[addressSize];
  uint8_t address[] = {
		  w_IaRmsoffse,
		  w_IbRmsoffse,
		  w_IcRmsoffse,
		  w_UaRmsoffse,
		  w_UbRmsoffse,
		  w_UcRmsoffse,
		  w_UgainA,
		  w_UgainB,
		  w_UgainC
  };
  uint32_t addressData[] = {
		  6,
		  6,
		  6,
		  6,
		  6,
		  6
  };
  powerRestoreCalib();
  powerSetup(address,addressData,addressStatus,addressSize);
  powerReadSensor(addrSensor, valueSensor, valueFloat, 46);
  dataRX[0] = powerCalculateCalib(VRMS_GAIN, valueSensor[20], 50);
  address[0] = w_UgainA;
  address[1] = w_UgainB;
  address[2] = w_UgainC;
  addressData[0] = dataRX[0];
  addressData[1] = dataRX[0];
  addressData[2] = dataRX[0];
  powerCalib(address, addressData, addressStatus, 3);
  for(;;){
	  powerReadSensor(addrSensor, valueSensor, valueFloat, 46);
	  HAL_Delay(75);

  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
