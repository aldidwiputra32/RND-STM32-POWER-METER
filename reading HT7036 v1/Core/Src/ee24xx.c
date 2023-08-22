#include "ee24xx.h"
#include "main.h"

#if (_EEPROM_USE_FREERTOS == 1)
#include "cmsis_os.h"
#define ee24_delay(x)   osDelay(x)
#else
#define ee24_delay(x)   HAL_Delay(x)
#endif

#if (_EEPROM_SIZE_KBIT == 1) || (_EEPROM_SIZE_KBIT == 2)
#define _EEPROM_PSIZE     8
#elif (_EEPROM_SIZE_KBIT == 4) || (_EEPROM_SIZE_KBIT == 8) || (_EEPROM_SIZE_KBIT == 16)
#define _EEPROM_PSIZE     16
#else
#define _EEPROM_PSIZE     32
#endif

static uint8_t ee24_lock = 0;
static I2C_HandleTypeDef ee_hi2c;
static bool ee_wp_en = 0;
static GPIO_TypeDef* ee_wp_gpiox;
static uint16_t ee_wp_pin;

// MODIFY BEGIN
static uint16_t ee24AddrVirtual=0;
// MODIFY END

void ee24_init(I2C_HandleTypeDef* hi2c, bool wp_en, GPIO_TypeDef* wp_gpio, uint16_t wp_pin)
{
    ee_hi2c = *hi2c;
    ee_wp_en = wp_en;
    ee_wp_gpiox = wp_gpio;
    ee_wp_pin = wp_pin;
}

bool ee24_isConnected(void)
{
  if (ee_wp_en)
  HAL_GPIO_WritePin(ee_wp_gpiox, ee_wp_pin, GPIO_PIN_SET);

  if (HAL_I2C_IsDeviceReady(&ee_hi2c, _EEPROM_ADDRESS, 2, 100)==HAL_OK) return true;
  else return false;
}

bool ee24_read(uint16_t address, uint8_t *data, size_t len, uint32_t timeout)
{
  if (ee24_lock == 1) return false;

  ee24_lock = 1;

  if (ee_wp_en)
  HAL_GPIO_WritePin(ee_wp_gpiox,  ee_wp_pin, GPIO_PIN_SET);

  #if ((_EEPROM_SIZE_KBIT == 1) || (_EEPROM_SIZE_KBIT == 2))
  if (HAL_I2C_Mem_Read(&ee_hi2c, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
  #elif (_EEPROM_SIZE_KBIT == 4)
  if (HAL_I2C_Mem_Read(&ee_hi2c, _EEPROM_ADDRESS | ((address & 0x0100) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
  #elif (_EEPROM_SIZE_KBIT == 8)
  if (HAL_I2C_Mem_Read(&ee_hi2c, _EEPROM_ADDRESS | ((address & 0x0300) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
  #elif (_EEPROM_SIZE_KBIT == 16)
  if (HAL_I2C_Mem_Read(&ee_hi2c, _EEPROM_ADDRESS | ((address & 0x0700) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
  #else
  if (HAL_I2C_Mem_Read(&ee_hi2c, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_16BIT, data, len, timeout) == HAL_OK)
  #endif
  {
    ee24_lock = 0;
    return true;
  }
  else
  {
    ee24_lock = 0;
    return false;
  }
}

bool ee24_write(uint16_t address, uint8_t *data, size_t len, uint32_t timeout)
{
  if (ee24_lock == 1) return false;

  ee24_lock = 1;
  uint16_t curr_wsize;
  uint32_t startTime = HAL_GetTick();

  if (ee_wp_en)
  HAL_GPIO_WritePin(ee_wp_gpiox,  ee_wp_pin, GPIO_PIN_RESET);

  while (1)
  {
    curr_wsize = _EEPROM_PSIZE - (address  % _EEPROM_PSIZE);
    if (curr_wsize > len) curr_wsize = len;
    #if ((_EEPROM_SIZE_KBIT==1) || (_EEPROM_SIZE_KBIT==2))
    if (HAL_I2C_Mem_Write(&ee_hi2c, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, curr_wsize, 100) == HAL_OK)
    #elif (_EEPROM_SIZE_KBIT==4)
    if (HAL_I2C_Mem_Write(&ee_hi2c, _EEPROM_ADDRESS | ((address & 0x0100) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, curr_wsize, 100) == HAL_OK)
    #elif (_EEPROM_SIZE_KBIT==8)
    if (HAL_I2C_Mem_Write(&ee_hi2c, _EEPROM_ADDRESS | ((address & 0x0300) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, curr_wsize, 100) == HAL_OK)
    #elif (_EEPROM_SIZE_KBIT==16)
    if (HAL_I2C_Mem_Write(&ee_hi2c, _EEPROM_ADDRESS | ((address & 0x0700) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, curr_wsize, 100) == HAL_OK)
    #else
    if (HAL_I2C_Mem_Write(&ee_hi2c, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_16BIT, data, curr_wsize, 100) == HAL_OK)
    #endif
    {
      ee24_delay(10);
      len -= curr_wsize;
      data += curr_wsize;
      address += curr_wsize;
      if (len == 0)
      {
        if (ee_wp_en)
        HAL_GPIO_WritePin(ee_wp_gpiox,  ee_wp_pin, GPIO_PIN_SET);

        ee24_lock = 0;
        return true;
      }
      if (HAL_GetTick() - startTime >= timeout)
      {
        ee24_lock = 0;
        return false;
      }
    }
    else
    {
      if (ee_wp_en)
      HAL_GPIO_WritePin(ee_wp_gpiox,  ee_wp_pin, GPIO_PIN_SET);

      ee24_lock = 0;
      return false;
    }
  }
}



bool ee24_eraseChip(void)
{
  const uint8_t eraseData[32] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF\
    , 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint32_t bytes = 0;
  while ( bytes < (_EEPROM_SIZE_KBIT * 256))
  {
    if (ee24_write(bytes, (uint8_t*)eraseData, sizeof(eraseData), 100) == false)
      return false;
    bytes += sizeof(eraseData);
  }
  return true;
}

// MODIFY BEGIN

uint8_t ee24VirtualWrite(uint8_t data, uint16_t startAddr, uint16_t endAddr){
	uint8_t statusFunc = 0; // 0 = error, 1 = success, 2 = triggered clear eeprom
	uint8_t statusWrite = 0;
	uint8_t value = 0;
	while(1){
		ee24_read(ee24AddrVirtual, (uint8_t*)&value, sizeof(value), 100);
		if(value == 0xFF){
			statusWrite = ee24_write(ee24AddrVirtual, (uint8_t*)&data, sizeof(data), 100);
			if(statusWrite){
				statusFunc = 1;
				break;
			}else{
				statusFunc = 0;
				break;
			}
		}else{
			ee24AddrVirtual += 1;
			if(ee24AddrVirtual > endAddr){
				statusFunc = 2;
				ee24AddrVirtual = startAddr;
				ee24_eraseChip();
				break;
			}
		}
	}
	return statusFunc;
}

uint8_t ee24VirtualRead(uint8_t * data, uint16_t startAddr, uint16_t endAddr, uint16_t address){
	uint8_t statusRead = 0; // 0 = error, 1 = success
	uint16_t startAddrScan = startAddr;
	uint8_t valueBuffer[1024];
	uint8_t valueBuffer1 = 0;
	ee24_read(startAddrScan, (uint8_t*)valueBuffer, sizeof(valueBuffer), 100);
	while(1){
		if(valueBuffer[startAddrScan] == 0xFF){
			ee24AddrVirtual = startAddrScan - 64 + address;
			if(ee24AddrVirtual < startAddr){
				*data = 0xFFFF; // NULL Data
				ee24AddrVirtual = startAddr;
				statusRead = 0;
			}else{
				ee24_read(ee24AddrVirtual, (uint8_t*)data, sizeof(uint8_t), 100);
				valueBuffer1 = *data;
				statusRead = 1;
			}
			break;
		}else{
			startAddrScan += 1;
			if(startAddrScan > endAddr){
				statusRead = 0;
				break;
			}
		}
	}
	return statusRead;
}


void ee24Debug(){
	uint8_t eepromBuffer[1024];
	uint8_t dataPrint[300];
	ee24_read(0, (uint8_t*)eepromBuffer, sizeof(eepromBuffer), 1000);
	HAL_GPIO_WritePin(MODBUS_En_GPIO_Port, MODBUS_En_Pin, GPIO_PIN_RESET);
	sprintf(dataPrint,"\r\n=======EEPROM DEBUG=======\r\n");
	HAL_UART_Transmit(&huart2, dataPrint, 40, 1000);
	memset(dataPrint, 0, sizeof(dataPrint));
	for(uint8_t indeks=0;indeks<128;indeks++){
		uint8_t da
		sprintf(dataPrint,"%d[%d], %d[%d], %d[%d], %d[%d], %d[%d], %d[%d], %d[%d], %d[%d]\r\n",
				indeks++, indeks++, indeks++, indeks++, indeks++, indeks++, indeks++, indeks++
		);
		HAL_UART_Transmit(&huart2, &dataPrint, 30, 1000);
		memset(dataPrint, 0, sizeof(dataPrint));
	}

}


//----------------------------------------------------NOTE-----------------------------------------------------------------
//
//uint8_t dataAll[512];
//ee24_eraseChip();
//for(;;);
//ee24_read(0, (uint8_t*)dataAll, sizeof(dataAll), 1000);
//
//for(int indeks=0;indeks<32;indeks++)ee24VirtualRead(&eepromBufferRead[indeks], 0, 1024, indeks);
//
//uint8_t dataPrint[500];
//HAL_GPIO_WritePin(MODBUS_En_GPIO_Port,MODBUS_En_Pin,GPIO_PIN_RESET);
//serialPrint("\r\n------------READ EEPROM-----------\r\n", 40);
//sprintf(dataPrint,"\r\n0[%d], 1[%d], 2[%d], 3[%d], 4[%d]\r\n, 5[%d], 6[%d], 7[%d], 8[%d], 9[%d]\r\n, 10[%d], 11[%d], 12[%d], 13[%d], 14[%d]\r\n, 15[%d], 16[%d], 17[%d], 18[%d], 19[%d]\r\n, 20[%d], 21[%d], 22[%d], 23[%d], 24[%d]\r\n, 25[%d], 26[%d], 27[%d], 28[%d], 29[%d]\r\n, 30[%d], 31[%d]\r\n",
//				  eepromBufferRead[0],eepromBufferRead[1],eepromBufferRead[2],eepromBufferRead[3],
//				  eepromBufferRead[4],eepromBufferRead[5],eepromBufferRead[6],eepromBufferRead[7],
//				  eepromBufferRead[8],eepromBufferRead[9],eepromBufferRead[10],eepromBufferRead[11],
//				  eepromBufferRead[12],eepromBufferRead[13],eepromBufferRead[14],eepromBufferRead[15],
//				  eepromBufferRead[16],eepromBufferRead[17],eepromBufferRead[18],eepromBufferRead[19],
//				  eepromBufferRead[20],eepromBufferRead[21],eepromBufferRead[22],eepromBufferRead[23],
//				  eepromBufferRead[24],eepromBufferRead[25],eepromBufferRead[26],eepromBufferRead[27],
//				  eepromBufferRead[28],eepromBufferRead[29],eepromBufferRead[30],eepromBufferRead[31]
//);
//HAL_UART_Transmit(&huart2, dataPrint, 500, 1000);
//HAL_GPIO_WritePin(MODBUS_En_GPIO_Port,MODBUS_En_Pin,GPIO_PIN_SET);
//memset(dataPrint, 0, sizeof(dataPrint));
//
//for(int indeks=0;indeks<32;indeks++)eepromBufferWrite[indeks]=indeks;
//uint8_t gain = 0;
//
//for(;;){
//	  gain += 1;
//	  for(int indeks=0;indeks<32;indeks++){
//		  eepromBufferWrite[indeks]=indeks+gain;
//		  ee24VirtualWrite(eepromBufferWrite[indeks], 0, 1024);
//	  }
//	  HAL_Delay(5000);
//}
//  for(;;){
//	  eepromEncode(
//			  test16++,test16++,test16++,test16++,test16++,test16++,test16++,
//			  test16++,test16++,test16++,test16++,test16++,	test16++,test16++
//	  );
//	  for(uint8_t indeks1=0;indeks1<64;indeks1++){
//		if(ee24VirtualWrite(eepromBufferWrite[indeks1], 0, 1024) == TRIG_CLEAR){
//			for(uint8_t indeks2=0;indeks2<64;indeks2++)ee24VirtualWrite(eepromBufferWrite[indeks2], 0, 1024);
//			break;
//		}
//	  }
//
//	  HAL_Delay(1000);
//	  ee24_read(0, (uint8_t*)test1024,sizeof(test1024), 1000);
//
//  }
