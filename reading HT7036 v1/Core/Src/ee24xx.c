#include "ee24xx.h"

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

//void memoryVirtualWrite(uint64_t data, uint32_t startAddr, uint32_t endAddr){
//	uint64_t value = 0;
//	while(1){
//		memorySingleRead(flashAddrVirtual, &value);
//		if(value == 0xffffffffffffffff){
//			HAL_StatusTypeDef status;
//			HAL_FLASH_Unlock();
//			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flashAddrVirtual, data);
//			HAL_FLASH_Lock();
//			if(status == HAL_OK){
//				break;
//			}
//		}else{
//			flashAddrVirtual += 4;
//			if(flashAddrVirtual > endAddr - 7 ){ // size per address 8 byte (DoubleWord)
//				memoryReset(startAddr,endAddr);
//				flashAddrVirtual = startAddr;
//			}
//		}
//	}
//}
//
//void memoryVirtualRead(uint32_t startAddr, uint32_t startAddrVirtual, uint64_t * data, uint8_t number){
//	uint32_t startAddrScan = startAddr;
//	uint64_t tresholdAddr = 0;
//	while(1){
//		// GET DATA FROM FLASH MEMORY
////		memorySingleRead(startAddrScan, &tresholdAddr);// flashValue = *(uint32_t *)startAddrScan;
//		// SCAN DATA FROM FLASH ADDRESS
//		if(tresholdAddr == 0xffffffffffffffff){
//			// GET LAST ADDRESS WRITED
//			flashAddrVirtual = startAddrScan - 8*number + 8*startAddrVirtual;
//			if(flashAddrVirtual < startAddr){
//				*data = NULL_DATA;
//				flashAddrVirtual = startAddr;
//			}else{
//				// GET DATA FROM FLASH MEMORY
//				memorySingleRead(flashAddrVirtual, data); // *data = *(uint32_t *)startAddr;
//			}
//			break;
//		}else{
//			startAddrScan += 4;
//		}
//	}
//}

// MODIFY END
