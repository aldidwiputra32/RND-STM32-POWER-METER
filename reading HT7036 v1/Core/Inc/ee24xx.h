#ifndef	_EE24_H
#define	_EE24_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_i2c.h"

#ifndef	_EE24CONFIG_H
#define	_EE24CONFIG_H

#define		_EEPROM_SIZE_KBIT							16
#define		_EEPROM_USE_FREERTOS                        0
#define		_EEPROM_ADDRESS                             0b10100000
#define 	ERROR 										0
#define 	SUCCESS										0
#define 	TRIG_CLEAR									0


void    ee24_init(I2C_HandleTypeDef* hi2c, bool wp_en, GPIO_TypeDef* wp_gpio, uint16_t wp_pin);
bool    ee24_isConnected(void);
bool    ee24_write(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);
bool    ee24_read(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);
bool    ee24_eraseChip(void);
uint8_t ee24VirtualWrite(uint8_t data, uint16_t startAddr, uint16_t endAddr);
uint8_t ee24VirtualRead(uint8_t * data, uint16_t startAddr, uint16_t endAddr, uint16_t address);

#ifdef __cplusplus
}
#endif

#endif
#endif
