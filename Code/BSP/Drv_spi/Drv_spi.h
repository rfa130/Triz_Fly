#ifndef _DRV_SPI_H_
#define _DRV_SPI_H_

#include "stdint.h"
#include "stdio.h"

void Drv_SPI2_Init(void);
void Drv_SPI2_Transmit(uint8_t *pData, uint16_t Size);
void Drv_SPI2_Receive(uint8_t *pData, uint16_t Size);
uint8_t Drv_SPI2_RW(uint8_t value);

#endif
