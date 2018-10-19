#ifndef _USART_H_
#define _USART_H_
#include "include.h"
#include "stdio.h"

void Uart0_putbuff(uint8_t *buff, uint32_t len);
void Uart0_Init(uint32_t bound);

void Uart1_putbuff(uint8_t *buff, uint32_t len);
void Uart1_Init(uint32_t bound);

void Uart2_Init(uint32_t bound);



#endif
