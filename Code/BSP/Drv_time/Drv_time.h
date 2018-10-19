#ifndef _Drv_TIME_H_
#define _Drv_TIME_H_
#include "stdint.h"


#define TICK_PER_SECOND	1000
#define TICK_US	(1000000/TICK_PER_SECOND)



/* 系统时间（调试用） */
typedef struct {
		uint16_t Hours;         //时
		uint8_t  Minutes;       //分
		uint8_t  Seconds;       //秒
		uint16_t Milliseconds;  //毫秒
}System_Time_TypeDef;
extern volatile uint32_t sysTickUptime;


void delay_us(uint32_t);
void delay_ms(uint32_t);
void SysTick_Configuration(void);
uint64_t GetSysTime_us(void);
uint32_t SysTick_GetTick(void);
#endif

