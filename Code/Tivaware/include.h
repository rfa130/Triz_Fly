#ifndef __INCLUDE_H_
#define __INCLUDE_H_

#include <stdbool.h>
#include <stdint.h>

#include "system_TM4C12x_Conf.h"
#include "pin_map.h"
#include "APP\Task_Scheduler\Task_Scheduler.h"

//宏定义时候用
#define GPIO_SetBits(ui32Port, ui8Pins) GPIOPinWrite(ui32Port, ui8Pins, ui8Pins)
#define GPIO_ResetBits(ui32Port, ui8Pins) HWREG(ui32Port + (GPIO_O_DATA + (ui8Pins << 2))) = 0

//=======================================
//系统时钟频率        数值在Sysinit里获得!!
#define Tiva_SysClock 120000000

//=======================================
//数据类型定义
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;
typedef volatile int32_t vs32;
typedef volatile int16_t vs16;
typedef volatile int8_t vs8;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef volatile uint32_t vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t vu8;
#endif
