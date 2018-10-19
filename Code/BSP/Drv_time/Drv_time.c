//-----------------------------------------------------------------------------
// Drv_time.c
//
//  Created on	: 2018-4-17
//      Author	: Divenire
//		version	: V1.1
//		brief	:
//-----------------------------------------------------------------------------
// Attention:
//
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "Tivaware\system_TM4C12x.h"
#include "BSP\Drv_include.h"

volatile uint32_t sysTickUptime = 0;
System_Time_TypeDef System_Time = {0}; //系统时间

static void sys_time(void);
//static void System_Time_Running(void);
void SysTick_Configuration(void)
{
    uint32_t cnt;
    cnt = Tiva_SysClock / TICK_PER_SECOND;
    SysTickPeriodSet(cnt);
    /* 使能Systick外设 */
    SysTickEnable();
    /* 使能Systick中断 */
    SysTickIntEnable();
}
/*****************************************************************************
* Function Name   : SysTick_GetTick
* Input           : None
* Output          : None
* Return          : None
* Description     :
*****************************************************************************/
uint32_t systime_ms = 0;

uint32_t SysTick_GetTick(void)
{
    return systime_ms;
}
/*****************************************************************************
* Function Name   : delay_ms
* Input           : None
* Output          : None
* Return          : None
* Description     : 精度略高
*****************************************************************************/
void delay_us(uint32_t us)
{
    uint64_t now;
    now = GetSysTime_us();
    while (GetSysTime_us() - now < us)
        ;
}
/*****************************************************************************
* Function Name   : delay_us
* Input           : None
* Output          : None
* Return          : None
* Description     : 精度为0.002%
*****************************************************************************/
void delay_ms(uint32_t ms)
{
    while (ms--)
        delay_us(1000);
}
/*****************************************************************************
* Function Name   : SysTick_Handler
* Input           : None
* Output          : None
* Return          : None
* Description     : 	滴答定时器中断函数
*											系统时间ms数sysTickUptime++
*****************************************************************************/
void SysTick_Handler(void)
{
    sysTickUptime++;
    sys_time();

    //		System_Time_Running(); //调试时间用。
}
/*****************************************************************************
* Function Name   : GetSysTime_us
* Input           : None
* Output          : None
* Return          : 当前的系统时间单位为us
* Description     : 	58W年以后溢出。。。
*****************************************************************************/
uint64_t GetSysTime_us(void)
{
    volatile uint32_t ms;
    uint32_t value;
    ms = sysTickUptime;
    //		Value_reload = SysTickPeriodGet(); (HWREG(NVIC_ST_RELOAD) + 1)
    //		Value_Current = SysTickValueGet(); (HWREG(NVIC_ST_CURRENT)))

    //直接操作寄存器
    value = ms * TICK_US + ((HWREG(NVIC_ST_RELOAD) + 1) - (HWREG(NVIC_ST_CURRENT))) * TICK_US / (HWREG(NVIC_ST_RELOAD) + 1);
    return value;
}
//用于任务调度
static void sys_time()
{
    systime_ms++;
}
