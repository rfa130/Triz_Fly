/*
 *  Drv_pwm.c
 *
 *  Created on: 2018年5月4日
 *      Author: CJ_S
 */
#include "BSP\Drv_include.h"

void PWM_Set(int16_t pwm[4])
{
    uint16_t high[4];
    uint8_t i;
    for (i = 0; i < 4; i++)
    high[i] = (uint16_t)(pwm[i] * PWM_RADIO) + PWM_IDLE_HIGH;
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, high[3]);
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, high[0]);
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, high[2]);
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, high[1]);
}

void PWM_Init()
{
    //使能PWM0模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    //使能PWM1模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    //PWM时钟配置：64分频
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_4);
    //配置引脚为PWM功能
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);
    //使能PWM引脚
    GPIOPinConfigure(GPIO_PK5_M0PWM7);
    GPIOPinConfigure(GPIO_PK4_M0PWM6);
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    //配置PWM发生器0和发生器1：加减计数不分频
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    //设置PWM发生器0的频率，时钟频率/pwm分频数/n,120M/4/400=75000
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PWM_LOAD_VALUE);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWM_LOAD_VALUE);
    //设置PWM1/PWM1输出的脉冲宽度
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PWM_IDLE_HIGH);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, PWM_IDLE_HIGH);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWM_IDLE_HIGH);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, PWM_IDLE_HIGH);

    //使能PWM发生器
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    //使能PWM1和PWM1的输出
    PWMOutputState(PWM0_BASE, (PWM_OUT_4_BIT), true);
    PWMOutputState(PWM0_BASE, (PWM_OUT_5_BIT), true);
    PWMOutputState(PWM0_BASE, (PWM_OUT_6_BIT), true);
    PWMOutputState(PWM0_BASE, (PWM_OUT_7_BIT), true);
}
