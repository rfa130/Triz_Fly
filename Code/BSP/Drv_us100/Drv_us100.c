/*
 *  Drv_us100.c
 *
 *  Created on: 2018年5月4日
 *      Author: CJ_S
 * Description: 超声波底层驱动
 * 
 */

#include "BSP\Drv_include.h"

#define US100_MOVING_AVERAGE 5

float us100_origin_height; //原始高度

float us100_height; //滤波之后的高度数据
float z_speed;      //竖直方向上的速度。
float us100_height_offset;

void Timer3B_Init() //超声波捕获定时器初始化
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3); //使能Wtimer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  //使能GPIO口

    GPIOPinConfigure(GPIO_PD5_T3CCP1);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_5);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_B_CAP_TIME_UP | TIMER_CFG_SPLIT_PAIR); //Configure WTimer0B
    TimerControlEvent(TIMER3_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
    TimerPrescaleSet(TIMER3_BASE, TIMER_B, 255);
	IntMasterEnable();
    TimerIntRegister(TIMER3_BASE, TIMER_B, Timer3B_IntHandler); //Set function for Timer0B interrupt.
    TimerIntEnable(TIMER3_BASE, TIMER_CAPB_EVENT);              //Set Timer0 interrupt when overtime.
    IntEnable(INT_TIMER3B);                                     //Enable Timer0B interrupt in NVIC.
    TimerEnable(TIMER3_BASE, TIMER_B);
}

void US100_HeightMeasure_Init() //超声波模块初始化
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_4);
    Timer3B_Init();
}

void US100_Height_Measure() //发高度测量信号
{
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_PIN_4);
    SysCtlDelay(Tiva_SysClock / 300000);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, 0);
}


void Timer3B_IntHandler(void)
{
    static uint32_t time_cap1;
    static uint16_t height_pt=0; 
    static float origin_height[US100_MOVING_AVERAGE];

    static bool level_flag;

    uint32_t temp;
    uint32_t time_past;
    TimerIntClear(TIMER3_BASE, TIMER_CAPB_EVENT);

    if (level_flag == 0)
    {
        TimerControlEvent(TIMER3_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE); //捕获模式，B定时器，下降沿捕获
        time_cap1 = TimerValueGet(TIMER3_BASE, TIMER_B);
        level_flag = 1;
    }
    else
    {
        TimerControlEvent(TIMER3_BASE, TIMER_B, TIMER_EVENT_POS_EDGE); //捕获模式，B定时器，上升沿捕获
        temp = TimerValueGet(TIMER3_BASE, TIMER_B);
        //test=TimerPrescaleGet(TIMER1_BASE,TIMER_A) ;
        if (temp > time_cap1)
        {
            time_past = temp - time_cap1;
        }
        else
        {
            time_past = 0xffffff - time_cap1 + temp;
        }
        level_flag = 0;

        us100_origin_height = time_past / 120000.0 * 17.0; //计算得原始高度

        Moving_Average(origin_height, US100_MOVING_AVERAGE, &height_pt, us100_origin_height, &us100_height); //滑动滤波，窗口5 

    }
    if(us100_height>1 && us100_height<=300)
    {
        sens_hd_check.sonar_ok=1;
    }
    else
    {
        sens_hd_check.sonar_ok=0;
    }
    
}
/* END OF FILE */
