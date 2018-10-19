/*******************************************
Copyright:Triz
Author:Criss
Date:2018.5.15
Dscript:1.LED所用引脚初始化
		2.判断飞机状态，发出相应的灯光信号
********************************************/
#ifndef _LED_H_
#define _LED_H_

#include "include.h"

#define LED_USER GPIO_PIN_5
#define LED_BLUE GPIO_PIN_1                                              //红灯所用引脚
#define LED_RED GPIO_PIN_2                                               //蓝灯所用引脚
#define LED_GREEN GPIO_PIN_3                                             //绿灯所用引脚

#define LED_USER_ON GPIOPinWrite(GPIO_PORTD_BASE, LED_USER, LED_USER)
#define LED_USER_OFF GPIOPinWrite(GPIO_PORTD_BASE, LED_USER, 0)
#define LED_BLUE_ON GPIOPinWrite(GPIO_PORTK_BASE, LED_BLUE, LED_BLUE)    //红灯开
#define LED_RED_ON GPIOPinWrite(GPIO_PORTK_BASE, LED_RED, LED_RED)       //蓝灯开
#define LED_GREEN_ON GPIOPinWrite(GPIO_PORTK_BASE, LED_GREEN, LED_GREEN) //绿灯开
#define LED_BLUE_OFF GPIOPinWrite(GPIO_PORTK_BASE, LED_BLUE, 0)          //红灯关
#define LED_RED_OFF GPIOPinWrite(GPIO_PORTK_BASE, LED_RED, 0)            //蓝灯关
#define LED_GREEN_OFF GPIOPinWrite(GPIO_PORTK_BASE, LED_GREEN, 0)        //绿灯关
//通过枚举，为不同颜色灯配置相应的数值，当添加新的灯时，灯数会自动增加
//如：希望红灯亮，只需传递Red_Led给函数相应的颜色参数
enum
{
    User,
    G_led,
    R_led,
    B_led,
    LED_NUM,
};

struct DRV_LED
{
    uint8_t State;
    uint8_t State_Old;
    uint8_t Accuracy;
    double Brightness[LED_NUM];
};
extern struct DRV_LED LED;
void Drv_LED_Init(void);
void Drv_LED_1ms(void);
uint8_t Drv_LED_Breath(uint8_t dT_ms, uint8_t i, uint16_t dT);
uint8_t Drv_LED_Flash(uint8_t dT_ms, uint8_t i, uint8_t lb, uint8_t group_n, uint16_t on_ms, uint16_t off_ms, uint16_t group_dT_ms);
void Drv_LED_Task(uint8_t dT_ms);
/********************************************************************************************************
LED状态标志位			状态					灯光
LED.State=1：			开机静止前				白色快闪	
LED.State=2:			无信号未与遥控器连接	红色呼吸
LED.State=3：			解锁前					白色短闪+长间隔	
LED.State=4:			解锁后					绿色短闪+长间隔
LED.State=5:			
LED.State=6:			校准罗盘第一步			黄色快闪
LED.State=7:			校准罗盘第二步			绿色呼吸
LED.State=8:			校准罗盘第三步			紫色快闪
LED.State=9:			校准罗盘第四步			蓝色呼吸
LED.State=10:			校准罗盘成功			绿色
LED.State=11:			校准罗盘失败			红色
LED.State=12:			低压警报				红色短闪+短间隔
LED.State=13:			惯性传感器异常			红色快闪+长间隔(每组闪两次)
LED.State=14:			电子罗盘异常			红色快闪+长间隔(每组闪三次)
LED.State=15:			气压计异常				红色快闪+长间隔(每组闪四次)
LED.State=16:			预留状态				紫色呼吸
LED.State=17:			预留状态				黄色呼吸
LED.State=18:			预留状态				青色呼吸
LED.State=19:			预留状态				三彩灯
LED.State=20:			预留状态				红色呼吸500ms
LED.State=21:			预留状态				蓝色呼吸500ms
LED.State=22:			预留状态				绿色呼吸500ms
********************************************************************************************************/
#endif // !_LED_H_
