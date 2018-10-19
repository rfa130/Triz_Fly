//-----------------------------------------------------------------------------
// main.c
//
//  Created on	: 2018-4-17
//      Author	: Divenire
//		version	: V1.1
//		brief	:
//-----------------------------------------------------------------------------
// Attention:
//
//
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "APP/App_include.h"
#include "BSP/Drv_include.h"
#include "Common/Com_include.h"
//=======================================================================================
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
//=======================================================================================
static uint8_t System_Init(void);

int main(void)
{
    flag.start_ok=System_Init();
    Scheduler_Setup(); //调度器初始化，系统为裸奔，这里人工做了一个时分调度器
    while (1)
    {
        Scheduler_Run(); //运行任务调度器，所有系统功能，除了中断服务函数，都在任务调度器内完成
    }
}

static uint8_t System_Init(void)
{
    /*系统滴答定时器初始化*/
    SysTick_Configuration();
    /*LED初始化*/
    Drv_LED_Init();
   
    /*PWM初始化*/
    PWM_Init();
    /*初始化SPI*/
    Drv_SPI2_Init();
    
    /*ICM20602初始化*/
    Drv_Icm20602CSPin_Init();
    //icm陀螺仪加速度计初始化，若初始化成功，则将陀螺仪和加速度的初始化成功标志位赋值
    sens_hd_check.gyro_ok = sens_hd_check.acc_ok = Drv_Icm20602Reg_Init();
	
	/*AK8975初始化*/
    Drv_AK8975CSPin_Init();
    /*气压计初始化*/
    Drv_SPL06CSPin_Init();
	
	sens_hd_check.mag_ok = 0;       //标记罗盘OK	
    sens_hd_check.baro_ok=Drv_Spl0601_Init();
    /*串口0初始化*/
    //Uart0_Init(115200);
    
	/*遥控器捕获初始化*/
    Capture_Init();
    /*蓝牙模块初始化*/
    Bluetooth_Init();
	/*us100高度测量初始化*/
    US100_HeightMeasure_Init();
	/*参数数据初始化*/
    Para_Data_Init();
	Drv_AdcInit();
    ANO_DT_SendString("SYS init OK!",sizeof("SYS init OK!"));
    return(1);
}
