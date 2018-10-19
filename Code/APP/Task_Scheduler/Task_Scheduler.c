//-----------------------------------------------------------------------------
// Task_Scheduler.c
//
//  Created on	: 2018-5-13
//      Author	: Divenire
//		version	: V1.0
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

uint32_t test_dT_1000hz[3], test_rT[6];

static void Loop_1000Hz(void) //1ms执行一次
{
    //static int cnt = 0;
    //////////////////////////////////////////////////////////////////////
    test_rT[0] = GetSysTime_us();
    /*传感器数据读取*/
    Fc_Sensor_Get();
    /*惯性传感器数据准备*/
    Sensor_Data_Prepare(1);
    /*姿态解算更新*/
    IMU_Update_Task(1);
    /*获取WC_Z加速度*/
    WCZ_Acc_Get_Task();
    /*飞行状态任务*/
    Flight_State_Task(1, CH_N);
    /*开关状态任务*/
    Swtich_State_Task(1);

    /*姿态角速度环控制*/
    Att_1level_Ctrl(1e-3f);

    /*电机输出控制*/
    Motor_Ctrl_Task(1);
	
	Drv_LED_1ms();
   
    test_rT[1] = GetSysTime_us();
    test_rT[2] = (uint32_t)(test_rT[1] - test_rT[0]);
    //////////////////////////////////////////////////////////////////////
}

static void Loop_500Hz(void) //2ms执行一次
{
	/*数传数据交换*/
    ANO_DT_Data_Exchange();
}

static void Loop_200Hz(void) //5ms执行一次
{
  
}

static void Loop_100Hz(void) //10ms执行一次
{
    //test_rT[0]= GetSysTime_us ();

    /*遥控器数据处理*/
	RC_duty_task(10);
	
	/*飞行模式设置任务*/
	Flight_Mode_Set(10);
	
	/*获取姿态角（ROLL PITCH YAW）*/
	calculate_RPY();
	
	/*姿态角度环控制*/
	Att_2level_Ctrl(10e-3f,CH_N);
	
	/*位置速度环控制（暂无）*/
	Loc_1level_Ctrl(10,CH_N);
	
	/*高度数据融合任务*/
	WCZ_Fus_Task(10);
	/*高度速度环控制*/
	Alt_1level_Ctrl(10e-3f);
	
	/*高度环控制*/
	Alt_2level_Ctrl(10e-3f);

	/*灯光控制*/	
	Drv_LED_Task(10);

    US100_Height_Measure();
//////////////////////////////////////////////////////////////////////		
			//test_rT[1]= GetSysTime_us ();
			//test_rT[2] = (u32)(test_rT[1] - test_rT[0]) ;
}

static void Loop_50Hz(void) //20ms执行一次
{
    Mag_Update_Task(20);
}

static void Loop_20Hz(void) //50ms执行一次
{   
    /*电源电压获取任务*/
    ADC_ValueGet();

    /*电压相关任务*/
    Power_UpdateTask(50);
}

static void Loop_2Hz(void) //500ms执行一次
{
}
//系统任务配置，创建不同执行频率的“线程”
static sched_task_t sched_tasks[] =
    {
        {Loop_1000Hz, 1000, 0, 0},
        {Loop_500Hz, 500, 0, 0},
        {Loop_200Hz, 200, 0, 0},
        {Loop_100Hz, 100, 0, 0},
        {Loop_50Hz, 50, 0, 0},
        {Loop_20Hz, 20, 0, 0},
        {Loop_2Hz, 2, 0, 0},
};
//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks) / sizeof(sched_task_t))

void Scheduler_Setup(void)
{
    uint8_t index = 0;
    //初始化任务表
    for (index = 0; index < TASK_NUM; index++)
    {
        //计算每个任务的延时周期数
        sched_tasks[index].interval_ticks = TICK_PER_SECOND / sched_tasks[index].rate_hz;
        //最短周期为1，也就是1ms
        if (sched_tasks[index].interval_ticks < 1)
        {
            sched_tasks[index].interval_ticks = 1;
        }
    }
}
//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
void Scheduler_Run(void)
{
    uint8_t index = 0;
    //循环判断所有线程，是否应该执行
    for (index = 0; index < TASK_NUM; index++)
    {
        //获取系统当前时间，单位MS
        uint32_t tnow = SysTick_GetTick();
        //进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
        if (tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
        {

            //更新线程的执行时间，用于下一次判断
            sched_tasks[index].last_run = tnow;
            //执行线程函数，使用的是函数指针
            sched_tasks[index].task_func();
        }
    }
}
