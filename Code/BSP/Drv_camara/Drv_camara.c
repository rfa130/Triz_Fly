/*
 * Drv_carama.c
 *
 *  Created on:  2018年7月19日
 *      Author:  Cj_S
 * Description:  摄像头数据接收
 */
/*
 *  协议说明，起始
 */
#include "BSP\Drv_include.h"

#define DATA_LENGTH 6 //数据包长度

int8_t target_point[2]; //目标点坐标，发过来的数据为int8_t 类型

uint8_t RxBuffer[DATA_LENGTH] = {0X00};

static void Carama_Data_Process(void);

void Carama_Data_Process(void)
{
    //RxBuffer[1]   :   0x80
    //RxBuffer[2]   :   X轴数据
    //RxBuffer[3]   :   Y轴数据
    if (RxBuffer[1] == 0x80)
    {
        target_point[0] = RxBuffer[2];
        target_point[1] = RxBuffer[3];
    }
    else //其他数据包类型
        ;
}

//帧头      ：  0x7F
//帧尾      ：  0x7e
//帧长      ：  6
//RxBuffer[1]   :   数据包类型
//RxBuffer[2]   :   X轴数据(类型0x80)
//RxBuffer[3]   :   Y轴数据(类型0x80)

void Carama_Data_Receive_Prepare(uint8_t data)
{
    static uint8_t state = 0;

    if (state == 0 && data == 0x7F) //帧头0x8F
    {
        state = 1;
        RxBuffer[0] = data;
    }
    else if (state == 1 && data == 0x80) //
    {
        state = 2;
        RxBuffer[1] = data;
    }
    else if (state == 2) //X轴数据 int8_t;
    {
        state = 3;
        RxBuffer[2] = data;
    }
    else if (state == 3) //y轴数据 int8_t;
    {
        state = 4;
        RxBuffer[3] = data;
    }
    else if (state == 4) //校验和
    {
        state = 5;
        RxBuffer[4] = data;
        if (data != (uint8_t)(RxBuffer[2] + RxBuffer[3]))
            state = 0;
    }
    else if (state == 5 && data == 0x7E) //帧尾
    {
        RxBuffer[5] = data;
        Carama_Data_Process();
        state = 0;
    }
    else
        state = 0;
}
