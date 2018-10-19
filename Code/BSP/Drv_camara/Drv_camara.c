/*
 * Drv_carama.c
 *
 *  Created on:  2018��7��19��
 *      Author:  Cj_S
 * Description:  ����ͷ���ݽ���
 */
/*
 *  Э��˵������ʼ
 */
#include "BSP\Drv_include.h"

#define DATA_LENGTH 6 //���ݰ�����

int8_t target_point[2]; //Ŀ������꣬������������Ϊint8_t ����

uint8_t RxBuffer[DATA_LENGTH] = {0X00};

static void Carama_Data_Process(void);

void Carama_Data_Process(void)
{
    //RxBuffer[1]   :   0x80
    //RxBuffer[2]   :   X������
    //RxBuffer[3]   :   Y������
    if (RxBuffer[1] == 0x80)
    {
        target_point[0] = RxBuffer[2];
        target_point[1] = RxBuffer[3];
    }
    else //�������ݰ�����
        ;
}

//֡ͷ      ��  0x7F
//֡β      ��  0x7e
//֡��      ��  6
//RxBuffer[1]   :   ���ݰ�����
//RxBuffer[2]   :   X������(����0x80)
//RxBuffer[3]   :   Y������(����0x80)

void Carama_Data_Receive_Prepare(uint8_t data)
{
    static uint8_t state = 0;

    if (state == 0 && data == 0x7F) //֡ͷ0x8F
    {
        state = 1;
        RxBuffer[0] = data;
    }
    else if (state == 1 && data == 0x80) //
    {
        state = 2;
        RxBuffer[1] = data;
    }
    else if (state == 2) //X������ int8_t;
    {
        state = 3;
        RxBuffer[2] = data;
    }
    else if (state == 3) //y������ int8_t;
    {
        state = 4;
        RxBuffer[3] = data;
    }
    else if (state == 4) //У���
    {
        state = 5;
        RxBuffer[4] = data;
        if (data != (uint8_t)(RxBuffer[2] + RxBuffer[3]))
            state = 0;
    }
    else if (state == 5 && data == 0x7E) //֡β
    {
        RxBuffer[5] = data;
        Carama_Data_Process();
        state = 0;
    }
    else
        state = 0;
}
