/*
 * Drv_us100.c
 *
 *  Created on: 2018��5��5��
 *      Author: Cj_Song
 * 	Desprition:	�������ײ�����
 * 	  Function:	ͨ�������������߶Ȳ�����
 */
#ifndef _DRV_US100_H_
#define _DRV_US100_H_

#include "include.h"

extern float us100_height;
extern float us100_origin_height;	

extern void Timer3B_Init(void);         //������������ʱ����ʼ��
extern void US100_HeightMeasure_Init(void);   //�߶Ȳ�����ʼ����������ʱ���ĳ�ʼ��
extern void US100_Height_Measure(void);       //���͸߶Ȳ����źš�
extern void Timer3B_IntHandler(void);   //��ʱ��0�жϴ�����ȡ�߶����ݣ��浽����air_height.

#endif

/* END OF FILE */
