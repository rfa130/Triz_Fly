/*
 * Drv_us100.c
 *
 *  Created on: 2018年5月5日
 *      Author: Cj_Song
 * 	Desprition:	超声波底层驱动
 * 	  Function:	通过超声波测量高度并处理
 */
#ifndef _DRV_US100_H_
#define _DRV_US100_H_

#include "include.h"

extern float us100_height;
extern float us100_origin_height;	

extern void Timer3B_Init(void);         //超声波测量定时器初始化
extern void US100_HeightMeasure_Init(void);   //高度测量初始化，包括定时器的初始化
extern void US100_Height_Measure(void);       //发送高度测量信号。
extern void Timer3B_IntHandler(void);   //宽定时器0中断处理，获取高度数据，存到变量air_height.

#endif

/* END OF FILE */
