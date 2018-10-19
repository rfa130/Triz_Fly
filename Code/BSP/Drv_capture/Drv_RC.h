/*
 * capture.h
 *
 *  Created on: 2018��5��4��
 *      Author: CJ_S
 */

#ifndef _DRV_RC_H_
#define _DRV_RC_H_

#include "include.h"
#include "APP/APP_include.h"
typedef struct //???????
{
    u16 s_cnt;
    u8 s_now_times;
    u8 s_state;
} _stick_f_c_st;

#define _stick_f_lp_st u16 //??

enum
{
    CH1 = 0,
    CH2,
    CH3,
    CH4,
    CH5,
    CH6,
    CH7,
    CH8
};
//????
extern u8 state[];
//????? +-500
extern s16 CH_N[], RX_CH[8];
extern u16 signal_intensity;
extern u16 Rc_Pwm_In[CH_NUM];
extern u16 Rc_Ppm_In[CH_NUM];
extern u16 Rc_Sbus_In[CH_NUM]; //遥控器通道值量化后

void fail_safe_check(u8 dT_ms);

void stick_function(u8 dT_ms);

void stick_function_check_longpress(u8 dT_ms, u16 *time_cnt, u16 longpress_time_ms, u8 en, u8 trig_val, u8 *trig);

void ch_watch_dog(u8 dT_ms);

void RC_duty_task(u8 dT_ms);

void signal_check_task(u8);

void ch_watch_dog_feed(u8 ch_n);

void Capture_Init(void);
void Rc_ValueGet(uint8_t data_receive);
void Uart2_IntHandler(void);
#endif

/* END OF FILE */
