/*
 *  Drv_pwm.h
 *
 *  Created on: 2018��5��4��
 *      Author: CJ_S
 */
#ifndef _DRV_PWM_H_
#define _DRV_PWM_H_

#define PWM_LOAD_VALUE 75000
#define PWM_IDLE_HIGH 30000
#define PWM_RADIO 30

void PWM_Set(int16_t pwm[4]);
void PWM_Init(void);

#endif

/* END OF FILE*/
