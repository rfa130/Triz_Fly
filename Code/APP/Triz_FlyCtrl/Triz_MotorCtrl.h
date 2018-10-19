
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _TRIZ_MOTOR_CTRL_H_
#define _TRIZ_MOTOR_CTRL_H_

/* Includes ------------------------------------------------------------------*/
#include "include.h"

/* Exported types ------------------------------------------------------------*/

typedef struct
{
	s32 ct_val_rol;
	s32 ct_val_pit;
	s32 ct_val_yaw; 
	s32 ct_val_thr;
} _mc_st;
extern _mc_st mc;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void Motor_Ctrl_Task(u8 dT_ms);

extern s16 motor[MOTORSNUM];
#endif
