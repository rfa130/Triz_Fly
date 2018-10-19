
#ifndef _TRIZ_ALTCTRL_H_
#define _TRIZ_ALTCTRL_H_

#include "include.h"


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void Alt_1level_Ctrl(float dT_s);
void Alt_1level_PID_Init(void);

void Alt_2level_PID_Init(void);
void Alt_2level_Ctrl(float dT_s);

void Auto_Take_Off_Land_Task(u8 dT_ms);

#endif
