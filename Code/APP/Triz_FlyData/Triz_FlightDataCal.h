
#ifndef __FIGHT_DATA_COMP_H_
#define __FIGHT_DATA_COMP_H_

#include "include.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
extern s32 baro_height, ref_height_get;
extern u16 ref_son_height;
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void Fc_Sensor_Get(void);

void IMU_Update_Task(u8 dT_ms);

void Mag_Update_Task(u8 dT_ms);

void WCZ_Acc_Get_Task(void);

void WCZ_Fus_Task(u8 dT_ms);

#endif
