/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _TRIZ_MAGPROCESS_H_
#define _TRIZ_MAGPROCESS_H_
/* Includes ------------------------------------------------------------------*/
#include "include.h"
/* Exported types ------------------------------------------------------------*/

typedef struct
{
	u8 mag_CALIBRATE;
//	s16 offset[VEC_XYZ];
//	float gain[VEC_XYZ];
	s16 val[VEC_XYZ];

}_mag_cal_st;
extern _mag_cal_st mag;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void Mag_Data_Deal_Task(u8 dT_ms,s16 mag_in[],float z_vec_z,float gyro_deg_x,float gyro_deg_z);
#endif
