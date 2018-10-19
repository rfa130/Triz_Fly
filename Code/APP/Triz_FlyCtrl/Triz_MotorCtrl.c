
#include "APP/App_include.h"
#include "BSP/Drv_include.h"

/*
四轴：
      机头
   m2     m1
     \   /
      \ /
      / \
     /   \
   m3     m4
      屁股
*/
s16 motor[MOTORSNUM];
s16 motor_step[MOTORSNUM];
//float motor_lpf[MOTORSNUM];

static u16 motor_prepara_cnt;
_mc_st mc;
#define IDLING 100
void Motor_Ctrl_Task(u8 dT_ms)
{
	u8 i;
	
//	if(flag.taking_off)
//	{
//		flag.motor_preparation = 1;
//		motor_prepara_cnt = 0;			
//	}
	
	if(flag.fly_ready)
	{		
		if(flag.motor_preparation == 0)
		{
			motor_prepara_cnt += dT_ms;
			
			if(flag.motor_preparation == 0)
			{			
				if(motor_prepara_cnt<300)
				{
					motor_step[m1] = IDLING;
				}
				else if(motor_prepara_cnt<600)
				{
					motor_step[m2] = IDLING;
				}
				else if(motor_prepara_cnt<900)
				{
					motor_step[m3] = IDLING;
				}	
				else if(motor_prepara_cnt<1200)
				{	
					motor_step[m4] = IDLING;
				}
				else
				{
					flag.motor_preparation = 1;
					motor_prepara_cnt = 0;
				}
			}
			
		}	
	}
	else
	{
		flag.motor_preparation = 0;
	}
	

			
	if(flag.motor_preparation == 1)
	{		
		motor_step[m1] = mc.ct_val_thr -mc.ct_val_rol +mc.ct_val_pit +mc.ct_val_yaw;
		motor_step[m2] = mc.ct_val_thr +mc.ct_val_rol +mc.ct_val_pit -mc.ct_val_yaw;
		motor_step[m3] = mc.ct_val_thr +mc.ct_val_rol -mc.ct_val_pit +mc.ct_val_yaw;
		motor_step[m4] = mc.ct_val_thr -mc.ct_val_rol -mc.ct_val_pit -mc.ct_val_yaw;
	}	
		
	
	for(i=0;i<MOTORSNUM;i++)
	{
		if(flag.fly_ready)
		{
			if(flag.motor_preparation == 1)
			{
				motor_step[i] = LIMIT(motor_step[i],IDLING,1000);
			}
	
		}
		else
		{		
			motor_step[i] = 0;
		}	
		//motor[i] += 0.5f *(motor_step[i] - motor[i]);//LPF_1_(80,dT,motor_step[i],motor_lpf[i]);
		motor[i] = motor_step[i];		
	}

	//motor_out(motor);
	PWM_Set(motor);
}

