
#include "APP/App_include.h"
#include "BSP/Drv_include.h"
#include "Common/Com_include.h"


float Plane_Votage = 0;
static float voltage_f = 30000;
static u8 voltage_init_ok;
void Power_UpdateTask(u8 dT_ms)
{
	static s16 voltage_s16;
	float cut_off_freq;
	
	voltage_s16 = AdcValue *10.4677f;//1.128f; ;
	
	if(voltage_init_ok == 0)
	{
		cut_off_freq = 2.0f;
		
		if(voltage_f >2000 && ABS(voltage_s16 - voltage_f) <200)
		{
			voltage_init_ok = 1;
		}
	}	
	else
	{
		cut_off_freq = 0.05f;
	}
	
	LPF_1_(cut_off_freq,dT_ms*1e-3f,voltage_s16,voltage_f);
	

	
	Plane_Votage = voltage_f *0.001f;
		
	if(Plane_Votage<Triz_Parame.set.lowest_power_voltage)
	{
		flag.power_state = 3;
		
	}

	if(Plane_Votage<Triz_Parame.set.warn_power_voltage)
	{
		if(LED.State>115)
		{
			LED.State = 1;
		}
	}
		
	if(Plane_Votage<Triz_Parame.set.return_home_power_voltage)
	{
	
	
	}
}
