/*
 * capture.h
 *
 *  Created on: 2018年5月4日
 *      Author: CJ_S
 */
#include "BSP\Drv_include.h"
#include "APP\APP_include.h"
uint8_t temp;

//摇杆触发值，摇杆值范围为+-500，超过300属于触发范围
#define UN_YAW_VALUE 300
#define UN_THR_VALUE 300
#define UN_PIT_VALUE 300
#define UN_ROL_VALUE 300

u16 Rc_Pwm_In[CH_NUM];
u16 Rc_Ppm_In[CH_NUM];
u16 Rc_Sbus_In[CH_NUM]; //遥控器通道值量化后

unsigned char ch_value_temp[27];

void Capture_Init() //初始化遥控器捕获
{
    Uart2_Init(100000);
}

void Uart2_IntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART2_BASE, true);
    UARTIntClear(UART2_BASE, ui32Status);
    while (UARTCharsAvail(UART2_BASE))
    {
        temp = UARTCharGetNonBlocking(UART2_BASE);
        Rc_ValueGet(temp);
    }
}

u16 signal_intensity;

s16 CH_N[CH_NUM] = {0, 0, 0, 0};

_stick_f_lp_st unlock_f;
u8 stick_fun_0;
u16 unlock_time = 200;

void unlock(u8 dT_ms)
{
    if (flag.power_state <= 2 && para_sta.save_trig == 0) //只有电池电压非最低并且没有操作flash时，才允许进行解锁
    {
        if (sens_hd_check.acc_ok && sens_hd_check.gyro_ok)
        {
            if (sens_hd_check.baro_ok)
            {
                if (flag.sensor_ok) //传感器正常时，才允许解锁
                {
                    flag.unlock_en = 1; //允许解锁标志位
                    if (LED.State == 10)
                        LED.State = 0;
                }
                else
                {
                    flag.unlock_en = 0; //传感器异常，不允许解锁
                    if (LED.State > 115)
                        LED.State = 10;
                }
            }
            else
            {
                LED.State = 82;
                flag.unlock_en = 0; //气压计异常，不允许解锁。
            }
        }
        else
        {
            LED.State = 80;
            flag.unlock_en = 0; //惯性传感器异常，不允许解锁。
        }
    }
    else
    {
        flag.unlock_en = 0; //电池电压异常，不允许解锁
    }
    ////////////////////////////////////////////
    //所有功能判断，都要油门在低值时才进行
    if (CH_N[CH_THR] < -UN_THR_VALUE)
    {
        //判断用户是否想要上锁、解锁
        if (ABS(CH_N[CH_YAW]) > 0.1f * UN_YAW_VALUE && CH_N[CH_PIT] < -0.1f * UN_PIT_VALUE)
        {
            if (flag.locking == 0)
            {
                flag.locking = 1;
            }
        }
        else
        {
            flag.locking = 0;
        }

        //飞控上锁、解锁检测
        if (CH_N[CH_PIT] < -UN_PIT_VALUE && CH_N[CH_ROL] > UN_ROL_VALUE && CH_N[CH_YAW] < -UN_YAW_VALUE)
        {
            stick_fun_0 = 1;
            flag.locking = 2;
        }
        else if (CH_N[CH_PIT] < -UN_PIT_VALUE && CH_N[CH_ROL] < -UN_ROL_VALUE && CH_N[CH_YAW] > UN_YAW_VALUE)
        {
            stick_fun_0 = 1;
            flag.locking = 2;
        }
        else
        {
            stick_fun_0 = 0;
        }

        u8 f = 0;
        if (flag.fly_ready)
        {
            //如果为解锁状态，最终f=0，将f赋值给flag.fly_ready，飞控完成上锁
            f = 0;
            unlock_time = 1000;
        }
        else
        {
            //如果飞控为锁定状态，并且允许解锁，则f=2，将f赋值给flag.fly_ready，飞控解锁完成
            if (flag.unlock_en)
            {
                f = 2;
            }
            else
            {
                f = 0;
                if (LED.State > 115)
                {
                    LED.State = 18;
                }
            }
            unlock_time = 200;
        }
        //进行最终的时间积分判断，摇杆必须满足条件unlock_time时间后，才会执行锁定和解锁动作
        stick_function_check_longpress(dT_ms, &unlock_f, unlock_time, stick_fun_0, f, &flag.fly_ready);
    }
    else
    {
        flag.locking = 0; //油门高
        if (flag.fly_ready == 2)
        {
            flag.fly_ready = 1;
        }
    }

    if (CH_N[CH_THR] > -350)
    {
        flag.thr_low = 0; //油门非低
    }
    else
    {
        flag.thr_low = 1; //油门拉低
    }
}

void RC_duty_task(u8 dT_ms) //建议2ms调用一次
{
    if (flag.start_ok)
    {
        /////////////获得通道数据////////////////////////
        if (Triz_Parame.set.pwmInMode == PWM)
        {
            for (u8 i = 0; i < CH_NUM; i++)
            {
                if (Rc_Pwm_In[i] != 0) //该通道有值，=0说明该通道未插线
                {
                    CH_N[i] = 1.2f * ((s16)Rc_Pwm_In[i] - 1500); //1100 -- 1900us,处理成+-500摇杆量
                }
                else
                {
                    CH_N[i] = 0;
                }
                CH_N[i] = LIMIT(CH_N[i], -500, 500); //限制到+—500
            }
        }
        else if (Triz_Parame.set.pwmInMode == PPM)
        {
            for (u8 i = 0; i < CH_NUM; i++)
            {
                if (Rc_Ppm_In[i] != 0) //该通道有值，=0说明该通道未插线
                {
                    CH_N[i] = 1.2f * ((s16)Rc_Ppm_In[i] - 1100); //1100 -- 1900us,处理成+-500摇杆量
                }
                else
                {
                    CH_N[i] = 0;
                }
                CH_N[i] = LIMIT(CH_N[i], -500, 500); //限制到+—500
            }
        }
        else if (Triz_Parame.set.pwmInMode == SBUS)
        {
            for (u8 i = 0; i < CH_NUM; i++)
            {
                if (Rc_Sbus_In[i] != 0)
                {
                    CH_N[i] = 0.72042f * ((s16)Rc_Sbus_In[i] - 1000);
                }
            }
        }

        ///////////////////////////////////////////////
        //解锁监测
        unlock(dT_ms);
        //摇杆触发功能监测
        stick_function(dT_ms);
        //通道看门狗
        ch_watch_dog(dT_ms);

        //失控保护检查
        fail_safe_check(dT_ms); //3ms
    }
}

static u16 cwd_cnt[10] ;
u8 chn_en_bit = 0;

void ch_watch_dog(u8 dT_ms)//如果是PPM模式，也只能检测前8通道
{
	for(u8 i = 0;i<8;i++)
	{
		if(cwd_cnt[i]<500)
		{
			cwd_cnt[i] += dT_ms;
			chn_en_bit |= 0x01<<i;
		}
		else
		{
			cwd_cnt[i]=0;
			chn_en_bit &= ~(0x01<<i);
			Rc_Pwm_In[i] = 0;  //把捕获值复位
			Rc_Ppm_In[i] = 0;
            Rc_Sbus_In[i]=0;
		}
	}
}

void Rc_DataProcess(void)
{
    Rc_Sbus_In[2] = (ch_value_temp[2] % 8) * 256 + ch_value_temp[1];
    Rc_Sbus_In[1] = (ch_value_temp[3] % 64) * 32 + ch_value_temp[2] / 8;
    Rc_Sbus_In[0] = (ch_value_temp[5] % 2) * 1024 + ch_value_temp[4] * 4 + ch_value_temp[3] / 64;
    Rc_Sbus_In[3] = (ch_value_temp[6] % 16) * 128 + ch_value_temp[5] / 2;
    Rc_Sbus_In[4] = (ch_value_temp[7] % 128) * 16 + ch_value_temp[6] / 16;
    Rc_Sbus_In[5] = (ch_value_temp[9] % 4) * 512 + ch_value_temp[8] * 2 + ch_value_temp[7] / 128;
    Rc_Sbus_In[6] = (ch_value_temp[10] % 32) * 64 + ch_value_temp[9] / 4;
    Rc_Sbus_In[7] = (ch_value_temp[11]) * 8 + ch_value_temp[10] / 32;
}

void Rc_ValueGet(uint8_t data_receive) //处理通道值
{
    static uint8_t receive_pointer = 0;

    if (data_receive == 0x0f && receive_pointer == 0)
    {
        ch_value_temp[receive_pointer] = data_receive;
        receive_pointer = 1;
    }
    else if (receive_pointer > 0 && receive_pointer <= 24)
    {
        ch_value_temp[receive_pointer] = data_receive;
        receive_pointer++;
    }
    else if (receive_pointer >= 25 && ch_value_temp[23] == 0x00 && ch_value_temp[24] == 0x00)
    {
        Rc_DataProcess();
        ch_value_temp[23] = 0x01; 
        receive_pointer = 0;
    }
    else
    {
        receive_pointer = 0;
    }
}


void fail_safe()
{
	for(u8 i = 0;i<4;i++)
	{
		CH_N[i] = 0;
	}

	if(CH_N[CH_THR]>0)
	{
		CH_N[CH_THR] = 0;
	}

	CH_N[CH_ROL] = 0;
	CH_N[CH_PIT] = 0;
	CH_N[CH_YAW] = 0;
}

u16 test_si_cnt;

void fail_safe_check(u8 dT_ms) //dT秒调用一次
{
	static u16 cnt;
	static s8 cnt2;
	
	cnt += dT_ms;
	if(cnt >= 500) //500*dT 秒
	{
		cnt=0;
		if((chn_en_bit & 0x0F) != 0x0F) //前4通道有任意一通道无信号
		{
			cnt2 ++;
		}
		else
		{
			cnt2 --;	
		}
		
		if(cnt2>=2)
		{
			cnt2 = 0;
			
			flag.rc_loss = 1; //认为丢失遥控信号
			
			fail_safe();
			
			if(LED.State >115 && LED.State!= 9)
			{
				LED.State = 11;
			}
		
			if(flag.fly_ready)
			flag.auto_take_off_land = AUTO_LAND; //如果解锁，自动降落标记置位
				
		}
		else if(cnt2<=-2) //认为信号正常
		{
			cnt2 = 0;
			
			flag.rc_loss = 0;
			if(LED.State == 11 || LED.State == 9)
			{
					LED.State = 0;
				
					if(flag.taking_off)
					flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
			}
		}
		
		test_si_cnt = signal_intensity;
		signal_intensity=0; //累计接收次数
	}
	
	
}


void stick_function_check(u8 dT_ms,_stick_f_c_st *sv,u8 times_n,u16 reset_time_ms,u8 en,u8 trig_val,u8 *trig)
{
	if(en)
	{
		sv->s_cnt = 0; //清除计时
		if(sv->s_state==0)
		{
			if(sv->s_now_times!=0)
			{
				sv->s_now_times++;
			}
			sv->s_state = 1;
		}
	}
	else
	{
		sv->s_state = 0;
		/////
		sv->s_cnt += dT_ms;
		if(sv->s_cnt>reset_time_ms)
		{
			sv->s_now_times = 1; //清除记录次数
		}
	}

	if(sv->s_now_times> times_n)
	{
		*trig = trig_val;            //触发功能标记
		sv->s_now_times = 0;
	}

}

void stick_function_check_longpress(u8 dT_ms,u16 *time_cnt,u16 longpress_time_ms,u8 en,u8 trig_val,u8 *trig)
{
	//dT_ms：调用间隔时间
	//time_cnt：积分时间
	//longpress_time_ms：阈值时间，超过这个时间则为满足条件
	//en：摇杆状态是否满足
	//trig_val：满足后的触发值
	//trig：指向需要触发的寄存器
	if(en)//如果满足摇杆条件，则进行时间积分
	{
		if(*time_cnt!=0)
		{
			*time_cnt+=dT_ms;
		}
	}
	else//不满足条件，积分恢复1
	{
		*time_cnt=1;
	}
	//时间积分满足时间阈值，则触发标记
	if(*time_cnt>=longpress_time_ms)
	{
		*trig = trig_val;            //触发功能标记
		*time_cnt = 0;
	}

}

_stick_f_lp_st cali_gyro,cali_acc,cali_surface;
_stick_f_c_st cali_mag;

u8 stick_fun_1,stick_fun_2,stick_fun_3,stick_fun_4,stick_fun_5_magcali;
void stick_function(u8 dT_ms)
{
	//////////////状态监测
	if(flag.fly_ready == 0)
	{
		if(flag.thr_low)
		{

			if(CH_N[CH_PIT]<-350 && CH_N[CH_ROL]>350 && CH_N[CH_THR]<-350 && CH_N[CH_YAW]>350)
			{
				stick_fun_1 = stick_fun_2 = 1;
			}
			else
			{
				stick_fun_1 = stick_fun_2 = 0;
			}
			
			if(CH_N[CH_PIT]>350 && CH_N[CH_ROL]>350 && CH_N[CH_THR]<-350 && CH_N[CH_YAW]<-350)
			{
				stick_fun_3 = 1;
			}
			else
			{
				stick_fun_3 = 0;
			}
			
			if(CH_N[CH_PIT]>350 && CH_N[CH_ROL]>350 && CH_N[CH_THR]<-350 && CH_N[CH_YAW]>350)
			{
				stick_fun_4 = 1;
			}
			else
			{
				stick_fun_4 = 0;
			}
			
			if(CH_N[CH_PIT]>350)
			{
				stick_fun_5_magcali =1;
			}
			else if(CH_N[CH_PIT]<50)
			{
				stick_fun_5_magcali =0;
			}
		}
		
			///////////////
		stick_function_check_longpress(dT_ms,&cali_gyro,1000,stick_fun_1,1,&sensor.gyr_CALIBRATE);
		
		stick_function_check_longpress(dT_ms,&cali_acc,1000,stick_fun_2,1,&sensor.acc_CALIBRATE);
		
//		stick_function_check_longpress(dT_ms,&cali_surface,1000,stick_fun_4,1,&sensor_rot.surface_CALIBRATE );
		
		stick_function_check(dT_ms,&cali_mag,5,1000,stick_fun_5_magcali,1,&mag.mag_CALIBRATE);

		
	}

	//////////////
}
