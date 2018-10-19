
#include "APP/App_include.h"
#include "BSP/Drv_include.h"
#include "Common/Com_include.h"
/*PID参数初始化*/
void All_PID_Init(void)
{

    /*姿态控制，角速度PID初始化*/
    Att_1level_PID_Init();

    /*姿态控制，角度PID初始化*/
    Att_2level_PID_Init();

    /*高度控制，高度速度PID初始化*/
    Alt_1level_PID_Init();

    /*高度控制，高度PID初始化*/
    Alt_2level_PID_Init();

    /*位置速度控制PID初始化*/
    Loc_1level_PID_Init();
}

/*控制参数改变任务*/
void ctrl_parameter_change_task()
{
    if (0)
    {
        Set_Att_2level_Ki(0);
    }
    else
    {
        if (flag.auto_take_off_land == AUTO_TAKE_OFF)
        {

            Set_Att_1level_Ki(2);
        }
        else
        {

            Set_Att_1level_Ki(1);
        }

        Set_Att_2level_Ki(1);
    }
}

/*一键翻滚（暂无）*/
void one_key_roll()
{

    if (flag.flying && flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH)
    {
        if (rolling_flag.roll_mode == 0)
        {
            rolling_flag.roll_mode = 1;
        }
    }
}

static u16 one_key_taof_start;
/*一键起飞任务（主要功能为延迟）*/
void one_key_take_off_task(u16 dt_ms)
{
    if (one_key_taof_start != 0)
    {
        one_key_taof_start += dt_ms;

        if (one_key_taof_start > 1400 && flag.motor_preparation == 1)
        {
            one_key_taof_start = 0;
            if (flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
            {
                flag.auto_take_off_land = AUTO_TAKE_OFF;
                //解锁、起飞

                flag.taking_off = 1;
            }
        }
    }
}
/*一键起飞*/
void one_key_take_off()
{
    if (flag.unlock_en)
    {
        one_key_taof_start = 1;
        flag.fly_ready = 1;
    }
}
/*一键降落*/
void one_key_land()
{
    flag.auto_take_off_land = AUTO_LAND;
}

//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
_flight_state_st fs;

s16 flying_cnt, landing_cnt;

extern s32 ref_height_get;

float stop_baro_hpf;

/*降落检测*/

static s16 ld_delay_cnt;
void land_discriminat(s16 dT_ms)
{
    //	static s16 acc_delta,acc_old;

    //	acc_delta = imu_data.w_acc[Z]- acc_old;
    //	acc_old = imu_data.w_acc[Z];

    /*油门归一值小于0.1并且垂直方向加速度小于阈值  或者启动自动降落*/
    if ((fs.speed_set_h_norm[Z] < 0.1f && imu_data.w_acc[Z] < 200) || flag.auto_take_off_land == AUTO_LAND)
    {
        if (ld_delay_cnt > 0)
        {
            ld_delay_cnt -= dT_ms;
        }
    }
    else
    {
        ld_delay_cnt = 200;
    }

    /*意义是：如果向上推了油门，就需要等垂直方向加速度小于200cm/s2 保持200ms才开始检测*/
    if (ld_delay_cnt <= 0 && (flag.thr_low || flag.auto_take_off_land == AUTO_LAND))
    {
        /*油门最终输出量小于250并且没有在手动解锁上锁过程中，持续1秒，认为着陆，然后上锁*/
        if (mc.ct_val_thr < 250 && flag.fly_ready == 1 && flag.locking != 2) //ABS(wz_spe_f1.out <20 ) //还应当 与上速度条件，速度小于正20厘米每秒。
        {
            if (landing_cnt < 1500)
            {
                landing_cnt += dT_ms;
            }
            else
            {

                flying_cnt = 0;
                flag.taking_off = 0;
                ///////
                landing_cnt = 0;
                flag.fly_ready = 0;

                flag.flying = 0;
            }
        }
        else
        {
            landing_cnt = 0;
        }
    }
    else
    {
        landing_cnt = 0;
    }
}

/*飞行状态任务*/

void Flight_State_Task(u8 dT_ms, s16 *CH_N)
{
    s16 thr_deadzone;   //油门死区
    static float max_speed_lim; //最大速度限制
    /*设置油门摇杆量*/
    thr_deadzone = (flag.wifi_ch_en != 0) ? 0 : 50; //油门死区量100, 原来程序中设置为50
    fs.speed_set_h_norm[Z] = my_deadzone(CH_N[CH_THR], 0, thr_deadzone) * 0.0023f;  //竖直方向速度
    fs.speed_set_h_norm_lpf[Z] += 0.2f * (fs.speed_set_h_norm[Z] - fs.speed_set_h_norm_lpf[Z]); //竖直方向速度滤波之后

    /*推油门起飞*/
    if (flag.fly_ready) //如果满足起飞条件
    {
        //如果起飞的期望速度大于0.01，并且电机处于怠速状态
        if (fs.speed_set_h_norm[Z] > 0.01f && flag.motor_preparation == 1) // 0-1  
        {
            flag.taking_off = 1;    //起飞标志位置位
        }
    }
    //如果起飞标志位置位
    if (flag.taking_off)
    {

        if (flying_cnt < 1000) //延时1000ms
        {
            flying_cnt += dT_ms;
        }
        else
        {
            /*起飞后1秒，认为已经在飞行*/
            flag.flying = 1;
        }

        if (fs.speed_set_h_norm[Z] > 0)
        {
            /*设置上升速度*/
            fs.speed_set_h[Z] = fs.speed_set_h_norm_lpf[Z] * MAX_Z_SPEED_UP;
        }
        else
        {
            /*设置下降速度*/
            fs.speed_set_h[Z] = fs.speed_set_h_norm_lpf[Z] * MAX_Z_SPEED_DW;
        }
    }
    else
    {
        fs.speed_set_h[Z] = 0;
    }
    float speed_set_tmp[2];
    /*速度设定量，正负参考ANO坐标参考方向*/
    fs.speed_set_h_norm[X] = (my_deadzone(+CH_N[CH_PIT], 0, 50) * 0.0022f);
    fs.speed_set_h_norm[Y] = (my_deadzone(-CH_N[CH_ROL], 0, 50) * 0.0022f);

    LPF_1_(3.0f, dT_ms * 1e-3f, fs.speed_set_h_norm[X], fs.speed_set_h_norm_lpf[X]);
    LPF_1_(3.0f, dT_ms * 1e-3f, fs.speed_set_h_norm[Y], fs.speed_set_h_norm_lpf[Y]);

    //设置速度限制500cm/s
    max_speed_lim = MAX_SPEED;
    //如果光流打开
    if (switchs.of_flow_on)
    {
        max_speed_lim = 1.5f * wcz_hei_fus.out;
        max_speed_lim = LIMIT(max_speed_lim, 50, 150);
    }

    speed_set_tmp[X] = max_speed_lim * fs.speed_set_h_norm_lpf[X];
    speed_set_tmp[Y] = max_speed_lim * fs.speed_set_h_norm_lpf[Y];

    length_limit(&speed_set_tmp[X], &speed_set_tmp[Y], max_speed_lim, fs.speed_set_h_cms);

    fs.speed_set_h[X] = fs.speed_set_h_cms[X];
    fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];

    /*调用检测着陆的函数*/
    land_discriminat(dT_ms);

    /*倾斜过大上锁*/
    if (rolling_flag.rolling_step == ROLL_END)
    {
        if (imu_data.z_vec[Z] < 0.25f) //75度  ////////////////////////////////////////*************************** 倾斜过大上锁，慎用。
        {

            flag.fly_ready = 0;
        }
    }
    //////////////////////////////////////////////////////////
    /*校准中，复位重力方向*/
    if (sensor.gyr_CALIBRATE != 0 || sensor.acc_CALIBRATE != 0 || sensor.acc_z_auto_CALIBRATE)
    {
        imu_state.G_reset = 1;
    }

    /*复位重力方向时，认为传感器失效*/
    if (imu_state.G_reset == 1)
    {
        flag.sensor_ok = 0;
        WCZ_Data_Reset(); //复位高度数据融合
    }
    else if (imu_state.G_reset == 0)
    {
        if (flag.sensor_ok == 0)
        {
            flag.sensor_ok = 1;
            ANO_DT_SendString("IMU OK!", sizeof("IMU OK!"));
        }
    }

    /*飞行状态复位*/
    if (flag.fly_ready == 0)
    {
        flag.flying = 0;
        landing_cnt = 0;
        flag.taking_off = 0;
        flying_cnt = 0;

        //复位融合
        if (flag.taking_off == 0)
        {
            //			wxyz_fusion_reset();
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
static void LED_Switch()
{
    if ((LED.State != 0 && LED.State <= 115))
    {
        return;
    }

    if (flag.flight_mode == ATT_STAB)
    {
        if (flag.fly_ready)
        {
            LED.State = 131;
        }
        else
        {
            LED.State = 121;
        }
    }
    else if (flag.flight_mode == LOC_HOLD)
    {
        if (flag.fly_ready)
        {
            LED.State = 132;
        }
        else
        {
            LED.State = 122;
        }
    }
    else if (flag.flight_mode == RETURN_HOME)
    {
        if (flag.fly_ready)
        {
            LED.State = 133;
        }
        else
        {
            LED.State = 123;
        }
    }
}


//static u8 of_light_ok;
//static u16 of_light_delay;
void Swtich_State_Task(u8 dT_ms)
{
    //状态切换任务
    
    //气压测量：打开
    switchs.baro_on = 1;

    if(sens_hd_check.sonar_ok)//激光模块
	{
		if(us100_height<300.0f)
		{
			if(switchs.sonar_on == 0)
			{
				LED.State = 14 ;//切换指示触发1下（2闪蓝）
			}
			switchs.sonar_on = 1;
		}
		else
		{
			if(switchs.sonar_on )
			{
				LED.State = 24 ;//切换指示触发1下（2闪红）
			}
			switchs.sonar_on = 0;
		}
	}
	else
	{
		switchs.sonar_on = 0;
	}

    // if (sens_hd_check.of_ok) //光流模块
    // {
    //     // if (OF_LIGHT > 20 || flag.flying == 0) //光流亮度大于20或者在飞行之前，认为光流可用，判定可用延迟时间为1秒
    //     // {
    //     //     if (of_light_delay < 1000)
    //     //     {
    //     //         of_light_delay += dT_ms;
    //     //     }
    //     //     else
    //     //     {
    //     //         of_light_ok = 1;
    //     //     }
    //     // }
    //     // else
    //     // {
    //         of_light_delay = 0;
    //         of_light_ok = 0;
    //     // }

    //     if (OF_ALT < 1900 && flag.flight_mode == LOC_HOLD)
    //     {
    //         if (of_light_ok)
    //         {
    //             if (switchs.of_flow_on == 0)
    //             {
    //                 LED.State = 13; //切换指示触发1下（1闪蓝）
    //             }
    //             switchs.of_flow_on = 1;
    //         }
    //         else
    //         {
    //             if (switchs.of_flow_on)
    //             {
    //                 LED.State = 23; //切换指示触发1下（1闪红）
    //             }
    //             switchs.of_flow_on = 0;
    //         }

    //         if (switchs.of_tof_on == 0)
    //         {
    //             LED.State = 14; //切换指示触发1下（2闪蓝）
    //         }
    //         switchs.of_tof_on = 1;
    //     }
    //     else
    //     {
    //         if (switchs.of_tof_on)
    //         {
    //             LED.State = 24; //切换指示触发1下（2闪红）
    //         }
    //         switchs.of_tof_on = 0;

    //         switchs.of_flow_on = 0;
    //     }
    // }
    // else
    // {
    
    //光流，激光都关闭
        switchs.of_flow_on = switchs.of_tof_on = 0;
    //}

    // if (sens_hd_check.tof_ok) //激光模块
    // {
    //     if (tof_height_mm < 1900)
    //     {
    //         if (switchs.tof_on == 0)
    //         {
    //             LED.State = 14; //切换指示触发1下（2闪蓝）
    //         }
    //         switchs.tof_on = 1;
    //     }
    //     else
    //     {
    //         if (switchs.tof_on)
    //         {
    //             LED.State = 24; //切换指示触发1下（2闪红）
    //         }
    //         switchs.tof_on = 0;
    //     }
    // }
    // else
    // {
    //激光关闭
        switchs.tof_on = 0;
    //}
}

static void Speed_Mode_Switch()
{
    //	if( ubx_user_data.s_acc_cms > 60)// || ubx_user_data.svs_used < 6)
    //	{
    //		flag.speed_mode = 0;
    //	}
    //	else
    //	{
    //		flag.speed_mode = 1;
    //	}
}

u8 speed_mode_old = 255;
u8 flight_mode_old = 255;
void Flight_Mode_Set(u8 dT_ms)
{
    LED_Switch();
    Speed_Mode_Switch();

    if (speed_mode_old != flag.speed_mode) //状态改变
    {
        speed_mode_old = flag.speed_mode;

        if (flag.speed_mode == 1)
        {
            LED.State = 13;
        }
        else if (flag.speed_mode == 2)
        {
            LED.State = 14;
        }
        else
        {
            LED.State = 15;
        }

        //xy_speed_pid_init(flag.speed_mode);////////////
    }

    ///////////////////////////////////////////////////////

    if (CH_N[AUX1] < -200)
    {
        flag.flight_mode = ATT_STAB;
    }
    else if (CH_N[AUX1] < 200)
    {
        flag.flight_mode = LOC_HOLD;
    }
    else
    {
        flag.flight_mode = LOC_HOLD;
    }

    if (flight_mode_old != flag.flight_mode) //状态改变
    {
        flight_mode_old = flag.flight_mode;
    }
}
