
#include "APP/App_include.h"
#include "BSP/Drv_include.h"
#include "Common/Com_include.h"

u16 test_time_cnt;
//传感器数据获取函数
void Fc_Sensor_Get() //1ms
{
    static u8 cnt;
    if (flag.start_ok)
    {
        /*读取陀螺仪加速度计数据*/
        Drv_Icm20602_Read();

        cnt++;
        cnt %= 20;
        if (cnt == 0)
        {
            /*读取电子罗盘磁力计数据*/
            Drv_AK8975_Read();
            /*读取气压计数据*/
            baro_height = (s32)Drv_Spl0601_Read();
        }
    }
    test_time_cnt++;
}

extern s32 sensor_val_ref[];

static u8 reset_imu_f;
void IMU_Update_Task(u8 dT_ms)
{

    ////////////////////////////////////////////////////////////////////////
    /*如果准备飞行，复位重力复位标记和磁力计复位标记*/
    if (flag.fly_ready)
    {
        imu_state.G_reset = imu_state.M_reset = 0;
        reset_imu_f = 0;
    }
    else
    {
        if (flag.motionless == 0)
        {
            //						imu_state.G_reset = 1;//自动复位
            //sensor.gyr_CALIBRATE = 2;
        }

        if (reset_imu_f == 0) //&& flag.motionless == 1)
        {
            imu_state.G_reset = 1;    //自动复位
            sensor.gyr_CALIBRATE = 2; //校准陀螺仪，不保存
            reset_imu_f = 1;          //已经置位复位标记
        }
    }

    if (0)
    {
        imu_state.gkp = 0.0f;
        imu_state.gki = 0.0f;
    }
    else
    {
        if (0)
        {
            imu_state.gkp = 0.2f;
        }
        else
        {
            /*设置重力互补融合修正kp系数*/
            imu_state.gkp = 0.3f; //0.4f;
        }

        /*设置重力互补融合修正ki系数*/
        imu_state.gki = 0.002f;

        /*设置罗盘互补融合修正ki系数*/
        imu_state.mkp = 0.2f;
    }

    imu_state.M_fix_en = sens_hd_check.mag_ok; //磁力计修正使能

    //				imu_state.obs_en = 1;
    //				imu_data.obs_acc_w[X] = flow_decoupling.w_ref_acc[X];
    //				imu_data.obs_acc_w[Y] = flow_decoupling.w_ref_acc[Y];

    /*姿态计算，更新，融合*/
    IMU_update(dT_ms * 1e-3f, &imu_state, sensor.Gyro_rad, sensor.Acc_cmss, mag.val, &imu_data); //x3_dT_1[2] * 0.000001f
    //////////////////////////////////////////////////////////////////////
}

static s16 mag_val[3];
void Mag_Update_Task(u8 dT_ms)
{
    Mag_Get(mag_val);

    Mag_Data_Deal_Task(dT_ms, mag_val, imu_data.z_vec[Z], sensor.Gyro_deg[X], sensor.Gyro_deg[Z]);
}

s32 baro_height, baro_h_offset, ref_height_get_1, ref_height_get_2, ref_height_used;
s32 baro2son_offset, son2baro_offset;

float baro_fix1, baro_fix2, baro_fix;

static u8 wcz_f_pause;
float wcz_acc_use;

void WCZ_Acc_Get_Task() //最小周期
{
    wcz_acc_use += 0.2f * (imu_data.w_acc[Z] - wcz_acc_use);
}

//void Baro_Get_Task()
//{
////			ref_height_get += LIMIT((s32)user_spl0601_get() - ref_height_get,-20,20 );
////	baro_height =(s32)user_spl0601_get();
//}

u16 ref_son_height;
static u8 baro_offset_ok;   //气压计补偿获取标志位
static u8 son_offset_ok;    //超声波补偿获取标志位

void WCZ_Fus_Task(u8 dT_ms)
{

    if (flag.taking_off)
    {
        baro_offset_ok = 2;
    }
    else
    {
        if (baro_offset_ok == 2)
        {
            baro_offset_ok = 0;
        }
    }

    if (baro_offset_ok >= 1) //(flag.taking_off)
    {
        ref_height_get_1 = baro_height - baro_h_offset + baro_fix + son2baro_offset; //气压计相对高度，切换点跟随TOF
                                                                                     //baro_offset_ok = 0;
    }
    else
    {
        if (baro_offset_ok == 0)
        {
            baro_h_offset = baro_height;
            if (flag.sensor_ok)
            {
                baro_offset_ok = 1;
            }
        }
    }

    if ((flag.flying == 0) && flag.auto_take_off_land == AUTO_TAKE_OFF)
    {
        wcz_f_pause = 1;

        baro_fix = 0;
    }
    else
    {
        wcz_f_pause = 0;

        if (flag.taking_off == 0)
        {
            baro_fix1 = 0;
            baro_fix2 = 0;
        }
        baro_fix2 = -BARO_FIX;

        baro_fix = baro_fix1 + baro_fix2 - BARO_FIX; //+ baro_fix3;
    }

    if (sens_hd_check.sonar_ok && baro_offset_ok) //超声波硬件正常，且气压计记录相对值以后
    {
        if (switchs.sonar_on ) //超声波数据有效
        {
            ref_son_height=(uint16_t)us100_height;//获取超声波高度数据

            
            if(son_offset_ok==1)
            {
                ref_height_get_2 = ref_son_height + baro2son_offset;    //超声波参考高度，切换点随气压计
            
                ref_height_used = ref_height_get_2;
                son2baro_offset += 0.5f *((ref_height_get_2 - ref_height_get_1) -son2baro_offset); //记录气压计切换点，对气压计数据进行滤波，
            }
            else
            {
                baro2son_offset =  ref_height_get_1 -ref_son_height;    //记录超声波切换点
                son_offset_ok=1;
            }
        }
         else
        {
            son_offset_ok=0;
            ref_height_used=ref_height_get_1;
        }
    }
    else
	{
		ref_height_used = ref_height_get_1;
	}
    
    WCZ_Data_Calc(dT_ms, wcz_f_pause, (s32)wcz_acc_use, (s32)(ref_height_used));
}
