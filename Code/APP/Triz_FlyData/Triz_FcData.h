/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ANO_FCDATA_H
#define __ANO_FCDATA_H
/* Includes ------------------------------------------------------------------*/
#include "include.h"
/* Exported types ------------------------------------------------------------*/
#define TRUE 1
#define FALSE 0

enum
{
    AUTO_TAKE_OFF_NULL = 0, //未起飞
    AUTO_TAKE_OFF = 1,      //自动起飞
    AUTO_TAKE_OFF_FINISH,   //起飞完成
    AUTO_LAND,              //自动降落
};

enum pwminmode_e //遥控信号输入类型
{
    PWM = 0, //pwm模式
    PPM,     //PPM模式
    SBUS,    //SBUS模式， 在此程序中选用此模式
};

enum
{
    A_X = 0, //加速度计 x轴
    A_Y,
    A_Z,
    G_X, //陀螺仪三轴
    G_Y,
    G_Z,
    TEM,       //温度
    MPU_ITEMS, //运动传感器类型
};

enum //遥控通道数据枚举
{
    CH_PIT = 0,
    CH_THR,
    CH_ROL,
    CH_YAW,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    CH_NUM,
};

enum //电机枚举
{
    m1 = 0,
    m2,
    m3,
    m4,
    m5,
    m6,
    m7,
    m8,

};

enum
{
    MPU_6050_0 = 0,
    MPU_6050_1,

};

enum
{
    X = 0,
    Y = 1,
    Z = 2,
    VEC_XYZ, //XYZ 表示向量
};

enum
{
    ROL = 0,
    PIT = 1,
    YAW = 2,
    VEC_RPY, // RPY方式表示的向量
};

enum
{
    KP = 0,
    KI = 1,
    KD = 2,
    PID, //PID参数向量
};

enum _power_alarm
{

    HIGH_POWER = 0,
    HALF_POWER,
    LOW_POWER,
    LOWEST_POWER, //电量监测向量

};

enum _flight_mode
{
    ATT_STAB = 0, //Attitude stabilization 姿态稳定
    LOC_HOLD,     //定点
    RETURN_HOME,  //返回

};

//thr_mode
enum
{
    THR_MANUAL = 0, //手动油门
    THR_AUTO,       //自动油门

};

typedef struct
{
    u8 first_f;
    float acc_offset[VEC_XYZ];
    float gyro_offset[VEC_XYZ];

    float surface_vec[VEC_XYZ];

    float mag_offset[VEC_XYZ];
    float mag_gain[VEC_XYZ];

} _save_st;
extern _save_st save;

typedef struct
{
    //基本状态/传感器
    u8 start_ok;
    u8 sensor_ok;
    u8 motionless;
    u8 power_state;
    u8 wifi_ch_en;
    u8 rc_loss;
    u8 gps_ok;
    u8 gps_signal_bad;

    //控制状态
    u8 manual_locked;
    u8 unlock_en;
    u8 fly_ready; //unlocked
    u8 thr_low;
    u8 locking;
    u8 taking_off; //起飞
    u8 set_yaw;
    u8 ct_loc_hold;
    u8 ct_alt_hold;

    //飞行状态
    u8 flying;             //正在飞行标志位
    u8 auto_take_off_land; //自动起飞
    u8 home_location_ok;   //定点完成
    u8 speed_mode;         //速度模式
    u8 thr_mode;           //油门模式
    u8 flight_mode;        //飞行模式
    u8 gps_mode_en;        //gps模式使能位
    u8 motor_preparation;  //电机准备
    u8 locked_rotor;       //电机锁定

} _flag;
extern _flag flag;

typedef struct
{
    u8 sonar_on; //超声波打开
    u8 tof_on;
    u8 of_flow_on; //光流打开
    u8 of_tof_on;
    u8 baro_on; //气压计打开
    u8 gps_on;  //gps打开

} _switch_st;
extern _switch_st switchs;

typedef struct
{
    u8 gyro_ok;  //陀螺仪 ok
    u8 acc_ok;   //加速度计ok
    u8 mag_ok;   //磁力计ok
    u8 baro_ok;  //气压计ok
    u8 gps_ok;   //gps ok
    u8 sonar_ok; //超声波ok
    u8 tof_ok;   //tof
    u8 of_ok;

} _sensor_hd_check_st; //Hardware 传感器 硬件 检查 结构体
extern _sensor_hd_check_st sens_hd_check;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void data_save(void);
void Para_Data_Init(void);

#endif
