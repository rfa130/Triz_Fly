#ifndef _ICM20602_H_
#define _ICM20602_H_

#include "include.h"
#include "APP/App_include.h"

//���ٶ�����
#define ICM20_ACCEL_FS_2G (0 << 3)
#define ICM20_ACCEL_FS_4G (1 << 3)
#define ICM20_ACCEL_FS_8G (2 << 3)
#define ICM20_ACCEL_FS_16G (3 << 3)
//���ٶ�����
#define ICM20_GYRO_FS_250 (0 << 3)
#define ICM20_GYRO_FS_500 (1 << 3)
#define ICM20_GYRO_FS_1000 (2 << 3)
#define ICM20_GYRO_FS_2000 (3 << 3)
//CONFIG DPF
#define DLPF_BW_250 0x00 //Rate=8k
#define DLPF_BW_176 0x01
#define DLPF_BW_92 0x02
#define DLPF_BW_41 0x03
#define DLPF_BW_20 0x04
#define DLPF_BW_10 0x05
#define DLPF_BW_5 0x06
#define DLPF_BW_328 0x06 //Rate=8k
//ACCEL_CONFIG2
#define ACCEL_AVER_4 (0x00 << 4) //Rate=8k
#define ACCEL_AVER_8 (0x01 << 4)
#define ACCEL_AVER_16 (0x02 << 4)
#define ACCEL_AVER_32 (0x03 << 4)
//ACCEL_DLPF
#define ACCEL_DLPF_BW_218 0x00
//#define ACCEL_DLPF_BW_218         	0x01
#define ACCEL_DLPF_BW_99 0x02
#define ACCEL_DLPF_BW_44 0x03
#define ACCEL_DLPF_BW_21 0x04
#define ACCEL_DLPF_BW_10 0x05
#define ACCEL_DLPF_BW_5 0x06
#define ACCEL_DLPF_BW_420 0x06

#define RANGE_PN2000_TO_RAD 0.001065f
#define RANGE_PN8G_TO_CMSS  0.2395f


typedef struct
{
    float center_pos_cm[VEC_XYZ];
    float gyro_rad[VEC_XYZ];
    float gyro_rad_old[VEC_XYZ];
    float gyro_rad_acc[VEC_XYZ];
    float linear_acc[VEC_XYZ];
} _center_pos_st;
extern _center_pos_st center_pos;

typedef struct
{
    u8 surface_CALIBRATE;
    float surface_vec[VEC_XYZ];
    float surface_unitvec[VEC_XYZ];

} _sensor_rotate_st;
extern _sensor_rotate_st sensor_rot;

typedef struct
{
    u8 acc_CALIBRATE;
    u8 gyr_CALIBRATE;
    u8 acc_z_auto_CALIBRATE;

    s16 Acc_Original[VEC_XYZ];
    s16 Gyro_Original[VEC_XYZ];

    s16 Acc[VEC_XYZ];
    s32 Acc_cmss[VEC_XYZ];
    float Gyro[VEC_XYZ];
    float Gyro_deg[VEC_XYZ];
    float Gyro_rad[VEC_XYZ];

    s16 Tempreature;
    float Tempreature_C;

} _sensor_st; //__attribute__((packed))

extern _sensor_st sensor;

uint8_t icm20602_set_accel_fullscale(uint8_t fs);
uint8_t icm20602_set_gyro_fullscale(uint8_t fs);
void Drv_Icm20602CSPin_Init(void);
uint8_t Drv_Icm20602Reg_Init(void);
void Drv_Icm20602_Read(void);
void Sensor_Data_Prepare(u8 dT_ms);
void Center_Pos_Set(void);

#endif
