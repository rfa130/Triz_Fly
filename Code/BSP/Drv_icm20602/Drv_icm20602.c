//-----------------------------------------------------------------------------
// Drv_icm20602.c
//
//  Created on	: 2018-5-08
//      Author	: Divenire
//		version	: V1.0
//		brief	:
//-----------------------------------------------------------------------------
// Attention:
//
//
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "Common/Com_include.h"
#include "BSP/Drv_include.h"


//========ICM20602????????========================
/********************************************
*??��?????��??????????0??????
*Register 26  CONFIG				= 0x80
*Register 107 Power Management 1 	= 0x41
*Register 117 WHO_AM_I 				= 0x12
*********************************************/
//????????????
#define ICM20_XG_OFFS_TC_H 0x04
#define ICM20_XG_OFFS_TC_L 0x05
#define ICM20_YG_OFFS_TC_H 0x07
#define ICM20_YG_OFFS_TC_L 0x08
#define ICM20_ZG_OFFS_TC_H 0x0A
#define ICM20_ZG_OFFS_TC_L 0x0B
//???????????(????????????????????????????????
#define ICM20_SELF_TEST_X_ACCEL 0x0D
#define ICM20_SELF_TEST_Y_ACCEL 0x0E
#define ICM20_SELF_TEST_Z_ACCEL 0x0F
//???????????
#define ICM20_XG_OFFS_USRH 0x13
#define ICM20_XG_OFFS_USRL 0x14
#define ICM20_YG_OFFS_USRH 0x15
#define ICM20_YG_OFFS_USRL 0x16
#define ICM20_ZG_OFFS_USRH 0x17
#define ICM20_ZG_OFFS_USRL 0x18

#define ICM20_SMPLRT_DIV 0x19
#define ICM20_CONFIG 0x1A
#define ICM20_GYRO_CONFIG 0x1B
#define ICM20_ACCEL_CONFIG 0x1C
#define ICM20_ACCEL_CONFIG2 0x1D
#define ICM20_LP_MODE_CFG 0x1E

//??????????????
#define ICM20_ACCEL_WOM_X_THR 0x20
#define ICM20_ACCEL_WOM_Y_THR 0x21
#define ICM20_ACCEL_WOM_Z_THR 0x22

#define ICM20_FIFO_EN 0x23
#define ICM20_FSYNC_INT 0x36
#define ICM20_INT_PIN_CFG 0x37
//#define	ICM20_INT_ENABLE				0x38
#define ICM20_FIFO_WM_INT_STATUS 0x39
#define ICM20_INT_STATUS 0x3A

//????????
#define ICM20_ACCEL_XOUT_H 0x3B
#define ICM20_ACCEL_XOUT_L 0x3C
#define ICM20_ACCEL_YOUT_H 0x3D
#define ICM20_ACCEL_YOUT_L 0x3E
#define ICM20_ACCEL_ZOUT_H 0x3F
#define ICM20_ACCEL_ZOUT_L 0x40
//??????
#define ICM20_TEMP_OUT_H 0x41
#define ICM20_TEMP_OUT_L 0x42
//????????
#define ICM20_GYRO_XOUT_H 0x43
#define ICM20_GYRO_XOUT_L 0x44
#define ICM20_GYRO_YOUT_H 0x45
#define ICM20_GYRO_YOUT_L 0x46
#define ICM20_GYRO_ZOUT_H 0x47
#define ICM20_GYRO_ZOUT_L 0x48
//????????????
#define ICM20_SELF_TEST_X_GYRO 0x50
#define ICM20_SELF_TEST_Y_GYRO 0x51
#define ICM20_SELF_TEST_Z_GYRO 0x52

#define ICM20_FIFO_WM_TH1 0x60
#define ICM20_FIFO_WM_TH2 0x61
#define ICM20_SIGNAL_PATH_RESET 0x68
#define ICM20_ACCEL_INTEL_CTRL 0x69
#define ICM20_USER_CTRL 0x6A
//???????
#define ICM20_PWR_MGMT_1 0x6B
#define ICM20_PWR_MGMT_2 0x6C

#define ICM20_I2C_IF 0x70
#define ICM20_FIFO_COUNTH 0x72
#define ICM20_FIFO_COUNTL 0x73
#define ICM20_FIFO_R_W 0x74

#define ICM20_WHO_AM_I 0x75
//??????????
#define ICM20_XA_OFFSET_H 0x77
#define ICM20_XA_OFFSET_L 0x78
#define ICM20_YA_OFFSET_H 0x7A
#define ICM20_YA_OFFSET_L 0x7B
#define ICM20_ZA_OFFSET_H 0x7D
#define ICM20_ZA_OFFSET_L 0x7E
//????????????
#define NUM_MOVING_AVERAGE 10
//===========================================================

static float _accel_scale;
 static float _gyro_scale;

#define ICM20602_ADDRESS 0xD2

#define GRAVITY_MSS 9.80665f
#define _ACCEL_SCALE_1G (GRAVITY_MSS / 4096.0f) //????8G
#define _DEG_TO_RAD 0.0174532f                  //???????

uint8_t SPI2_write_reg(uint8_t reg_addr, uint8_t reg_val)
{
    Drv_SPI2_RW(reg_addr & 0x7f);
    Drv_SPI2_RW(reg_val);
    return 0;
}

uint8_t SPI2_read_reg(uint8_t reg_addr)
{
    Drv_SPI2_RW(reg_addr | 0x80);
    return Drv_SPI2_RW(0xff);
}

uint8_t SPI2_read_reg_buffer(uint8_t reg_addr, void *buffer, uint16_t len)
{
    uint8_t *p = buffer;
    uint16_t i;
    Drv_SPI2_RW(reg_addr | 0x80);
    for (i = 0; i < len; i++)
    {
        *p++ = Drv_SPI2_RW(0xff);
    }
    return 0;
}

uint8_t icm20602_write_reg(uint8_t reg, uint8_t val)
{
    GPIO_ResetBits(ICM20602_CS_GPIO, ICM20602_CS_PIN);
    SPI2_write_reg(reg, val);
    GPIO_SetBits(ICM20602_CS_GPIO, ICM20602_CS_PIN);
    return 0;
}

uint8_t icm20602_read_reg(uint8_t reg)
{
    uint8_t res;
    GPIO_ResetBits(ICM20602_CS_GPIO, ICM20602_CS_PIN);
    res = SPI2_read_reg(reg);
    GPIO_SetBits(ICM20602_CS_GPIO, ICM20602_CS_PIN);

    return res;
}

uint8_t icm20602_read_buffer(uint8_t reg, uint8_t len, uint8_t *buffer)
{

    GPIO_ResetBits(ICM20602_CS_GPIO, ICM20602_CS_PIN);
    SPI2_read_reg_buffer(reg, buffer, len);
    GPIO_SetBits(ICM20602_CS_GPIO, ICM20602_CS_PIN);
    return 0;
}

void Drv_Icm20602CSPin_Init(void)
{
    /*?????????*/
    SysCtlPeripheralEnable(ICM20602_CS_RCC);

    /*?????PP???*/
    GPIOPinTypeGPIOOutput(ICM20602_CS_GPIO, ICM20602_CS_PIN);

    /*????CS???????idel??*/
    GPIO_SetBits(ICM20602_CS_GPIO, ICM20602_CS_PIN);
}

s16 roll_gz_comp;
float wh_matrix[VEC_XYZ][VEC_XYZ] =
    {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}

};
void Center_Pos_Set()
{
    center_pos.center_pos_cm[X] = Triz_Parame.set.center_pos_cm[X]; //+0.0f;
    center_pos.center_pos_cm[Y] = Triz_Parame.set.center_pos_cm[Y]; //-0.0f;
    center_pos.center_pos_cm[Z] = Triz_Parame.set.center_pos_cm[Z]; //+0.0f;
}

uint8_t Drv_Icm20602Reg_Init()
{

    uint8_t id;
    icm20602_write_reg(ICM20_PWR_MGMT_1, 0x80); //??��????��??��0x41,???????
    delay_ms(10);
    icm20602_write_reg(ICM20_PWR_MGMT_1, 0x01); //????????????????
    delay_ms(10);

    id = icm20602_read_reg(ICM20_WHO_AM_I); //???ID
    // printf("icm_20602 id=0x%x\r\n", id);

    if (id != 0x12)
    {
        // printf("icm_20602 id error !!!\r\n");
        return 0;
    }

    // printf("icm20602 init pass\r\n\r\n");
    icm20602_write_reg(ICM20_SIGNAL_PATH_RESET, 0x03);
    delay_ms(10);
    icm20602_write_reg(ICM20_USER_CTRL, 0X01);
    delay_ms(10);
    icm20602_write_reg(ICM20_I2C_IF, 0x40); //dmp
    delay_ms(10);

    icm20602_write_reg(ICM20_PWR_MGMT_2, 0x00);
    delay_ms(10);
    icm20602_write_reg(ICM20_SMPLRT_DIV, 0); //?????=?0+1?????????????????????????
    delay_ms(10);
    icm20602_write_reg(ICM20_CONFIG, DLPF_BW_20); //GYRO??????????
    delay_ms(10);
    icm20602_write_reg(ICM20_GYRO_CONFIG, (3 << 3));
    delay_ms(10);
    icm20602_write_reg(ICM20_ACCEL_CONFIG, (2 << 3));
    delay_ms(10);
    icm20602_write_reg(ICM20_ACCEL_CONFIG2, ACCEL_AVER_4 | ACCEL_DLPF_BW_21); //ACCEL??????????
    delay_ms(10);

    //????????
    icm20602_set_accel_fullscale(ICM20_ACCEL_FS_8G);
    delay_ms(10);
    icm20602_set_gyro_fullscale(ICM20_GYRO_FS_2000);
    delay_ms(10);

    icm20602_write_reg(ICM20_LP_MODE_CFG, 0x00); //???????
    delay_ms(10);
    icm20602_write_reg(ICM20_FIFO_EN, 0x00); //???FIFO
    delay_ms(10);
    /*???????????????????????*/
    Center_Pos_Set();
    sensor.acc_z_auto_CALIBRATE = 1; //??????????Z??
    sensor.gyr_CALIBRATE = 2;        //???????��???????
    return 1;
}

//ICM20_ACCEL_FS_2G
//ICM20_ACCEL_FS_4G
//ICM20_ACCEL_FS_8G
//ICM20_ACCEL_FS_16G
uint8_t icm20602_set_accel_fullscale(uint8_t fs)
{
    switch (fs)
    {
    case ICM20_ACCEL_FS_2G:
        _accel_scale = 1.0f / 16348.0f;
        break;
    case ICM20_ACCEL_FS_4G:
        _accel_scale = 1.0f / 8192.0f;
        break;
    case ICM20_ACCEL_FS_8G:
        _accel_scale = 1.0f / 4096.0f;
        break;
    case ICM20_ACCEL_FS_16G:
        _accel_scale = 1.0f / 2048.0f;
        break;
    default:
        fs = ICM20_ACCEL_FS_8G;
        _accel_scale = 1.0f / 4096.0f;
        break;
    }
    _accel_scale *= GRAVITY_MSS;
    return icm20602_write_reg(ICM20_ACCEL_CONFIG, fs);
}

//ICM20_GYRO_FS_250
//ICM20_GYRO_FS_500
//ICM20_GYRO_FS_1000
//ICM20_GYRO_FS_2000
uint8_t icm20602_set_gyro_fullscale(uint8_t fs)
{
    switch (fs)
    {
    case ICM20_GYRO_FS_250:
        _gyro_scale = 1.0f / 131.068f; //32767/250
        break;
    case ICM20_GYRO_FS_500:
        _gyro_scale = 1.0f / 65.534f;
        break;
    case ICM20_GYRO_FS_1000:
        _gyro_scale = 1.0f / 32.767f;
        break;
    case ICM20_GYRO_FS_2000:
        _gyro_scale = 1.0f / 16.3835f;
        break;
    default:
        fs = ICM20_GYRO_FS_2000;
        _gyro_scale = 1.0f / 16.3835f;
        break;
    }
    return icm20602_write_reg(ICM20_GYRO_CONFIG, fs);
}

u8 icm_buffer[14];
_center_pos_st center_pos;
_sensor_st sensor;

void Drv_Icm20602_Read()
{
    icm20602_read_buffer(ICM20_ACCEL_XOUT_H, 14, icm_buffer);
}

s32 sensor_val[6];
s32 sensor_val_rot[6];
s32 sensor_val_ref[6];
//float sensor_val_lpf[2][6];

s32 sum_temp[7] = {0, 0, 0, 0, 0, 0, 0};
s32 acc_auto_sum_temp[3];
s16 acc_z_auto[4];

u16 acc_sum_cnt = 0, gyro_sum_cnt = 0, acc_z_auto_cnt;

s16 g_old[VEC_XYZ];
float g_d_sum[VEC_XYZ] = {500, 500, 500};

void mpu_auto_az()
{
    if (sensor.acc_z_auto_CALIBRATE)
    {
        acc_z_auto_cnt++;

        acc_auto_sum_temp[0] += sensor_val_ref[A_X];
        acc_auto_sum_temp[1] += sensor_val_ref[A_Y];
        acc_auto_sum_temp[2] += sensor_val_rot[A_Z];

        if (acc_z_auto_cnt >= OFFSET_AV_NUM)
        {
            sensor.acc_z_auto_CALIBRATE = 0;
            acc_z_auto_cnt = 0;

            for (u8 i = 0; i < 3; i++)
            {
                acc_z_auto[i] = acc_auto_sum_temp[i] / OFFSET_AV_NUM;

                acc_auto_sum_temp[i] = 0;
            }

            acc_z_auto[3] = my_sqrt(4096 * 4096 - (my_pow(acc_z_auto[0]) + my_pow(acc_z_auto[1])));

            save.acc_offset[Z] = acc_z_auto[2] - acc_z_auto[3];
        }
    }
}

void motionless_check(u8 dT_ms)
{
    u8 t = 0;

    for (u8 i = 0; i < 3; i++)
    {
        g_d_sum[i] += 3 * ABS(sensor.Gyro_Original[i] - g_old[i]);

        g_d_sum[i] -= dT_ms;

        g_d_sum[i] = LIMIT(g_d_sum[i], 0, 200);

        if (g_d_sum[i] > 10)
        {
            t++;
        }

        g_old[i] = sensor.Gyro_Original[i];
    }

    if (t >= 2)
    {
        flag.motionless = 0;
    }
    else
    {
        flag.motionless = 1;
    }
}

void ICM_Data_Offset()
{
    static u8 off_cnt;

    if (sensor.gyr_CALIBRATE || sensor.acc_CALIBRATE || sensor.acc_z_auto_CALIBRATE)
    {

        ///////////////////复位校准值///////////////////////////
        if (flag.motionless == 0 || sensor_val[A_Z] < 1000)
        {
            gyro_sum_cnt = 0;
            acc_sum_cnt = 0;
            acc_z_auto_cnt = 0;

            for (u8 j = 0; j < 3; j++)
            {
                acc_auto_sum_temp[j] = sum_temp[G_X + j] = sum_temp[A_X + j] = 0;
            }
            sum_temp[TEM] = 0;
        }

        ///////////////////////////////////////////////////////////
        off_cnt++;
        if (off_cnt >= 10)
        {
            off_cnt = 0;

            if (sensor.gyr_CALIBRATE)
            {
                LED.State = 2;
                gyro_sum_cnt++;

                for (u8 i = 0; i < 3; i++)
                {
                    sum_temp[G_X + i] += sensor.Gyro_Original[i];
                }
                if (gyro_sum_cnt >= OFFSET_AV_NUM)
                {
                    for (u8 i = 0; i < 3; i++)
                    {
                        save.gyro_offset[i] = (float)sum_temp[G_X + i] / OFFSET_AV_NUM;

                        sum_temp[G_X + i] = 0;
                    }
                    gyro_sum_cnt = 0;
                    if (sensor.gyr_CALIBRATE == 1)
                    {
                        if (sensor.acc_CALIBRATE == 0)
                        {
                            data_save();
                        }
                    }
                    sensor.gyr_CALIBRATE = 0;
                    ANO_DT_SendString("GYR init OK!", sizeof("GYR init OK!"));
                }
            }

            if (sensor.acc_CALIBRATE == 1)
            {
                LED.State = 3;
                acc_sum_cnt++;

                sum_temp[A_X] += sensor_val_rot[A_X];
                sum_temp[A_Y] += sensor_val_rot[A_Y];
                sum_temp[A_Z] += sensor_val_rot[A_Z] - G_1G; // - 65535/16;   // +-8G
                sum_temp[TEM] += sensor.Tempreature;

                if (acc_sum_cnt >= OFFSET_AV_NUM)
                {
                    for (u8 i = 0; i < 3; i++)
                    {
                        save.acc_offset[i] = sum_temp[A_X + i] / OFFSET_AV_NUM;

                        sum_temp[A_X + i] = 0;
                    }

                    acc_sum_cnt = 0;
                    sensor.acc_CALIBRATE = 0;
                    ANO_DT_SendString("ACC init OK!", sizeof("ACC init OK!"));
                    data_save();
                }
            }
        }
    }
}

static float gyr_f1[VEC_XYZ], acc_f1[VEC_XYZ];

void Sensor_Data_Prepare(u8 dT_ms)
{
    float hz = 0;
    if (dT_ms != 0)
        hz = 1000 / dT_ms;

    /*静止检测*/
    motionless_check(dT_ms);

    ICM_Data_Offset(); //校准函数

    /*获取buffer原始数据*/
    sensor.Acc_Original[X] = (s16)((((u16)icm_buffer[0]) << 8) | icm_buffer[1]); //>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
    sensor.Acc_Original[Y] = (s16)((((u16)icm_buffer[2]) << 8) | icm_buffer[3]);     //>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
    sensor.Acc_Original[Z] = (s16)((((u16)icm_buffer[4]) << 8) | icm_buffer[5]);     //>>1;// + 4 *sensor.Tempreature_C;// + 7 *sensor.Tempreature_C;

    sensor.Gyro_Original[X] = (s16)((((u16)icm_buffer[8]) << 8) | icm_buffer[9]);
    sensor.Gyro_Original[Y] = (s16)((((u16)icm_buffer[10]) << 8) | icm_buffer[11]);
    sensor.Gyro_Original[Z] = (s16)((((u16)icm_buffer[12]) << 8) | icm_buffer[13]);

    sensor.Tempreature = ((((int16_t)icm_buffer[6]) << 8) | icm_buffer[7]); //tempreature
    /*获取icm的温度*/
    sensor.Tempreature_C = sensor.Tempreature / 326.8f + 25;

    /*得出校准后的数据*/
    for (u8 i = 0; i < 3; i++)
    {

        sensor_val[A_X + i] = sensor.Acc_Original[i];

        sensor_val[G_X + i] = sensor.Gyro_Original[i] - save.gyro_offset[i];
        //sensor_val[G_X+i] = (sensor_val[G_X+i] >>2) <<2;
    }

    /*赋值*/
    for (u8 i = 0; i < 6; i++)
    {
        sensor_val_rot[i] = sensor_val[i];
    }

    /*数据坐标旋转90度*/
    sensor_val_ref[G_X] = sensor_val_rot[G_Y];
    sensor_val_ref[G_Y] = -sensor_val_rot[G_X];
    sensor_val_ref[G_Z] = sensor_val_rot[G_Z];

    sensor_val_ref[A_X] = (sensor_val_rot[A_Y] - save.acc_offset[Y]);
    sensor_val_ref[A_Y] = -(sensor_val_rot[A_X] - save.acc_offset[X]);
    sensor_val_ref[A_Z] = (sensor_val_rot[A_Z] - save.acc_offset[Z]);

    /*单独校准z轴模长*/
    mpu_auto_az();

    //======================================================================

    /*软件滤波*/
    for (u8 i = 0; i < 3; i++)
    {
        //0.24f??1ms ??50hz???; 0.15f,1ms,28hz; 0.1f,1ms,18hz
        gyr_f1[X + i] += 0.12f * (sensor_val_ref[G_X + i] - sensor.Gyro[X + i]);
        acc_f1[X + i] += 0.12f * (sensor_val_ref[A_X + i] - sensor.Acc[X + i]);
    }

    /*旋转加速度补偿*/
    //======================================================================

    for (u8 i = 0; i < 3; i++)
    {
        center_pos.gyro_rad_old[i] = center_pos.gyro_rad[i];
        center_pos.gyro_rad[i] = gyr_f1[X + i] * RANGE_PN2000_TO_RAD; //0.001065f;
        center_pos.gyro_rad_acc[i] = hz * (center_pos.gyro_rad[i] - center_pos.gyro_rad_old[i]);
    }

    center_pos.linear_acc[X] = +center_pos.gyro_rad_acc[Z] * center_pos.center_pos_cm[Y] - center_pos.gyro_rad_acc[Y] * center_pos.center_pos_cm[Z];
    center_pos.linear_acc[Y] = -center_pos.gyro_rad_acc[Z] * center_pos.center_pos_cm[X] + center_pos.gyro_rad_acc[X] * center_pos.center_pos_cm[Z];
    center_pos.linear_acc[Z] = +center_pos.gyro_rad_acc[Y] * center_pos.center_pos_cm[X] - center_pos.gyro_rad_acc[X] * center_pos.center_pos_cm[Y];

    //======================================================================
    /*赋值*/
    for (u8 i = 0; i < 3; i++)
    {

        sensor.Gyro[X + i] = gyr_f1[i]; //sensor_val_ref[G_X + i];

        sensor.Acc[X + i] = acc_f1[i] - center_pos.linear_acc[i] / RANGE_PN8G_TO_CMSS; //sensor_val_ref[A_X+i];//
    }

    /*转换单位*/
    for (u8 i = 0; i < 3; i++)
    {
        /*陀螺仪转换到度每秒，量程+-2000度*/
        sensor.Gyro_deg[i] = sensor.Gyro[i] * 0.06103f; //  /65535 * 4000; +-2000?? 0.061

        /*???????????????????????+-2000??*/
        sensor.Gyro_rad[i] = sensor.Gyro[i] * RANGE_PN2000_TO_RAD; //  0.001065264436f //???? 0.0010652f

        /*?????????????????????????+-8G*/
        sensor.Acc_cmss[i] = (sensor.Acc[i] * RANGE_PN8G_TO_CMSS); //   /65535 * 16*981; +-8G
    }
}
