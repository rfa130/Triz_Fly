//-----------------------------------------------------------------------------
// Drv_ak8975.c
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
#include "include.h"
#include "BSP\Drv_include.h"

void Drv_AK8975CSPin_Init(void)
{
    /*是能端口时钟*/
    SysCtlPeripheralEnable(AK8975_CS_RCC);

    /*配置为PP输出*/
    GPIOPinTypeGPIOOutput(AK8975_CS_GPIO, AK8975_CS_PIN);

    /*拉高CS引脚作为idel状态*/
    GPIO_SetBits(AK8975_CS_GPIO, AK8975_CS_PIN);
}

static void ak8975_enable(uint8_t ena)
{
    if (ena)
        GPIO_ResetBits(AK8975_CS_GPIO, AK8975_CS_PIN);
    else
        GPIO_SetBits(AK8975_CS_GPIO, AK8975_CS_PIN);
}

static void ak8975_Trig(void)
{
    ak8975_enable(1);
    Drv_SPI2_RW(AK8975_CNTL_REG);
    Drv_SPI2_RW(0x01);
    ak8975_enable(0);
}

static uint8_t ak8975_buf[6];
void Drv_AK8975_Read(void)
{

    ak8975_enable(1);
    Drv_SPI2_RW(AK8975_HXL_REG | 0x80);
    for (uint8_t i = 0; i < 6; i++)
        ak8975_buf[i] = Drv_SPI2_RW(0xff);
    ak8975_enable(0);
    ak8975_Trig();
}

void Mag_Get(int16_t mag_val[3])
{
    int t[3];

    t[0] = ((((int16_t)ak8975_buf[1]) << 8) | ak8975_buf[0]);
    t[1] = ((((int16_t)ak8975_buf[3]) << 8) | ak8975_buf[2]);
    t[2] = ((((int16_t)ak8975_buf[5]) << 8) | ak8975_buf[4]);

    mag_val[0] = t[0] - 118;
    mag_val[1] = 1.088 * t[1] - 69;
    mag_val[2] = t[2];
}
