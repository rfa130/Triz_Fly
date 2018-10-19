#ifndef _DRV_INCLUDE_H_
#define _DRV_INCLUDE_H_

#include "BSP/Drv_led/Drv_led.h"
#include "BSP/Drv_ak8975/Drv_ak8975.h"
#include "BSP/Drv_camara/Drv_camara.h"
#include "BSP/Drv_capture/Drv_RC.h"
#include "BSP/Drv_icm20602/Drv_icm20602.h"
#include "BSP/Drv_spi/Drv_spi.h"
#include "BSP/Drv_time/Drv_time.h"
#include "BSP/Drv_led/Drv_led.h"
#include "BSP/Drv_pwm/Drv_pwm.h"
#include "BSP/Drv_spl06/Drv_spl06.h"
#include "BSP/Drv_us100/Drv_us100.h"
#include "BSP/Drv_usart/Drv_uart_User_Setting.h"
#include "BSP/Drv_usart/Drv_uart.h"
#include "BSP/Drv_adc/Drv_adc.h"

#define ICM20602_CS_RCC		SYSCTL_PERIPH_GPIOC
#define ICM20602_CS_GPIO	GPIO_PORTC_BASE
#define ICM20602_CS_PIN		GPIO_PIN_6

#define SPL06_CS_RCC		SYSCTL_PERIPH_GPIOC
#define SPL06_CS_GPIO		GPIO_PORTC_BASE
#define SPL06_CS_PIN		GPIO_PIN_7

#define AK8975_CS_RCC		SYSCTL_PERIPH_GPIOD
#define AK8975_CS_GPIO		GPIO_PORTD_BASE
#define AK8975_CS_PIN		GPIO_PIN_2

#endif
