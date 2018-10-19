/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SYSTEM_TM4C12x_CONF_H
#define SYSTEM_TM4C12x_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/flash.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/can.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"

// #include "driverlib/aes.h"
// #include "driverlib/comp.h"
// #include "driverlib/cpu.h"
// #include "driverlib/crc.h"
// #include "driverlib/des.h"
// #include "driverlib/eeprom.h"
// #include "driverlib/emac.h"
// #include "driverlib/epi.h"
// #include "driverlib/hibernate.h"
// #include "driverlib/lcd.h"
// #include "driverlib/mpu.h"
// #include "driverlib/onewire.h"
// #include "driverlib/qei.h"
// #include "driverlib/shamd5.h"
// #include "driverlib/sw_crc.h"
// #include "driverlib/udma.h"
// #include "driverlib/usb.h"
// #include "driverlib/watchdog.h"

/* Uncomment the line below to enable peripheral layer file fuc */
#include "inc/hw_ssi.h"
#include "inc/hw_adc.h"
#include "inc/hw_aes.h"
#include "inc/hw_can.h"
#include "inc/hw_comp.h"
#include "inc/hw_ccm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_des.h"
#include "inc/hw_flash.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_i2c.h"
#include "inc/hw_qei.h"
#include "inc/hw_nvic.h"
#include "inc/hw_gpio.h"
#include "inc/hw_uart.h"

#endif /*SYSTEM_TM4C12x_CONF_H */