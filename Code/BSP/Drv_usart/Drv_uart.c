//-----------------------------------------------------------------------------
// Drv_uart.c
//
//  Created on	: 2018-4-18
//      Author	: Divenire
//		version	: V1.1
//		brief	:
//-----------------------------------------------------------------------------
// Attention:
//
//
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "BSP\Drv_include.h"

/*****************************************************************************
* Function Name   : Printf重构
* Input           : None
* Output          : None
* Return          : None
* Description     : 加入以下代码,支持printf函数,而不需要选择use MicroLIB
*****************************************************************************/
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
    x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
    //
    // Wait until space is available.
    //
    while (HWREG(Printf_Port + UART_O_FR) & UART_FR_TXFF)
        ;
    //
    // Send the char.
    //
    HWREG(Printf_Port + UART_O_DR) = ch;
    return ch;
}
#endif
/*****************************************************************************
* Function Name   : Uart0_Init
* Input           : None
* Output          : None
* Return          : None
* Description     : 初始化Uart0
*****************************************************************************/
void Uart0_Init(uint32_t bound)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //uart0,波特率115200,8位数据位1位停止位无奇偶校验
    UARTConfigSetExpClk(UART0_BASE, Tiva_SysClock, bound, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    //不使用FIFO
    UARTFIFODisable(UART0_BASE);
    IntMasterEnable(); //开启总中断
    IntEnable(INT_UART0);

#if EN_USART0_RX
    UARTIntEnable(UART0_BASE, UART_INT_RX); //仅开启RX中断
#endif
}
/*****************************************************************************
* Function Name   : Uart0_putbuff
* Input           : buff	发送数据地址的首指针
					len		发送数据的长度
* Output          : None
* Return          : None
* Description     : None
*****************************************************************************/
void Uart0_putbuff(uint8_t *buff, uint32_t len)
{
    while (len--)
    {
        UARTCharPut(UART0_BASE, *buff++);
    }
}
void UART0_Handler()
{
    uint32_t data;
    UARTIntClear(UART0_BASE, UARTIntStatus(UART0_BASE, true));
    data = UARTCharGet(UART0_BASE);
    if (data == 6)
    {
        printf("ok");
    }
    else
    {
        printf("data is %d", data);
    }
}
/*****************************************************************************
* Function Name   : Uart1_Init
* Input           : None
* Output          : None
* Return          : None
* Description     : 初始化Uart1， 用于蓝牙数据的传送
*****************************************************************************/
void Uart1_Init(uint32_t bound)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //uart1,8位数据位1位停止位无奇偶校验
    UARTConfigSetExpClk(UART1_BASE, Tiva_SysClock, bound, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    IntMasterEnable(); //开启总中断
    IntEnable(INT_UART1);

#if EN_USART1_RX
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT); //仅开启RX中断
#endif
}

/*****************************************************************************
* Function Name   : Uart1_putbuff
* Input           : buff	发送数据地址的首指针
					len		发送数据的长度
* Output          : None
* Return          : None
* Description     : None
*****************************************************************************/
void Uart1_putbuff(uint8_t *buff, uint32_t len)
{
    while (len--)
    {
        UARTCharPut(UART1_BASE, *buff++);
    }
}
/*****************************************************************************
* Function Name   : Uart6_Init
* Input           : None
* Output          : None
* Return          : None
* Description     : 初始化Uart6, 用于遥控器通道数据接收
*****************************************************************************/
void Uart2_Init(uint32_t bound)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2); //使能uart6
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //开外设使能
    GPIOPinConfigure(GPIO_PA6_U2RX);
    GPIOPinConfigure(GPIO_PA7_U2TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7); //使能相应IO
    //
    UARTConfigSetExpClk(UART2_BASE, Tiva_SysClock, bound, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_TWO | UART_CONFIG_PAR_EVEN));
    IntMasterEnable();    //开启总中断
    IntEnable(INT_UART2); //开启uart3中断
                          //UARTIntRegister(INT_UART2,UART2_IntHandler);
#if EN_USART2_RX
    UARTIntEnable(UART2_BASE, UART_INT_RX); //仅开启RX中断
#endif
}
