#ifndef _USART_USER_SETTING_H_
#define _USART_USER_SETTING_H_


#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
#define EN_USART2_RX 			1		//使能（1）/禁止（0）串口1接收
#define EN_USART3_RX 			1		//使能（1）/禁止（0）串口1接收
#define EN_USART0_RX 			1		//使能（1）/禁止（0）串口1接收
#define EN_USART6_RX			0
//通过宏定义选择printf函数的通信端口
#define USE_UART0_PRINTF
//#define USE_UART1_PRINTF
//#define USE_UART2_PRINTF
//#define USE_UART3_PRINTF



#if defined (USE_UART0_PRINTF)
#define Printf_Port				UART0_BASE
#elif defined (USE_UART1_PRINTF)
#define Printf_Port				UART1_BASE
#elif defined (USE_UART2_PRINTF)
#define Printf_Port				UART2_BASE
#elif defined (USE_UART3_PRINTF)
#define Printf_Port				UART3_BASE

#endif

#endif
