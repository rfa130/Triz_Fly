/**************************************************************************//**
 * @file     system_TM4C12x.c
 * @brief    系统层函�?
 * @version  V1.2
 * @date     2018/4/14
 *
 * @note
 *                                                             Designed by Divenire
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "system_TM4C12x.h"
#include "include.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

void SystemInit (void)
{
		//uint32_t rcc_clock;
		//系统时钟120MHz
		//rcc_clock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_USE_PLL |SYSCTL_OSC_MAIN| SYSCTL_CFG_VCO_480,120000000);
		SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_USE_PLL |SYSCTL_OSC_MAIN| SYSCTL_CFG_VCO_480,120000000);
		//int i;
		//i = i;
}
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

#include <stdio.h>
/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */

  while (1)
  {
		  printf("Tiva is dead");
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}
