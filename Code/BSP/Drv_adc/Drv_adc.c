
#include "BSP/Drv_include.h"

uint32_t AdcValue;

void Drv_AdcInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);         //Enable ADC0 Peripheral;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);        //Enable PORTE
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_0);        //Set PE0 as adc0 mode;
    ADCSequenceConfigure(ADC0_BASE, ADC_Sequence, ADC_TRIGGER_PROCESSOR, 0);                        //Use Sequence 3, triggered by processor;

    ADCSequenceStepConfigure(ADC0_BASE, ADC_Sequence, 0, ADC_CTL_CH16 | ADC_CTL_IE | ADC_CTL_END);  //Use Sequence 3, step 0, CH3, 采样并且产生中断，结束采样;
    ADCSequenceEnable(ADC0_BASE, ADC_Sequence);    //Enable ADC sampling;
    ADCIntClear(ADC0_BASE,ADC_Sequence);           //Clear interrupt flag.
	ADCProcessorTrigger(ADC0_BASE, ADC_Sequence);   //处理器触发采样
}

void ADC_ValueGet(void)
{
    while(!ADCIntStatus(ADC0_BASE, ADC_Sequence, false));        //等待采样结束

    ADCIntClear(ADC0_BASE, ADC_Sequence);                       //清除中断标志
    ADCSequenceDataGet(ADC0_BASE, ADC_Sequence, &AdcValue); //获取采样值

    ADCProcessorTrigger(ADC0_BASE, ADC_Sequence);   //处理器触发采样
}
