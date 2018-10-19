#ifndef _DRV_ADC_H_
#define _DRV_ADC_H_
#include "include.h"

#define ADC_Sequence   3       //ADCWheel sequence, sequence 0, ss0;
#define ADCMAX              4096
#define ADCREFVOLTS         3300

extern uint32_t AdcValue;

void Drv_AdcInit(void);
void ADC_ValueGet(void);

#endif
