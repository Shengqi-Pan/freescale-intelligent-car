#include "l_ad.h"

uint16 l_data1,l_data2,l_data3,l_data4;

void l_init(void)
{
    adc_init(ADC_P10,0);						//P10引脚		
	adc_init(ADC_P11,0);						//P11引脚		
	adc_init(ADC_P13,0);						//P13引脚		
	adc_init(ADC_P14,0);						//P14引脚		
}

void getl_once(void)
{
    l_data1 = adc_once(ADC_P10,ADC_10BIT);  //采集ADC_P10电压，精度10位
	l_data2 = adc_once(ADC_P11,ADC_10BIT); 	//采集ADC_P11电压，精度10位
	l_data3 = adc_once(ADC_P13,ADC_10BIT);	//采集ADC_P13电压，精度10位
	l_data4 = adc_once(ADC_P14,ADC_10BIT);	//采集ADC_P14电压，精度10位
}

