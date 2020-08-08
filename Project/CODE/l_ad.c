#include "l_ad.h"

int16 l_h_1,l_h_2,l_s_1,l_s_2,l_h_m;

void l_init(void)
{
    adc_init(ADC_P10, ADC_SYSclk_DIV_2);						//P10引脚		
	adc_init(ADC_P11, ADC_SYSclk_DIV_2);						//P11引脚		
	adc_init(ADC_P13, ADC_SYSclk_DIV_2);						//P13引脚		
	adc_init(ADC_P14, ADC_SYSclk_DIV_2);						//P14引脚		
    adc_init(ADC_P15, ADC_SYSclk_DIV_2);						//P15引脚，电容电压读取		
    adc_init(ADC_P16, ADC_SYSclk_DIV_2);						//P16引脚		
}

void getl_once(void)
{
    l_h_1 = adc_once(ADC_P14,ADC_10BIT);  //采集ADC_P14电压，精度10位
	l_h_2 = adc_once(ADC_P11,ADC_10BIT); 	//采集ADC_P11电压，精度10位
	l_s_1 = adc_once(ADC_P13,ADC_10BIT);	//采集ADC_P13电压，精度10位
	l_s_2 = adc_once(ADC_P10,ADC_10BIT);	//采集ADC_P10电压，精度10位
	l_h_m = adc_once(ADC_P16,ADC_10BIT);	//采集ADC_P12电压，精度10位
}

