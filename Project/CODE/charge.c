#include "charge.h"

void charge_init(void)
{
    adc_init(ADC_P06, ADC_SYSclk_DIV_2);						//P06引脚,电流采样	
	adc_init(ADC_P17, ADC_SYSclk_DIV_2);						//P17引脚，电压采样
    pwm_init(PWM4P_P16, 20000, 0);                              //初始化PWM2  使用P16引脚  初始化频率为20Khz
}

void charge_control(void)
{
    static uint16 u_in, i_in, pwm_out=2000;
    static float u, i, p, er, er_last = 0, er_previous = 0;
    u_in = adc_once(ADC_P17,ADC_10BIT);
    i_in = adc_once(ADC_P06,ADC_10BIT);
    u = u_in * 5.0 / 1024;
    i = i_in / 102.4;
    p = u * i;
    er = P_SET - p;
    pwm_out = pwm_out + CHARGE_P * (er - er_last) + CHARGE_I * er + CHARGE_D * (er - 2 * er_last + er_previous); 
    er_previous = er_last;
    er_last = er;
    pwm_duty(PWM4P_P16, pwm_out);
}

