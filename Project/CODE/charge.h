#ifndef __CHARGE_H_
#define __CHARGE_H_

#include "zf_adc.h"
#include "zf_pwm.h"

#define P_SET 30
#define CHARGE_P 0
#define CHARGE_I 0
#define CHARGE_D 0
void charge_init(void);
void charge_control(void);
#endif