#ifndef _init_h
#define _init_h

#include "headfile.h"


//#define boma_4 !gpio_get(D14)
//#define boma_3 !gpio_get(D15)
//#define boma_2 !gpio_get(D12)
//#define boma_1 !gpio_get(D13)

void init_all(void);
void my_pwm_init(void);
void my_pit_init(void);
void my_adc_init(void);
void my_gpio_init(void);
void key_init(void);

#endif
