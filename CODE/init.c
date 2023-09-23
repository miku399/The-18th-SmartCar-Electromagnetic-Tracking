#include "headfile.h"

//pwm初始化
void my_pwm_init()
{
//	pwm_init(PWMB_CH1_P74, 50, 2400);
	
	//pwm_init(PWMA_CH1P_P60, 17000, 0);     
	pwm_init(PWMA_CH2P_P62, 17000, 0);
	
	//pwm_init(PWMA_CH3P_P64, 17000, 0);
	pwm_init(PWMA_CH4P_P66, 17000, 0);
}
//定时器中断初始化
void my_pit_init()
{
//		pit_timer_ms(TIM_0, 5);
		pit_timer_ms(TIM_1, 5);
		pit_timer_ms(TIM_2, 5);
//		pit_timer_ms(TIM_3, 5);
	  pit_timer_ms(TIM_4, 5);
}
//ADC初始化
void my_adc_init()
{
	adc_init(ADC_P01,ADC_SYSclk_DIV_2);
	adc_init(ADC_P00,ADC_SYSclk_DIV_2);
	adc_init(ADC_P13,ADC_SYSclk_DIV_2);
	adc_init(ADC_P11,ADC_SYSclk_DIV_2);
	adc_init(ADC_P06,ADC_SYSclk_DIV_2);
	adc_init(ADC_P10,ADC_SYSclk_DIV_2);
	adc_init(ADC_P05,ADC_SYSclk_DIV_2);
}
//void my_gpio_init()
//{
//	icm20602_init_simspi();
//	gyroOffsetInit();

//}