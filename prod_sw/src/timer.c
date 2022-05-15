#include <libopencm3/stm32/timer.h>

#include "timer.h"

void timer_setup(void)
{
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM3, 1);
	timer_set_period(TIM3, 1000);
//	timer_disable_preload(TIM3);
	timer_enable_preload(TIM3);
	timer_continuous_mode(TIM3);
	timer_enable_break_main_output(TIM3);

	timer_disable_oc_output(TIM3, TIM_OC1);
	timer_disable_oc_output(TIM3, TIM_OC2);
	timer_disable_oc_output(TIM3, TIM_OC4);
	timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM2);
	timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1);
	timer_enable_oc_preload(TIM3, TIM_OC1);
	timer_enable_oc_preload(TIM3, TIM_OC2);
	timer_enable_oc_preload(TIM3, TIM_OC4);
	timer_set_oc_value(TIM3, TIM_OC1, 500);	
	timer_set_oc_value(TIM3, TIM_OC2, 500);	
	timer_set_oc_value(TIM3, TIM_OC4, 500);	
	timer_set_oc_polarity_high(TIM3, TIM_OC1);
	timer_set_oc_polarity_high(TIM3, TIM_OC2);
	timer_set_oc_polarity_high(TIM3, TIM_OC4);
	timer_enable_oc_output(TIM3, TIM_OC1);
	timer_enable_oc_output(TIM3, TIM_OC2);
	timer_enable_oc_output(TIM3, TIM_OC4);

	timer_enable_counter(TIM3);
}

void set_pwm0(uint16_t pwm){
	timer_set_oc_value(TIM3, TIM_OC1, pwm);
}
void set_pwm1(uint16_t pwm){
	timer_set_oc_value(TIM3, TIM_OC2, pwm);
}
void set_pwm2(uint16_t pwm){
	timer_set_oc_value(TIM3, TIM_OC4, pwm);
}

uint16_t get_pwm0(void){
	return TIM_CCR1(TIM3);
}
uint16_t get_pwm1(void){
	return TIM_CCR2(TIM3);
}
uint16_t get_pwm2(void){
	return TIM_CCR4(TIM3);
}

