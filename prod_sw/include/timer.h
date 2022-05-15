#ifndef TIMER_H
#define TIMER_H

void timer_setup(void);

void set_pwm0(uint16_t pwm);
void set_pwm1(uint16_t pwm);
void set_pwm2(uint16_t pwm);

uint16_t get_pwm0(void);
uint16_t get_pwm1(void);
uint16_t get_pwm2(void);

#endif