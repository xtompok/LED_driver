#ifndef TIME_H
#define TIME_H

#include <stdint.h>


extern volatile uint32_t __millis;

void systick_setup(void);

void delay(uint32_t ms);

inline uint32_t millis(void){
    return __millis;
}

//void sys_tick_handler(void);

#endif