#include <stdint.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include "time.h"

volatile  uint32_t __millis;

void systick_setup(void){
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(47999);
    systick_interrupt_enable();
    systick_counter_enable();
}

void sys_tick_handler(void){
    __millis++;
   // usb_send();
}

void delay(uint32_t ms){
    uint32_t now;
    now = __millis;
    while (__millis < now+ms){}
}
