

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2017 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This file implements some simple busy timers.  They are designed to be
 * portable, not performant.
 * TIM6 is appropriated for usage.
 */
#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>

#include "delay.h"

static volatile uint64_t _millis = 0;

uint64_t millis(void) {
    return _millis;
}

void sys_tick_handler(void) {
    // Increment our monotonic clock
    _millis++;
}

static void systick_setup(void) {
    // Set the systick clock source to our main clock
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    // Clear the Current Value Register so that we start at 0
    STK_CVR = 0;
    // In order to trigger an interrupt every millisecond, we can set the reload
    // value to be the speed of the processor / 1000 - 1
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    // Enable interrupts from the system tick clock
    systick_interrupt_enable();
    // Enable the system tick counter
    systick_counter_enable();
}


void delay_setup(void)
{
        systick_setup();
}


void delay(uint64_t duration) {
    const uint64_t until = millis() + duration;
    while (millis() < until);
}

void delay_us(uint16_t us)
{
        for(int i=0; i< 2*us; i++){
                __asm__("nop");
        }
        return;
}

