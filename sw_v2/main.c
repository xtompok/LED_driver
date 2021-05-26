/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2012 Karl Palsson <karlp@tweak.net.au>
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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/flash.h>
#include "delay.h"
#include <stdio.h>
#include <string.h>


#define PORT_O1 GPIOA
#define PIN_O1 GPIO6
#define PORT_O2 GPIOA
#define PIN_O2 GPIO7
#define PORT_O3 GPIOB
#define PIN_O3 GPIO1

#define ADC_PORT GPIOA
#define ADC_I_PIN GPIO0
#define ADC_VCC_PIN GPIO4
#define ADC_NCHANNELS 4

#define USART_PORT GPIOA
#define USART_TX_PIN GPIO2
#define USART_RX_PIN GPIO3
#define USART_DE_PIN GPIO1

volatile uint16_t adc_res[ADC_NCHANNELS];
//volatile uint32_t debug_res[ADC_NCHANNELS] = {1,2,3,4,5,6,7,8};
uint16_t test_data[] = {0xCC, 0xCC, 0xCC, 0xCC};
volatile uint32_t n_dmaint;

static void  rcc_clock_setup_in_hse_16mhz_out_48mhz(void)
 {
         rcc_osc_on(RCC_HSE);
         rcc_wait_for_osc_ready(RCC_HSE);
         rcc_set_sysclk_source(RCC_HSE);

         rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
         rcc_set_ppre(RCC_CFGR_PPRE_NODIV);

         flash_prefetch_enable();
         flash_set_ws(FLASH_ACR_LATENCY_024_048MHZ);

         /* PLL: 16MHz * 6 = 48MHz */
         rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL3);
         rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);
         rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);

         rcc_osc_on(RCC_PLL);
         rcc_wait_for_osc_ready(RCC_PLL);
         rcc_set_sysclk_source(RCC_PLL);

         rcc_apb1_frequency = 48000000;
         rcc_ahb_frequency = 48000000;
 }


static void clock_setup(void){

	rcc_clock_setup_in_hse_16mhz_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_DMA1);
	rcc_periph_clock_enable(RCC_TIM3);

}

static void usart_setup(void)
{

	/* Setup USART2 parameters. */
	usart_set_baudrate(USART1, 9600);
	usart_set_databits(USART1, 8);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
}


static void gpio_setup(void)
{

//	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);

//	gpio_mode_setup(PORT_O1, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_O1);


	gpio_mode_setup(PORT_O1, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_O1);
	gpio_mode_setup(PORT_O2, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_O2);
	gpio_mode_setup(PORT_O3, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_O3);
	gpio_set_output_options(PORT_O1, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, PIN_O1);
	gpio_set_output_options(PORT_O2, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, PIN_O2);
	gpio_set_output_options(PORT_O3, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, PIN_O3);
	gpio_set_af(PORT_O1, GPIO_AF1, PIN_O1);
	gpio_set_af(PORT_O2, GPIO_AF1, PIN_O2);
	gpio_set_af(PORT_O3, GPIO_AF1, PIN_O3);


	gpio_mode_setup(USART_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, USART_DE_PIN);
	gpio_mode_setup(USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TX_PIN);
	gpio_mode_setup(USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_RX_PIN);
	
	gpio_set_af(USART_PORT, GPIO_AF1, USART_TX_PIN);
	gpio_set_af(USART_PORT, GPIO_AF1, USART_RX_PIN);

	gpio_mode_setup(ADC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, ADC_I_PIN);
	gpio_mode_setup(ADC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, ADC_VCC_PIN);
}

static void timer_setup(void)
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

uint8_t adc_channels[] = {0,4, ADC_CHANNEL_TEMP, ADC_CHANNEL_VREF};

static void adc_setup(void)
{
	adc_power_off(ADC1);

	adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
//	adc_set_clk_source(ADC1, ADC_CLKSOURCE_PCLK_DIV4);

	adc_calibrate(ADC1);

	adc_set_operation_mode(ADC1, ADC_MODE_SEQUENTIAL);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_239DOT5);

	adc_set_right_aligned(ADC1);

	adc_enable_temperature_sensor();
	adc_enable_vrefint();
	
	adc_set_resolution(ADC1,ADC_RESOLUTION_12BIT);

	adc_set_regular_sequence(ADC1, ADC_NCHANNELS, adc_channels);
	adc_disable_analog_watchdog(ADC1);
	
	// ADC DMA start
	adc_set_continuous_conversion_mode(ADC1);
	adc_set_operation_mode(ADC1,ADC_MODE_SCAN_INFINITE);

	adc_disable_discontinuous_mode(ADC1);
	adc_enable_eoc_sequence_interrupt(ADC1);


	adc_enable_dma_circular_mode(ADC1);
	adc_enable_dma(ADC1);
	// ADC DMA end

	adc_power_on(ADC1);
	delay(10);
	// ADC DMA start
	adc_start_conversion_regular(ADC1);
	return;
/*

//	delay(100);
//	
*/
}

static uint16_t calculate_temperature(uint16_t vcc, uint16_t temp_val){
	#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
	#define VDD_CALIB ((uint32_t) 3300)
	#define AVG_SLOPE ((uint32_t) 4300)
	temp_val = (*TEMP30_CAL_ADDR - (temp_val*vcc/VDD_CALIB))*1000;
	temp_val = (temp_val / AVG_SLOPE)+30;
	return temp_val;
}

static uint16_t calculate_voltage(uint16_t adc_val){
	#define  VREFINT_CAL_ADDR ((uint16_t*)((uint32_t)0x1FFFF7BA))

	uint16_t vrefint_cal;                        // VREFINT calibration value
	vrefint_cal= *VREFINT_CAL_ADDR; // read VREFINT_CAL_ADDR memory location
	
	uint16_t voltage;
	voltage = 3300 * vrefint_cal/adc_val;
	return voltage;
}

static void dma_setup(void){
	adc_power_off(ADC1);
	dma_channel_reset(DMA1, DMA_CHANNEL1);

	dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t) &ADC_DR(ADC1));
	dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t) &adc_res);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
	dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_LOW);

	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
	dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_NCHANNELS);
	dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);

//	dma_enable_mem2mem_mode(DMA1, DMA_CHANNEL1);
//	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);
 
	dma_enable_channel(DMA1, DMA_CHANNEL1);
//	adc_enable_dma(DMA1);
}

void dma1_channel1_isr(void) {
	dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_IFCR_CGIF1);
	n_dmaint++;
}

void adc_comp_isr(void){
	n_dmaint++;
}


static void rs485_send_blocking(char c){
	gpio_set(USART_PORT,USART_DE_PIN);
	delay_us(10);
	usart_send_blocking(USART1,c);
	while(usart_get_flag(USART1,USART_ISR_TC)!=1) {}
	delay_us(10);
	gpio_clear(USART_PORT,USART_DE_PIN);

}

static void rs485_send_str_blocking(char * str){
	gpio_set(USART_PORT, USART_DE_PIN);
	delay_us(10);
	for (int i=0; i<strlen(str); i++){
		usart_send_blocking(USART1,str[i]);
	}
	//usart_send_blocking(USART1,'\n');
	while(usart_get_flag(USART1,USART_ISR_TC)!=1) {}
	delay_us(10);
	gpio_clear(USART_PORT, USART_DE_PIN);

}

int main(void)
{

	clock_setup();
	gpio_setup();
	usart_setup();
	delay_setup();
	timer_setup();
	dma_setup();
	adc_setup();
	delay(100);

		char str[20];

	/* Blink the LED (PC8) on the board. */
	while (1) {
		/*
		dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_NCHANNELS);
 
		snprintf(str,20,"ndata: %d",dma_get_number_of_data(DMA1,DMA_CHANNEL1));
		rs485_send_str_blocking(str);
		dma_enable_channel(DMA1, DMA_CHANNEL1);
		snprintf(str,20,"ndata: %d",dma_get_number_of_data(DMA1,DMA_CHANNEL1));
		rs485_send_str_blocking(str);
		adc_start_conversion_regular(ADC1);
		for (int i=0;i<20;i++){
		snprintf(str,20,"ndata: %d",dma_get_number_of_data(DMA1,DMA_CHANNEL1));
		rs485_send_str_blocking(str);
		}
		*/

		//snprintf(str,20,"tim: %d",timer_get_counter(TIM3));
		//rs485_send_str_blocking(str);

		rs485_send_blocking('\n');


		/*
		for (int i=0; i<4; i++){
			snprintf(str,20,"adc%d: %5d, ",i,adc_res[i]);
			rs485_send_str_blocking(str);

		}
		
		snprintf(str,20,"adc: %5d, int: %d",ADC_DR(ADC1),n_dmaint);
		rs485_send_str_blocking(str);
		snprintf(str,20,"ovr:%d ",adc_get_overrun_flag(ADC1));
		rs485_send_str_blocking(str);
		adc_clear_overrun_flag(ADC1);
		snprintf(str,20,"ovr:%d ",adc_get_overrun_flag(ADC1));
		rs485_send_str_blocking(str);
		adc_clear_overrun_flag(ADC1);*/

		uint16_t current;
		uint16_t vled;
		uint16_t temperature;
		uint16_t v_mcu;

		v_mcu = adc_res[3];
		//snprintf(str,20,"3V3_raw: %d ",v_mcu);
		//rs485_send_str_blocking(str);
		v_mcu = calculate_voltage(v_mcu);

		snprintf(str,20,"3V3: %d ",v_mcu);
		rs485_send_str_blocking(str);

		current = adc_res[0];
		current = 333 * v_mcu * current / 4095 / 50;

		snprintf(str,20,"current: %d ",current);
		rs485_send_str_blocking(str);

		vled = adc_res[1];
		vled = 25500 * (v_mcu*vled/4095)/1500;

		snprintf(str,20,"V_led: %d ",vled);
		rs485_send_str_blocking(str);

		temperature = adc_res[2];
		temperature = calculate_temperature(v_mcu, temperature);

		snprintf(str,20,"temperature: %d ",temperature);
		rs485_send_str_blocking(str);
		


		rs485_send_blocking('\n');
	

		/* Using API function gpio_toggle(): */
	//	gpio_toggle(PORT_LED, PIN_LED);	/* LED on/off */
//		gpio_toggle(PORT_O1, PIN_O1);
		delay(100);
	/*	rs485_send_blocking('y');
		rs485_send_blocking( adc_get_overrun_flag(ADC1)+'0');
		rs485_send_blocking( dma_get_interrupt_flag(DMA1,DMA_CHANNEL1,DMA_TEIF)+'0');

		rs485_send_blocking( dma_get_interrupt_flag(DMA1,DMA_CHANNEL1,DMA_TCIF)+'0');

		if ( adc_get_overrun_flag(ADC1)){
			adc_clear_overrun_flag(ADC1);		
		}
		rs485_send_blocking( adc_get_overrun_flag(ADC1)+'0');
		rs485_send_blocking('\n');

		if(dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_ISR_TCIF1)){
			rs485_send_blocking('F');
			
		} else {
			rs485_send_blocking('f');
		
		}
*/
/*
		rs485_send_blocking('d');
		uint16_t rch;
		rch = usart_recv_blocking(USART1);
		rs485_send_blocking('b');
		timer_set_oc_value(TIM3,TIM_OC1,rch);
		rs485_send_blocking('c');
		rs485_send_blocking(rch);

*/

/*
		for (int i=0; i<1000; i++){
			timer_set_oc_value(TIM1,TIM_OC1,i);
			delay(10);
		}
*/
	/*	for (int i=0;i<ADC_NCHANNELS;i++){
			char str[20];
			snprintf(str,20,"c: %d v:%6lu",i,adc_res[i]);
			rs485_send_str_blocking(str);
		}
		snprintf(str,20,"dmaint: %d",n_dmaint);
		rs485_send_str_blocking(str);

		snprintf(str,20,"ndata: %d",dma_get_number_of_data(DMA1,DMA_CHANNEL1));
		rs485_send_str_blocking(str);
*/
	}

	return 0;
}
