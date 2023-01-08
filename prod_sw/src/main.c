#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>

#include "time.h"
#include "config.h"
#include "timer.h"
#include "modbus.h"
#include "adc.h"
 

static void rcc_clock_setup_in_hse_16mhz_out_48mhz(void)
{
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	rcc_set_sysclk_source(RCC_HSE);

	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
	rcc_set_ppre(RCC_CFGR_PPRE_NODIV);

	flash_prefetch_enable();
	flash_set_ws(FLASH_ACR_LATENCY_024_048MHZ);

	/* PLL: 16MHz * 3 = 48MHz */
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL3);
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);
	rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);

	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);
	rcc_set_sysclk_source(RCC_PLL);

	rcc_apb1_frequency = 48000000;
	rcc_ahb_frequency = 48000000;
}

static void gpio_setup(void){
	gpio_mode_setup(IO0_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, IO0_PIN);
	gpio_set_output_options(IO0_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LOW, IO0_PIN);

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

static void clock_setup(void){
	rcc_clock_setup_in_hse_16mhz_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_DMA1);
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_TIM1);


}

static void irq_setup(void){
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
}

int main(void){
	clock_setup();
	systick_setup();
	gpio_setup();
	timer_setup();
	irq_setup();
	dma_setup();
	adc_setup();

	modbus_init();
	
	//TODO:
	// - calculate current
	// - on/off coil register
	
	

	while (1){
		modbus_loop();

	}	



}