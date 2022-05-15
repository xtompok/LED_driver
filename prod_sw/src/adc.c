#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>

#include "config.h"
#include "adc.h"
#include "time.h"


volatile uint16_t adc_res[ADC_NCHANNELS];
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
	//n_dmaint++;
}

void adc_comp_isr(void){
	//n_dmaint++;
}