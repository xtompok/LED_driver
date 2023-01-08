#ifndef ADC_H
#define ADC_H

#define ADC_NCHANNELS   4
#define ADC_AVG_WINDOW  8   // FIXME pro některé hodnoty 

extern uint16_t adc_temp_int;
extern uint16_t adc_3V3_v;
extern uint16_t adc_vin_v;
extern uint16_t adc_vin_i;

void dma_setup(void);
void adc_setup(void);

#endif