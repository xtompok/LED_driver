#ifndef CONFIG_H
#define CONFIG_H

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>

#define IO0_PORT	GPIOA
#define IO0_PIN		GPIO5


#define PORT_O1		GPIOA
#define PIN_O1		GPIO6
#define PORT_O2		GPIOA
#define PIN_O2		GPIO7
#define PORT_O3		GPIOB
#define PIN_O3		GPIO1

#define AD_TEMP_INT_IDX		2
#define AD_VREF_INT_IDX		3

#define ADC_TEMP_V25            1430
#define ADC_TEMP_SLOPE          43

#define AD_I_IN_PORT		GPIOA
#define AD_I_IN_PIN		GPIO0
#define AD_I_IN_CH		0
#define AD_I_IN_IDX		0
#define AD_I_IN_RES		4

#define AD_V_IN_PORT		GPIOA
#define AD_V_IN_PIN		GPIO4
#define AD_V_IN_CH		4
#define AD_V_IN_IDX		1
#define AD_V_IN_RES_HIGH	24000
#define AD_V_IN_RES_LOW		1500

#define ADC_PORT	GPIOA
#define ADC_I_PIN	GPIO0
#define ADC_VCC_PIN	GPIO4
#define ADC_NCHANNELS	4

#define USART_PORT	GPIOA
#define USART_TX_PIN	GPIO2
#define USART_RX_PIN	GPIO3
#define USART_DE_PIN	GPIO1

#define MODBUS_OUR_ADDRESS	0x01
#define MODBUS_USART	USART1
#define MODBUS_TIMER	TIM1
#define CPU_CLOCK_MHZ	48
#define MODBUS_NVIC_TIMER_IRQ	NVIC_TIM1_BRK_UP_TRG_COM_IRQ
#define MODBUS_NVIC_USART_IRQ	NVIC_USART1_IRQ
#define MODBUS_TIMER_ISR	tim1_brk_up_trg_com_isr
#define MODBUS_USART_ISR	usart1_isr
#define MODBUS_BAUD_RATE	115200

#define MODBUS_PWM0_REG		10
#define MODBUS_PWM1_REG		11
#define MODBUS_PWM2_REG		12

#define MODBUS_BASE_STATUS_REG	1
#define MODBUS_BASE_UPTIME_REG	2
#define MODBUS_BASE_VIN_REG	5
#define MODBUS_BASE_IIN_REG	6
#define MODBUS_BASE_3V3_REG	7
#define MODBUS_BASE_TEMPINT_REG	8
#define MODBUS_TXEN_GPIO_PORT	GPIOA
#define MODBUS_TXEN_GPIO_PIN	GPIO1


#endif