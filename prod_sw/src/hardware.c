#include "hardware.h"
#include "config.h"
#include "timer.h"
#include "time.h"

bool modbus_check_discrete_input(u16 addr){
	(void)addr;
	return false;
}
bool modbus_get_discrete_input(u16 addr){
	(void)addr;
	return false;
}

bool modbus_check_coil(u16 addr){
	switch (addr){
		default:
			return false;
	}
}

bool modbus_get_coil(u16 addr){
	switch (addr){
		default:
			return false;
	}
	return false;
}
void modbus_set_coil(u16 addr, bool value){
	switch (addr) {
	}
}

bool modbus_check_input_register(u16 addr){
	switch (addr){

		case MODBUS_BASE_VIN_REG:
		case MODBUS_BASE_IIN_REG:
		case MODBUS_BASE_3V3_REG:
		case MODBUS_BASE_TEMPINT_REG:
			return true;
		default:
			return false;
	}
}
u16 modbus_get_input_register(u16 addr){
	switch (addr){
		case MODBUS_BASE_VIN_REG:
			return UINT16_MAX;
		case MODBUS_BASE_IIN_REG:
			return UINT16_MAX;
		case MODBUS_BASE_3V3_REG:
//			return adc_3V3_v;
			return UINT16_MAX;
		case MODBUS_BASE_TEMPINT_REG:
//			return adc_temp_int;
			return UINT16_MAX;
	}
	return 0;
}

bool modbus_check_holding_register(u16 addr){
	switch (addr)
	{
		case MODBUS_BASE_STATUS_REG:
		case MODBUS_BASE_UPTIME_REG:
		case MODBUS_PWM0_REG:
		case MODBUS_PWM1_REG:
		case MODBUS_PWM2_REG:
			return true;
	default:
		return false;
	}
}

u16 modbus_get_holding_register(u16 addr){
	switch (addr)
	{
		case MODBUS_BASE_STATUS_REG:
			return 0;
		case MODBUS_BASE_UPTIME_REG:
			return millis()/1024;
		case MODBUS_PWM0_REG:
			return get_pwm0();
		case MODBUS_PWM1_REG:
			return get_pwm1();
		case MODBUS_PWM2_REG:
			return get_pwm2();
	default:
		return 0;
	}
}
void modbus_set_holding_register(u16 addr, u16 value){
	switch (addr)
	{
		case MODBUS_PWM0_REG:
			set_pwm0(value);
			break;
		case MODBUS_PWM1_REG:
			set_pwm1(value);
			break;
		case MODBUS_PWM2_REG:
			set_pwm2(value);
			break;
	default:
		break;
	}

}

void modbus_ready_hook(void){}
void modbus_frame_start_hook(void){}

const char * const modbus_id_strings[] = {
	"Jethro",
	"LED_Driver",
	"0.1"
};
