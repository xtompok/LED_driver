#ifndef HARDWARE_H
#define HARDWARE_H

#include "util.h"

bool modbus_check_discrete_input(u16 addr);
bool modbus_get_discrete_input(u16 addr);
bool modbus_check_coil(u16 addr);
bool modbus_get_coil(u16 addr);
void modbus_set_coil(u16 addr, bool value);
bool modbus_check_input_register(u16 addr);
u16 modbus_get_input_register(u16 addr);
bool modbus_check_holding_register(u16 addr);
u16 modbus_get_holding_register(u16 addr);
void modbus_set_holding_register(u16 addr, u16 value);
void modbus_ready_hook(void);
void modbus_frame_start_hook(void);

extern const char * const modbus_id_strings[];


#endif