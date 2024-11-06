#pragma once

#include <span>
#include <cstdint>
#include <cstddef>


enum struct ModbusCommand {
    READ_COILS = 0x01,
    READ_DIGITAL_INPUT = 0x02,
    READ_HOLDING = 0x03,
    READ_INPUTS = 0x04,
    WRITE_COIL = 0x05,
    WRITE_HOLDING = 0x06,
    WRITE_MULTIPLE_COILS = 0x0F,
    WRITE_MULTIPLE_HOLDING = 0x10
};

enum struct ModbusCommandResult {
    OK = 0,
    FAIL
};

class ModbusClientUpper {

    ModbusCommandResult process_pdu(std::span<std::byte> pdu){
        
    }

};