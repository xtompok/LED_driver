PROJECT = led_driver
BUILD_DIR = bin

#SHARED_DIR = ../my-common-code
CFILES = main.c
CFILES += delay.c
#AFILES += api-asm.S

# TODO - you will need to edit these two lines!
DEVICE=stm32f030f4p6
OOCD_TARGET=stm32f0x
OOCD_INTERFACE = stlink-v2

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=./libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ./rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
