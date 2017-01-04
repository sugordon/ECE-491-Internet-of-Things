# Sources

SRCS = main.c stm32f4xx_it.c system_stm32f4xx.c syscalls.c utils.c \
	   netconf.c stm32f4x7_eth_bsp.c tcp_echoserver.c lib/src/ethernet/stm32f4x7_eth.c lib/src/lwip/core/def.c lib/src/lwip/core/timers.c

# Project name

PROJ_NAME=stm32f4_sample
OUTPATH=build

###################################################

# Check for valid float argument
# NOTE that you have to run make clan after
# changing these as hardfloat and softfloat are not
# binary compatible
ifneq ($(FLOAT_TYPE), hard)
ifneq ($(FLOAT_TYPE), soft)
override FLOAT_TYPE = hard
#override FLOAT_TYPE = soft
endif
endif

###################################################

BINPATH=~/sat/bin
CC=$(BINPATH)/arm-none-eabi-gcc
OBJCOPY=$(BINPATH)/arm-none-eabi-objcopy
SIZE=$(BINPATH)/arm-none-eabi-size

CFLAGS  = -std=gnu99 -g -O2 -Wall -Tstm32_flash.ld
CFLAGS += -mlittle-endian -mthumb -mthumb-interwork -nostartfiles -mcpu=cortex-m4

ifeq ($(FLOAT_TYPE), hard)
CFLAGS += -fsingle-precision-constant -Wdouble-promotion
CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=hard
#CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
else
CFLAGS += -msoft-float
endif

###################################################

vpath %.c src
vpath %.a lib

ROOT=$(shell pwd)

# Include files, why so jank
CFLAGS += -Iinc -Ilib -Ilib/inc \
		  -Ilib/inc/core -Ilib/inc/peripherals \
		  -Ilib/inc/ethernet \
		  -Ilib/inc/lwip -Ilib/inc/lwip/lwip \
		  -Ilib/inc/lwip/ipv4 -Ilib/inc/lwip/netif \
		  -Ilib/src/lwip/port/STM32F4x7 -Ilib/src/lwip/port/STM32F4x7/Standalone

SRCS += lib/startup_stm32f4xx.s # add startup file to build

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib proj

all:
	make build
	make flash

build: lib proj
	$(SIZE) $(OUTPATH)/$(PROJ_NAME).elf

flash:
	sudo openocd -f ~/Documents/coding/iot/openocd/tcl/interface/stlink-v2.cfg -c "set WORKAREASIZE 0x2000" -f ~/Documents/coding/iot/openocd/tcl/target/stm32f4x_stlink.cfg -c "program build/stm32f4_sample.elf verify reset"

lib:
	$(MAKE) -C lib FLOAT_TYPE=$(FLOAT_TYPE)

proj: 	$(OUTPATH)/$(PROJ_NAME).elf

$(OUTPATH)/$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -Llib -lstm32f4 -lm
	$(OBJCOPY) -O ihex $(OUTPATH)/$(PROJ_NAME).elf $(OUTPATH)/$(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(OUTPATH)/$(PROJ_NAME).elf $(OUTPATH)/$(PROJ_NAME).bin

clean:
	rm -f *.o
	rm -f $(OUTPATH)/$(PROJ_NAME).elf
	rm -f $(OUTPATH)/$(PROJ_NAME).hex
	rm -f $(OUTPATH)/$(PROJ_NAME).bin
	$(MAKE) clean -C lib # Remove this line if you don't want to clean the libs as well
	
