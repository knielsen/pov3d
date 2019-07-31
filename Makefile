STM_DIR=/kvm/src/STM32F4xx_DSP_StdPeriph_Lib_V1.6.1
STM_SRC = $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/src

WITH_WIRELESS_BOOTLOADER := 0
TARGET=ledtorus

ifeq ($(WITH_WIRELESS_BOOTLOADER), 1)
	FLASH_ADDR := 0x8004000
else
	FLASH_ADDR := 0x8000000
endif

OBJS = $(TARGET).o led.o dbg.o spi.o timers.o adc.o tlc.o my_misc.o \
  gfx.o font_tonc.o nrf24l01p.o sd_sdio.o hall.o \
  stm324xg_eval_sdio_sd.o ev_fat.o

vpath %.c $(STM_SRC)
STM_OBJS = system_stm32f4xx.o
STM_OBJS  += stm32f4xx_rcc.o
STM_OBJS  += stm32f4xx_gpio.o
STM_OBJS  += stm32f4xx_usart.o
STM_OBJS  += stm32f4xx_spi.o
STM_OBJS  += stm32f4xx_tim.o
STM_OBJS  += stm32f4xx_dma.o
STM_OBJS  += stm32f4xx_adc.o
STM_OBJS  += stm32f4xx_syscfg.o
STM_OBJS  += stm32f4xx_exti.o
STM_OBJS  += misc.o
STM_OBJS  += stm32f4xx_sdio.o

INC_DIRS += $(STM_DIR)/Libraries/CMSIS/Include
INC_DIRS += $(STM_DIR)/Libraries/CMSIS/Device/ST/STM32F4xx/Include
INC_DIRS += $(STM_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/inc
INC_DIRS += .
INC = $(addprefix -I,$(INC_DIRS))

# We need to copy in the ST sdio_sd source files to the tree.
# Because the stm324xg_eval_sdio_sd.h file includes its config from
# "stm324xg_eval.h", and we want to pick up our own version of that,
# not the one that's shipped in the same directory as
# stm324xg_eval_sdio_sd.h
ST_SDIO_SD_C = $(STM_DIR)/Utilities/STM32_EVAL/STM3240_41_G_EVAL/stm324xg_eval_sdio_sd.c
ST_SDIO_SD_H = $(STM_DIR)/Utilities/STM32_EVAL/STM3240_41_G_EVAL/stm324xg_eval_sdio_sd.h


CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy


STARTUP_OBJ=startup_stm32f4xx.o
STARTUP_SRC=$(STM_DIR)/Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/TrueSTUDIO/startup_stm32f40xx.s
LINKSCRIPT=$(TARGET).ld

ARCH_FLAGS=-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -ffast-math

CFLAGS=-ggdb -O2 -std=c99 -Wall -Wextra -Warray-bounds -Wno-unused-parameter $(ARCH_FLAGS) $(INC) -DSTM32F40XX -DUSE_STDPERIPH_DRIVER
LDFLAGS=-Wl,--gc-sections -Wl,--defsym,FLASH_ADDR=$(FLASH_ADDR) -lm


.PHONY: all flash clean tty cat

all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf

$(TARGET).elf: $(OBJS) $(STM_OBJS) $(STARTUP_OBJ) $(LINKSCRIPT)
	$(LD) $(ARCH_FLAGS) -T $(LINKSCRIPT) -o $@ $(STARTUP_OBJ) $(OBJS) $(STM_OBJS) $(LDFLAGS)

$(TARGET).o: $(TARGET).c ledtorus.h

$(STARTUP_OBJ): $(STARTUP_SRC)
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.c ledtorus.h stm32f4xx_conf.h
	$(CC) $(CFLAGS) -c $< -o $@

nrf24l01p.o: nrf24l01p.h
stm324xg_eval_sdio_sd.o: stm324xg_eval_sdio_sd.h
sd_sdio.o: stm324xg_eval_sdio_sd.h

stm324xg_eval_sdio_sd.h: $(ST_SDIO_SD_H)
	ln -s $< $@

stm324xg_eval_sdio_sd.c: $(ST_SDIO_SD_C)
	ln -s $< $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

flash: $(TARGET).bin
	st-flash write $(TARGET).bin $(FLASH_ADDR)

clean:
	rm -f $(OBJS) $(STM_OBJS) $(TARGET).elf $(TARGET).bin $(STARTUP_OBJ)

tty:
	stty -F/dev/stellaris raw -echo -hup cs8 -parenb -cstopb 115200

cat:
	cat /dev/stellaris
