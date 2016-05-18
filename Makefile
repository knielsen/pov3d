TARGET=ledtorus

OBJS = $(TARGET).o led.o dbg.o spi.o my_misc.o tlc.o \
  #timers.o adc.o \
  #gfx.o font_tonc.o nrf24l01p.o sd_sdio.o hall.o \
  #ev_fat.o \
  #stm324xg_eval_sdio_sd.o
  

OPENCM3_DIR=/kvm/src/libopencm3

INC_DIRS += $(OPENCM3_DIR)/include
INC_DIRS += .
INC = $(addprefix -I,$(INC_DIRS))


CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy


LINKSCRIPT=$(TARGET).ld

ARCH_FLAGS=-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -ffast-math

CFLAGS=-ggdb -O2 -std=c99 -Wall -Wextra -Warray-bounds -Wno-unused-parameter -fno-common $(ARCH_FLAGS) $(INC) -DSTM32F4
LDFLAGS=-Wl,--gc-sections -L$(OPENCM3_DIR)/lib -lopencm3_stm32f4 -lm


.PHONY: all flash clean tty cat

all: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf

$(TARGET).elf: $(OBJS) $(STM_OBJS) $(STARTUP_OBJ) $(LINKSCRIPT)
	$(LD) $(ARCH_FLAGS) -T $(LINKSCRIPT) -o $@ $(STARTUP_OBJ) $(OBJS) $(STM_OBJS) $(LDFLAGS)

$(TARGET).o: $(TARGET).c ledtorus.h

$(STARTUP_OBJ): $(STARTUP_SRC)
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.c ledtorus.h
	$(CC) $(CFLAGS) -c $< -o $@

nrf24l01p.o: nrf24l01p.h

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

flash: $(TARGET).bin
	st-flash write $(TARGET).bin 0x8004000

clean:
	rm -f $(OBJS) $(STM_OBJS) $(TARGET).elf $(TARGET).bin $(STARTUP_OBJ)

tty:
	stty -F/dev/stellaris raw -echo -hup cs8 -parenb -cstopb 115200

cat:
	cat /dev/stellaris
