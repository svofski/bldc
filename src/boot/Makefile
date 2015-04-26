#MCU_TARGET    = atmega8535
MCU_TARGET    = atmega8
#MCU_TARGET    = atmega88
#MCU_TARGET     = atmega168
#MCU_TARGET	= atmega644

PRG            = boot_$(MCU_TARGET)
OBJ            = $(PRG).o

CC             = avr-gcc
LD             = avr-ld

ifeq ($(MCU_TARGET), atmega168)
  LDFLAGS        = --section-start .bootloader=0x3E00
else ifeq ($(MCU_TARGET), atmega644)
  LDFLAGS        = --section-start .bootloader=0xfE00
else ifeq ($(MCU_TARGET), atmega8535)
  LDFLAGS        = --section-start .bootloader=0x1F00 --defsym homebase=0x00
else ifeq ($(MCU_TARGET), atmega8)
  LDFLAGS        = --section-start .bootloader=0x1F00 --defsym homebase=0x00
else ifeq ($(MCU_TARGET), atmega88)
  LDFLAGS        = --section-start .bootloader=0x1F00 --defsym homebase=0x00
else
  ($error unknown mcu_target $(MCU_TARGET))
endif  

CFLAGS         = -Wall -c -mmcu=$(MCU_TARGET)
OBJCOPYFLAGS   = -j .bootloader

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

############################################################
#
# Programmer ...
#
############################################################
PROG=uisp -dprog=avr910 -dserial=/dev/ttyS1 -dspeed=115200 -dpart=auto -v
ERASEFLAGS=--erase 
PROGFLAGS=--upload if=$(PRG).hex
VERIFYFLAGS=--verify
EEPFLAGS= --segment=eeprom --upload if=$(PRG)_eeprom.hex


############################################################################################################

all: hex lst

obj:	$(PRG).o
elf:	$(PRG).elf
hex:	$(PRG).hex
lst:	$(PRG).lst
bin:	$(PRG).bin

%.o:    %.S
	$(CC) $(CFLAGS) $< -o $@

%.elf:	%.o
	$(LD) $(LDFLAGS) $< -o $@

%.lst: %.elf
	$(OBJDUMP) -D $< > $@

%.hex: %.elf
	$(OBJCOPY) $(OBJCOPYFLAGS) -O ihex $< $@

%.bin: %.elf
	$(OBJCOPY) $(OBJCOPYFLAGS) -O binary $< $@


prog: $(TARGET)
	$(PROG) $(ERASEFLAGS) $(PROGFLAGS) $(VERIFYFLAGS)

clean:
	rm -rf *.obj *.elf *.hex *.lst *.bin *~
