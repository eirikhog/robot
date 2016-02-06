

CC=avr-gcc
MCU=atmega328p
CFLAGS=-c -g -Os -mmcu=$(MCU) -std=gnu99 -Wall -Werror
OBJDUMP=avr-objdump
AVRSIZE=avr-size

all: robot remote
#	$(AVRSIZE) --mcu=$(MCU) -C robot.elf
#	$(AVRSIZE) --mcu=$(MCU) -C remote.elf

robot:
	$(CC) robot.c -c $(CFLAGS) -o robot.o
	$(CC) -g -mmcu=$(MCU) -o robot.elf robot.o
	$(OBJDUMP) -h -S robot.elf > robot.lst

remote:
	$(CC) remote.c -c $(CFLAGS) -o remote.o
	$(CC) -g -mmcu=$(MCU) -o remote.elf remote.o
	$(OBJDUMP) -h -S remote.elf > remote.lst

clean:
	rm *.o *.elf *.lst

program: all
	avrdude -c usbasp -p atmega328p -U flash:w:robot.elf

