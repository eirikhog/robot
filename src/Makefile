

CC=avr-gcc
MCU=atmega328p
CFLAGS=-c -g -Os -mmcu=$(MCU) -std=gnu99 -Wall -Werror -Wno-unused-function
OBJDUMP=avr-objdump
AVRSIZE=avr-size

all: robot remote
#	$(AVRSIZE) --mcu=$(MCU) -C robot.elf
#	$(AVRSIZE) --mcu=$(MCU) -C remote.elf

nrf24.o:
	$(CC) nrf24.c -c $(CFLAGS) -o nrf24.o

robot: nrf24.o
	$(CC) robot.c -c $(CFLAGS) -o robot.o
	$(CC) -g -mmcu=$(MCU) -o robot.elf robot.o nrf24.o
	$(OBJDUMP) -h -S robot.elf > robot.lst

remote: nrf24.o
	$(CC) remote.c -c $(CFLAGS) -o remote.o
	$(CC) -g -mmcu=$(MCU) -o remote.elf remote.o nrf24.o
	$(OBJDUMP) -h -S remote.elf > remote.lst

clean:
	rm *.o *.elf *.lst

progrobot: robot
	avrdude -c usbasp -p atmega328p -U flash:w:robot.elf

progremote: remote
	avrdude -c usbasp -p atmega328p -U flash:w:remote.elf


