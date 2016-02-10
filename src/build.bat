@echo off

set cc=avr-gcc
set mmcu=atmega328p
set cflags=-c -g -Os -mmcu=%mmcu% -std=gnu99 -Wall -Werror
set outputDir=..\build\

if not exist ..\build mkdir ..\build
pushd ..\build

:BUILD
echo Building robot...
avr-gcc ..\src\robot.c -c %cflags% -o robot.o 
avr-gcc ..\src\nrf24.c -c %cflags% -o nrf24.o
avr-gcc -g -mmcu=%mmcu% -o robot.elf robot.o nrf24.o
avr-objdump -h -S robot.elf > robot.lst
avr-size --mcu=%mmcu% robot.elf

echo.
echo Building remote...
avr-gcc %cflags% -o remote.o ..\src\remote.c
avr-gcc -g -mmcu=%mmcu% -o remote.elf remote.o
avr-objdump -h -S remote.elf > remote.lst
avr-size --mcu=%mmcu% remote.elf

popd
