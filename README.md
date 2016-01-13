# Robot

This is the firmware for a simple robot done as a hobbyist project.

The code is work in progress.

## Hardware

I chose to use the ATmega328p for both part of this project due to cost and availability.
It has one UART which is used for wireless communication, an ADC which is useful for sensors
and analog input, and PWM which is used for motor control.

TODO: Describe hardware setup (diagram?).

## Firmware

The code is written for and compiled with avr-gcc. It started off in Atmel Studio 7, but
I wanted to program the device from OS X and Linux as well and it was a pain to bring
the entire Atmel Studio to a new computer every time.

To build the code, make sure avr-gcc is in PATH and run build.bat from the src folder.

