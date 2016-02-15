# Robot

This is the software for my little remote controlled robot (RC tank, more or less).
I started this project because I wanted to refresh some embedded skills, and
to finally do something about my plans for an RC toy.

The code currently works fine, but it probably is not much use without the hardware.

[![Build Status](http://eirikhog.com:8080/job/Robot/badge/icon)](http://eirikhog.com:8080/job/Robot/)

## Hardware

I chose to use the ATmega328p for both part of this project due to cost and availability.
The same chips is used on both the car and the remote, both run with the internal oscillator
at 8 MHz.

### Remote

The base for the remote is a prototype board, with an ATmega328p as the controller.
It also contains:
- An LCD screen (Nokia 5110) via SPI.
- A PS2 Analog joystick via ADC
- 4 buttons connected to GPIO
- LED power indicator
- nRF42L01+ transceiver module via Software SPI

### Robot

The robot stared out on a homebuilt Lego platform, but I later decided to buy a
finished robot base set which made for a more solid base.
Features:
- Ultrasouns range detector, via GPIO.
- LED controllable with GPIO pin.
- LED for power indication.
- Motor driver, controlled with 2 PWMs.
- nRF24L01+ tranceiver module via Software SPI

One notable issue with the design is that the motor driver I had required 5V,
while the radio needed 3V. Fortunately the nRF24L01+ tolerates 5V logic, but I still
needed to add another voltage regulator for 3.3V supply voltage.

## Firmware

The code is written for and compiled with avr-gcc. There are build files for both
Windows (build.bat) and *nix (Makefile). Both should work, provided avr-gcc is in
path.

