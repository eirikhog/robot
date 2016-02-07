#include "nrf24.h"

#include <avr/io.h>

/**
 * Pin configuration:
 * PINB5 = SCK
 * PINB4 = MISO
 * PINB3 = MOSI
 * PINB0 = CS
 * PIND7 = IRQ
 */

static void
spi_transmit(char data) {
    SPDR = data;
    while(!(SPSR & (1<<SPIF)));
}

static void
chip_select() {
    PORTD |= (1<<PD7);
}

static void
chip_deselect() {
    PORTD &= ~(1<<PD7);
}

void nrf24_init() {
    DDRD |= (1<<PD7);
    DDRB |= (1<<PB0);

    // Setup SPI
    // Set output: MOSI and SCK
    DDRB = (1<<PB3)|(1<<PB5);
    // SPI Enable, Master mode, f_osc / 16
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
    // TODO: Clock mode?
}

void nrf24_config(uint16_t channel, uint16_t size) {
    chip_select();

    // Dummy:
    spi_transmit('Q');

    chip_deselect();
}


