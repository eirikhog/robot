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

#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define NOP        0xFF

static uint8_t
spi_transfer(uint8_t data) {
    SPDR = data;
    while(!(SPSR & (1<<SPIF)));
    return SPDR;
}

static void
spi_transfer_many(uint8_t *ibuffer, uint8_t *obuffer,
                  uint8_t len) {
    for (uint8_t i = 0; i < len; ++i) {
        obuffer[i] = spi_transfer(ibuffer[i]);
    }
}

static void
chip_select() {
    PORTD &= ~(1<<PD7);
}

static void
chip_deselect() {
    PORTD |= (1<<PD7);
}

static void
nrf24_read_register(uint8_t addr, uint8_t *buffer, uint8_t len) {
    chip_select();
    spi_transfer(R_REGISTER | (0x1F & addr)); 
    spi_transfer_many(buffer, buffer, len);
    chip_deselect();
}

void nrf24_config_reg(uint8_t reg, uint8_t value) {
    chip_select();
    // TODO: W_REGISTER
    chip_deselect();
}

void nrf24_init() {
    DDRD |= (1<<PD7);
    DDRB |= (1<<PB0);

    // Set output: MOSI and SCK
    DDRB = (1<<PB3)|(1<<PB5);
    // SPI Enable, Master mode, f_osc / 16
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);

    uint8_t buff = 0;
    nrf24_read_register(0, &buff, 1);
}

static uint16_t packet_size;

void nrf24_config(uint16_t channel, uint16_t size) {
    chip_select();

    packet_size = size;


    chip_deselect();
}



