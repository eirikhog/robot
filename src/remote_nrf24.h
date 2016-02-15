#ifndef _REMOTE_NRF24_H
#define _REMOTE_NRF24_H

#include <stdint.h>
#include <util/delay.h>

 /**
 * Radio: (Software SPI)
 * PD0: MOSI
 * PD1: MISO
 * PD2: CE
 * PD3: CSN
 * PD4: SCK
 */

// SPI Software functions
#define SPI_SCK_LOW() PORTD &= ~(1<<PD4)
#define SPI_SCK_HIGH() PORTD |= (1<<PD4)
#define SPI_MOSI_LOW() PORTD &= ~(1<<PD0)
#define SPI_MOSI_HIGH() PORTD |= (1<<PD0)
#define SPI_MISO_READ() (PIND & (1<<PD1))

// Software SPI
void spi_init() {
    DDRD |= (1<<PD4)|(1<<PD0);
    DDRD &= ~(1<<PD1);
}

uint8_t spi_transfer(uint8_t data) {
    uint8_t in = 0;

    SPI_SCK_LOW();
    for (uint8_t i = 0; i < 8; ++i) {
        // Send MSB first
        if (data & (1 << (7 - i))) {
            SPI_MOSI_HIGH();
        } else {
            SPI_MOSI_LOW();
        }

        SPI_SCK_HIGH();

        uint8_t inPin = SPI_MISO_READ();
        if (inPin) {
            in |= (1 << (7 - i));
        }

        SPI_SCK_LOW();
    }

    return in;
}

void delay(uint8_t ms) {
    while(ms--) {
        _delay_ms(1);
    }
}

void nrf24_set_ce(uint8_t state) {
    if (state) {
        PORTD |= _BV(PD2);
    } else {
        PORTD &= ~_BV(PD2);
    }
}

void nrf24_set_csn(uint8_t state) {
    if (state) {
        PORTD |= _BV(PD3);
    } else {
        PORTD &= ~_BV(PD3);
    }
}

#endif

