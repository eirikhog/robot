#ifndef _ROBOT_NRF24_H
#define _ROBOT_NRF24_H

/**
 * Pin configuration:
 * PIND4 = SCK
 * PIND3 = MISO
 * PIND2 = MOSI
 * PINB0 = CSN
 * PIND0 = CE
 * PIND7 = IRQ
 */

// SPI Software functions
#define SPI_SCK_LOW() PORTD &= ~(1<<PD4)
#define SPI_SCK_HIGH() PORTD |= (1<<PD4)
#define SPI_MOSI_LOW() PORTD &= ~(1<<PD2)
#define SPI_MOSI_HIGH() PORTD |= (1<<PD2)
#define SPI_MISO_READ() (PIND & (1<<PD3))

// Software SPI
void spi_init() {
    DDRD |= (1<<PD4)|(1<<PD2);
    DDRD &= ~(1<<PD3);
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
    // TODO: Fix stupid implementation. (Demands compile time constant)
    while (ms--) {
        _delay_ms(1);
    }
}

void nrf24_set_ce(uint8_t state) {
    if (state) {
        PORTB |= (1<<PB7);
    } else {
        PORTB &= ~(1<<PB7);
    }
}

void nrf24_set_csn(uint8_t state) {
    if (state) {
        PORTD |= (1<<PD7);
    } else {
        PORTD &= ~(1<<PD7);
    }
}

#endif

