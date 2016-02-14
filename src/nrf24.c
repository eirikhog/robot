#include "nrf24.h"

#include <avr/io.h>
#include <stdio.h>


/* SPI Commands */
#define R_REGISTER  0x00
#define W_REGISTER  0x20
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX    0xE1
#define FLUSH_RX    0xE2
#define REUSE_TX_PL 0xE3
#define ACTIVATE    0x50
#define R_RX_PL_WID 0x60
#define W_ACK_PAYLOAD 0xA8 // Last 3 bits determines pipe
#define W_TX_PAYLOAD_NOACK 0xB0
#define NOP         0xFF

/* Register mask for *_REGISTER */
#define REGISTER_MASK 0x1F

/* Register map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

/* Config bits */
#define MASK_RX_DR 6
#define MASK_TX_DS 5
#define MASK_MAX_RT 4
#define EN_CRC 3
#define CRCO 2
#define PWR_UP 1
#define PRIM_RX 0

/* EN_RXADDR bits */
#define ERX_P0 0
#define ERX_P1 1
#define ERX_P2 2
#define ERX_P3 3
#define ERX_P4 4
#define ERX_P5 5

/* STATUS bits */
#define RX_DR 6
#define TX_DS 5
#define MAX_RT 4
#define TX_FULL 0 

/* EN_AA bits */
#define ENAA_P5 5
#define ENAA_P4 4
#define ENAA_P3 3
#define ENAA_P2 2
#define ENAA_P1 1
#define ENAA_P0 0

// FIFO Status register
#define FIFO_TX_REUSE 6
#define FIFO_TX_FULL  5
#define FIFO_TX_EMPTY 4
#define FIFO_RX_FULL  1
#define FIFO_RX_EMPTY 0

static void
spi_transfer_many(uint8_t *ibuffer, uint8_t *obuffer,
                  uint8_t len) {
    for (uint8_t i = 0; i < len; ++i) {
        obuffer[i] = spi_transfer(ibuffer[i]);
    }
}

#define CSN_LOW() PORTD &= ~(1<<PD7)
#define CSN_HIGH() PORTD |= (1<<PD7)
#define CE_LOW() PORTB &= ~(1<<PB7)
#define CE_HIGH() PORTB |= (1<<PB7)

static void
nrf24_read_register_len(uint8_t addr, uint8_t *buffer, uint8_t len) {
    CSN_LOW();
    spi_transfer(R_REGISTER | (0x1F & addr)); 
    spi_transfer_many(buffer, buffer, len);
    CSN_HIGH();
}

static uint8_t
nrf24_read_register(uint8_t addr) {
    uint8_t result = 0;
    nrf24_read_register_len(addr, &result, 1);
    return result;
}

static void
nrf24_write_register_len(uint8_t addr, uint8_t *buffer, uint8_t len) {
    CSN_LOW();
    spi_transfer(W_REGISTER | (REGISTER_MASK & addr));
    spi_transfer_many(buffer, buffer, len);
    CSN_HIGH();
}

void nrf24_write_register(uint8_t reg, uint8_t value) {
    CSN_LOW();

    spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    spi_transfer(value);

    CSN_HIGH();
}

uint8_t nrf24_read_config(uint8_t reg) {
    CSN_LOW();

    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    uint8_t result = spi_transfer(NOP);

    CSN_HIGH();

    return result;
}

void nrf24_init() {
    // CSN output
    DDRD |= (1<<PD7);
    // CE output
    DDRB |= _BV(PB7);

    // Set output: MOSI and SCK
    DDRB |= (1<<PB3)|(1<<PB5);
    // SPI Enable, Master mode, f_osc / 128
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);

    CE_LOW();
    CSN_HIGH();
}

static uint8_t payload_size;

void nrf24_config(uint8_t channel, uint8_t size) {
    payload_size = size;

    // Wait for the radio to settle in defined state.
    delay(5);

    // Enable CRC
    nrf24_write_register(CONFIG, (1 << EN_CRC) | (0 << CRCO));

    nrf24_write_register(RF_CH, channel);

    nrf24_write_register(RX_PW_P0, 0);
    nrf24_write_register(RX_PW_P1, size & 0x1F);

    // Set Air Data Rate = 1mbit, RF output power 0db
    nrf24_write_register(RF_SETUP, 0x06);

    // Auto Acknowledgment
    nrf24_write_register(EN_AA, (1 << ENAA_P0) | (1 << ENAA_P1));

    // Enable RX addr
    nrf24_write_register(EN_RXADDR, (1 << ERX_P0) | (1 << ERX_P1));
    
    // Auto retransmit delay, 500 us, 15 retransmit
    nrf24_write_register(SETUP_RETR, 0x1F);

    // No dynamic length
    nrf24_write_register(DYNPD, 0);

    // Reset current status
    nrf24_write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    // Flush buffers
    CSN_LOW();
    spi_transfer(FLUSH_TX);
    spi_transfer(FLUSH_RX);
    CSN_HIGH();

    nrf24_power_up();

    uint8_t config = nrf24_read_register(CONFIG);
    nrf24_write_register(CONFIG, config | _BV(PRIM_RX));

    CE_HIGH();
}

void nrf24_set_rx_addr(uint8_t addr[5]) {
    nrf24_write_register_len(RX_ADDR_P1, addr, 5);
}

void nrf24_set_tx_addr(uint8_t addr[5]) {
    nrf24_write_register_len(RX_ADDR_P0, addr, 5);
    nrf24_write_register_len(TX_ADDR, addr, 5);
	nrf24_write_register(RX_PW_P0, payload_size);
}

void nrf24_power_up(void) {
	
    uint8_t config = nrf24_read_register(CONFIG);
    if (!(config & (1 << PWR_UP))) {
        nrf24_write_register(CONFIG, config | _BV(PWR_UP));

        // Wait for transition state to complete.
        delay(5);
    }
}

void nrf24_power_down(void) {
    CE_LOW();
    uint8_t config = nrf24_read_register(CONFIG);
    nrf24_write_register(CONFIG, config & ~_BV(PWR_UP));
}

uint8_t nrf24_rx_fifo_empty(void) {
    uint8_t fifoStatus = nrf24_read_register(FIFO_STATUS);
    return (fifoStatus & (1 << FIFO_RX_EMPTY));
}

uint8_t nrf24_has_data(void) {
    uint8_t status = nrf24_status();
    if (status & (1 << RX_DR)) {
        return 1;
    }
    uint8_t fifoEmpty = nrf24_rx_fifo_empty();
    return !fifoEmpty;
}

void nrf24_recv(void *buffer, uint8_t len) {
	uint8_t *dest = (uint8_t *)buffer;

    CSN_LOW();
    spi_transfer(R_RX_PAYLOAD);
	while (len--) {
		*dest++ = spi_transfer(0xFF);
	}
    CSN_HIGH();

	CSN_LOW();
	spi_transfer(FLUSH_RX);
	CSN_HIGH();

    nrf24_write_register(STATUS, _BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS));
}

uint8_t nrf24_status(void) {
    CSN_LOW();
    uint8_t status = spi_transfer(NOP);
    CSN_HIGH();

    return status;
}
