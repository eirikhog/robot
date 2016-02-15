#ifndef _NRF24_H
#define _NRF24_H

#include <stdint.h>

/**
 * Required defines:
 *
 * PORT_CE
 * PIN_CE
 * PORT_CSN
 * PIN_CSN
 * PORT_MISO
 * PIN_MISO
 * PORT_MOSI
 * PIN_MOSI
 * PORT_SCK
 * PIN_SCK
 */

void nrf24_init();
void nrf24_config(uint8_t channel, uint8_t size);
void nrf24_set_rx_addr(uint8_t mac[5]);
void nrf24_set_tx_addr(uint8_t mac[5]);
void nrf24_send(void *buffer, uint8_t len);
void nrf24_recv(void *buffer, uint8_t len);
uint8_t nrf24_status(void);
void nrf24_power_up(void);
void nrf24_power_down(void);
uint8_t nrf24_has_data(void);
uint8_t nrf24_status(void);
uint8_t nrf24_is_sending(void);
uint8_t nrf24_last(void);
uint8_t nrf24_read_config(void);
uint8_t nrf24_read_register(uint8_t addr);
uint8_t nrf24_retransmissions(void);
uint8_t nrf24_lost_packets(void);

// User implementations
extern uint8_t spi_transfer(uint8_t value);
extern void nrf24_set_ce(uint8_t state);
extern void nrf24_set_csn(uint8_t state);
extern void delay(uint8_t ms);

#endif
