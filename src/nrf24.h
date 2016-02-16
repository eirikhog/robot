#ifndef _NRF24_H
#define _NRF24_H

/**
 * Software driver for the nRF24L01+ radio module.
 *
 * Current features:
 * - Send messages
 * - Receive messages
 *
 * Not implemented yet:
 * - Set transmission rate (always uses 1Mbps)
 * - Multicast
 * - Ack configuration
 * - Asynchronous interface.
 *
 */

#include <stdint.h>

/**
 * Initialize the library.
 */
void nrf24_init();

/**
 * Configure and power up the device in RX mode.
 */
void nrf24_config(uint8_t channel, uint8_t size);

/**
 * Set the receive address, always 5 bytes.
 */
void nrf24_set_rx_addr(uint8_t mac[5]);

/**
 * Set the transmit address, always 5 bytes.
 */
void nrf24_set_tx_addr(uint8_t mac[5]);

/**
 * Send a message (synchronous)
 */
void nrf24_send(void *buffer, uint8_t len);

/**
 * Receive a message. len has to equal packet size.
 */
void nrf24_recv(void *buffer, uint8_t len);

/**
 * Power up the radio.
 */
void nrf24_power_up(void);

/**
 * Power down the radio.
 */
void nrf24_power_down(void);

/**
 * Returns non-zero if there is data received.
 */
uint8_t nrf24_has_data(void);

/**
 * Returns true if a message is currently transmitting.
 */
uint8_t nrf24_is_sending(void);

/**
 * Get the number of retransmissions of last sent message.
 */
uint8_t nrf24_retransmissions(void);

/**
 * Returns the total number of lost packets, after max retries failed.
 */
uint8_t nrf24_lost_packets(void);

// User implementations
extern uint8_t spi_transfer(uint8_t value);
extern void nrf24_set_ce(uint8_t state);
extern void nrf24_set_csn(uint8_t state);
extern void delay(uint8_t ms);

#endif
