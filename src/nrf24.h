#ifndef _NRF24_H
#define _NRF24_H

#include <stdint.h>

void nrf24_init();
void nrf24_config(uint8_t channel, uint8_t size);
void nrf24_set_rx_addr(uint8_t mac[5]);
void nrf24_set_tx_addr(uint8_t mac[5]);
void nrf24_send(uint8_t *buffer);
void nrf24_recv(uint8_t *buffer, uint8_t len);
uint8_t nrf24_status(void);
void nrf24_power_up(void);
void nrf24_power_down(void);
uint8_t nrf24_has_data(void);
uint8_t nrf24_status(void);

#endif
