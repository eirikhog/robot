#ifndef _COMMON_H
#define _COMMON_H

#include <stdint.h>

typedef uint8_t byte;
typedef uint8_t bool;

#define set_bit(reg, n) ((reg) |= (1 << (n)))
#define clear_bit(reg, n) ((reg) &= ~(1 << (n)))
#define bit_set(reg, n) ((reg) & (1 << (n)))

// Note: Not safe in edge cases
// #define abs(x) ((x > 0) ? x : -x)
#define max(x,y) ((x < y) ? y : x)

typedef enum {
    FORWARD,
    BACKWARD
} Direction;


typedef enum {
    RADIO_CMD_RESERVED = 0,
	RADIO_CMD_LIGHT_ON,
	RADIO_CMD_LIGHT_OFF,
    RADIO_CMD_STOP,
    RADIO_CMD_SET_MOTOR,
    RADIO_CMD_COUNT
} RadioCommand;

typedef enum {
    RADIO_NACK = 0,
    RADIO_ACK = 1
} RadioProtocol;

typedef struct {
    RadioCommand cmd;
    uint8_t data[4];
} RemoteCommand;

void init_uart(uint16_t baud, uint32_t freq) {

	uint16_t ubrr = freq/16/baud-1;
	
	/* Set baud rate */
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t)ubrr;

	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);

	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void uart_send(unsigned char data) {
	/* Wait for empty transmit buffer */
	while (!( UCSR0A & (1<<UDRE0)));
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

void uart_send_string(char *str, int length) {
	for (int i = 0; i < length; ++i) {
		uart_send(str[i]);
	}
}

byte uart_recv() {
	while (!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

bool uart_waiting() {
    return (UCSR0A & (1 << RXC0)) != 0;
}

// data must be 4 bytes!
void radio_send(RadioCommand cmd, uint8_t *data) {
    uart_send(cmd);
    uart_send(*data);
    uart_send(*(data + 1));
    uart_send(*(data + 2));
    uart_send(*(data + 3));
}

#endif

