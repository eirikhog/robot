#ifndef _COMMON_H
#define _COMMON_H

#include <stdint.h>

typedef uint8_t byte;
typedef uint8_t bool;

#define set_bit(reg, n) ((reg) |= (1 << (n)))
#define clear_bit(reg, n) ((reg) &= ~(1 << (n)))
#define bit_set(reg, n) ((reg) & (1 << (n)))

// Note: Not safe in edge cases
#define abs(x) ((x > 0) ? x : -x);
#define max(x,y) ((x < y) ? y : x);

typedef enum {
	RADIO_CMD_LIGHT_ON = 0x40,
	RADIO_CMD_LIGHT_OFF = 0x41,
    RADIO_CMD_STOP = 0x80,
    RADIO_CMD_MOTOR_LEFT_FORWARD,
    RADIO_CMD_MOTOR_LEFT_BACKWARD,
    RADIO_CMD_MOTOR_RIGHT_FORWARD,
    RADIO_CMD_MOTOR_RIGHT_BACKWARD
} RadioCommand;

typedef enum {
    RADIO_NACK = 0,
    RADIO_ACK = 1
} RadioProtocol;

typedef struct {
    RadioCommand cmd;
    uint8_t data;
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

void radio_send(RadioCommand cmd, uint8_t data) {
    // Send command
    uart_send(cmd);
    uart_send(data);

    // TODO: Check for ACK
}

RadioCommand radio_get(uint8_t *data) {
    uint8_t cmd = (RadioCommand)uart_recv();
    *data = (uint8_t)uart_recv();

    // TODO: Send ACK

    return cmd;
}

#endif

