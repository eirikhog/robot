#ifndef _ROBOT_DEBUG_H
#define _ROBOT_DEBUG_H

#define DEBUG_INIT() robot_debug_init()
#define DEBUG_PRINT(x) robot_debug_send(x)

void robot_debug_init() {
	// Debug output set to UART

    uint16_t baud = 9600;
    uint16_t ubrr = F_CPU/16/baud-1;
    /* Set baud rate */
    UBRR0H = (unsigned char)(ubrr>>8); 
    UBRR0L = (unsigned char)ubrr;
    /* Enable receiver and transmitter */
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    /* Set frame format: 8data, 2stop bit */
    UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void robot_debug_send_byte(uint8_t data) {
    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1<<UDRE0)));
    /* Put data into buffer, sends the data */
    UDR0 = data;
}

void robot_debug_send(char *str) {
    while (*str) {
        robot_debug_send_byte(*str);
        str++;
    }
}

#endif