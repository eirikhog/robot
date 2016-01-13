#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>

#include "common.h"


typedef enum {
    MOTOR_LEFT,
    MOTOR_RIGHT
} Motor;

typedef enum {
    FORWARD,
    BACKWARD
} Direction;

typedef enum {
    STATE_STOPPED,
    STATE_FORWARD,
    STATE_BACKWARD,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT
} RobotState;

void init_motor() {
    // Phase correct PWM OC0A/B
    TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) | (1<<WGM00);
    TCCR0B = (1<<WGM02) | (1<<CS00);
    OCR0A = 0;
    OCR0B = 0;
    DDRD |= (1<<PIND5)|(1<<PIND6);

    // Phase correct PWM OC2A/B
    TCCR1A = (1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (1<<WGM10);
    TCCR1B = (1<<WGM12) | (1 << CS00);
    OCR1A = 0;
    OCR1B = 0;
    DDRB |= (1<<PINB2) | (1<<PINB1);
}

void stop_motors() {
    OCR0A = 0;
    OCR0B = 0;
    OCR1A = 0;
    OCR1B = 0;
    
    _delay_ms(500);
}

void start_motor(Motor m, Direction d) {
    // TODO: OCR1 is a 16-bit register, and we need to make sure that the timing
    // of this register matches that of OCR0 so that the motors behave similar.
    volatile uint8_t* output_a;
    volatile uint8_t* output_b;
    byte shift = 0;
    if (m == MOTOR_LEFT) {
        output_a = (uint8_t*)&OCR1B;
        output_b = (uint8_t*)&OCR1A;
        shift = 0;
    } else {
        output_a = &OCR0A;
        output_b = &OCR0B;
        shift = 1;
    }
    
    switch (d) {
        case FORWARD:{
            *output_a = 0xC0 >> shift;
            *output_b = 0x00;
        } break;
        case BACKWARD:{
            *output_a = 0x00;
            *output_b = 0xC0 >> shift;
        } break;            
    }
}

int main(void) {

	// Initialize LED
	set_bit(DDRC, 0);
    set_bit(PORTC, 0);

    init_uart(9600, F_CPU);
    init_motor();

    RobotState state;
    state = STATE_STOPPED;
	
    while (1) {
	    RadioCommand cmd = (RadioCommand)uart_recv();
		switch (cmd) {
			case RADIO_CMD_LIGHT_ON:{
				set_bit(PORTC, 0);
				uart_send(RADIO_ACK);
			} break;
			case RADIO_CMD_LIGHT_OFF:{
				clear_bit(PORTC, 0);
                uart_send(RADIO_ACK);
			} break;
            case RADIO_CMD_STOP:{
                stop_motors();
                state = STATE_STOPPED;
                uart_send(RADIO_ACK);
            } break;
            case RADIO_CMD_FORWARD:{
                if (state != STATE_STOPPED) {
                    stop_motors();
                }
                start_motor(MOTOR_LEFT, FORWARD);
                start_motor(MOTOR_RIGHT, FORWARD);
                state = STATE_FORWARD;
                uart_send(RADIO_ACK);
            } break;
            case RADIO_CMD_BACKWARD:{
                if (state != STATE_STOPPED) {
                    stop_motors();
                }
                start_motor(MOTOR_LEFT, BACKWARD);
                start_motor(MOTOR_RIGHT, BACKWARD);
                state = STATE_BACKWARD;
                uart_send(RADIO_ACK);
            } break;
            default:{
                uart_send(RADIO_NACK);
            }
		}
    }
}


