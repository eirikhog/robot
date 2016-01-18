#define F_CPU 8000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "common.h"

typedef struct {
    uint32_t counter;
    bool working;
} Range;

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

static Range range_state;

void toggle_led(void);

static uint8_t recv_buffer[2];
static uint8_t recv_buffer_count;

ISR(USART_RX_vect) {
    if (recv_buffer_count < 2) {
        recv_buffer[recv_buffer_count] = UDR0;
        ++recv_buffer_count;
    }
}

ISR(PCINT1_vect) {
    //toggle_led();
    bool rangeOn = PINC & (1 << PINC2);
    if (rangeOn && !range_state.working) {
       // Start timer...
       range_state.counter = 0;
       TCNT2 = 0;
       TCCR2A = 0;
       TCCR2B = (1 << CS20);
       TIMSK2 = (1 << TOIE2);
       range_state.working = 1;
    } // Else: Ignore this interrupt

    if (!rangeOn && range_state.working) {
        // Stop timer...
        range_state.counter += TCNT2;
        TCCR2B = 0;
        range_state.working = 0;
    }
}

ISR(TIMER2_OVF_vect) {
    range_state.counter += 255;
    //toggle_led(); 
}

void init_motor() {
    // Phase correct PWM OC0A/B
    TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) | (1<<WGM00);
    TCCR0B = (0<<WGM02) | (1<<CS00);
    OCR0A = 0;
    OCR0B = 0;
    DDRD |= (1<<PIND5) | (1<<PIND6);

    // Phase correct PWM OC1A/B
    TCCR1A = (1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (1<<WGM10);
    TCCR1B = (0<<WGM12) | (1 << CS00);
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

void motor_set_right(Direction d, uint8_t speed) {
    switch (d) {
        case FORWARD:{
            OCR0A = speed;
            OCR0B = 0;
        } break;
        case BACKWARD:{
            OCR0A = 0;
            OCR0B = speed;
        } break;            
    }
}

void motor_set_left(Direction d, uint16_t speed) {
    switch (d) {
        case FORWARD:{
            OCR1A = 0;
            OCR1B = speed;
        } break;
        case BACKWARD:{
            OCR1A = speed;
            OCR1B = 0;
        } break;
    }

}

void init_range() {
    set_bit(DDRC, 1);
    clear_bit(PORTC, 1);

    clear_bit(DDRC, 2);
    clear_bit(PORTC, 2);

    range_state.counter = 0;
    range_state.working = 0;

    // Enable interrupts
    PCMSK1 |= (1<<PCINT10);
    EICRA |= (1<<ISC11) | (1<<ISC10);
    EIMSK |= (1<<INT1);
    PCICR |= (1<<PCIE1);
}

void range_check() {
    set_bit(PORTC, 1);
    _delay_us(14);
    clear_bit(PORTC, 1);
}

void toggle_led() {
    if (PORTC & 1) {
        clear_bit(PORTC, 0);
    } else {
        set_bit(PORTC, 0);
    }
}

int main(void) {

    // Initialize LED
    set_bit(DDRC, 0);
    clear_bit(PORTC, 0);

    init_uart(9600, F_CPU);
    init_motor();
    init_range();

    UCSR0B |= (1 << RXCIE0);

    recv_buffer_count = 0;

    sei();

    RobotState state;
    state = STATE_STOPPED;

    #define STOP_SAMPLES 8 
    const uint8_t stopThreshold = STOP_SAMPLES / 3;
    bool stopSamples[STOP_SAMPLES];
    uint8_t stopCounter = 0;

    while (1) {
        range_check();
        uint8_t waitTime = 0;
        uint8_t maxWait = 200;
        while (range_state.working && waitTime < maxWait) {
            _delay_ms(10);
            waitTime += 10;
        }
        if (range_state.counter < 0x1FFF && waitTime < maxWait) {
            stopSamples[stopCounter % STOP_SAMPLES] = 1;
            //
        } else {
            stopSamples[stopCounter % STOP_SAMPLES] = 0;
        }

        int8_t stops = 0;
        stopCounter++;
        for (int i = 0; i < STOP_SAMPLES; ++i) {
            if (stopSamples[i])
                stops++;
        }

        if (stops >= stopThreshold) {
            if (state != STATE_STOPPED && state != STATE_BACKWARD) {
                stop_motors();
                state = STATE_STOPPED;
            }
            clear_bit(PORTC, 0);
        } else {
            set_bit(PORTC, 0);
        }

        // Critical section
        bool hasCommand;
        RadioCommand cmd = 0;
        uint8_t data = 0;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            if (recv_buffer_count == 2) {
                cmd = (RadioCommand)recv_buffer[0];
                data = recv_buffer[1];
                recv_buffer_count = 0;
                hasCommand = 1;
            }
        }
        
        if (hasCommand) {
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
                case RADIO_CMD_MOTOR_LEFT_FORWARD:{
                    motor_set_left(FORWARD, data);
                } break;
                case RADIO_CMD_MOTOR_LEFT_BACKWARD:{
                    motor_set_right(BACKWARD, data);
                } break;
                case RADIO_CMD_MOTOR_RIGHT_FORWARD:{
                    motor_set_right(FORWARD, data);
                } break;
                case RADIO_CMD_MOTOR_RIGHT_BACKWARD:{
                    motor_set_right(BACKWARD, data);
                } break;
                default:{
                    uart_send(RADIO_NACK);
                }
            }
        }
    }
}


