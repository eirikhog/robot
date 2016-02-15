#define F_CPU 8000000
#define DEBUG

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdio.h>

#include "common.h"
#include "robot_debug.h"
#include "robot_nrf24.h"
#include "nrf24.h"

typedef enum {
    MOTOR_LEFT,
    MOTOR_RIGHT
} Motor;

typedef struct {
    uint32_t counter;
    bool working;
} Range;

static Range range_state;

// Interrupt for the range sensor response.
ISR(PCINT1_vect) {
    bool rangeOn = PINC & _BV(PINC2);
    if (rangeOn && !range_state.working) {
       // Start timer...
       range_state.counter = 0;
       TCNT2 = 0;
       TCCR2A = 0;
       TCCR2B = _BV(CS20);
       TIMSK2 = _BV(TOIE2);
       range_state.working = 1;
    } // Else: Ignore this interrupt

    if (!rangeOn && range_state.working) {
        // Stop timer...
        range_state.counter += TCNT2;
        TCCR2B = 0;
        range_state.working = 0;
    }
}

// Interrupt for range counter overflow.
ISR(TIMER2_OVF_vect) {
    range_state.counter += 255;
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
    
    // Wait for the motors to settle before returning
    _delay_ms(30);
}

void motor_set_left(Direction d, uint8_t speed) {
    switch (d) {
        case BACKWARD:{
            OCR0A = speed;
            OCR0B = 0;
        } break;
        case FORWARD:{
            OCR0A = 0;
            OCR0B = speed;
        } break;            
    }
}

void motor_set_right(Direction d, uint16_t speed) {
    switch (d) {
        case BACKWARD:{
            OCR1A = 0;
            OCR1B = speed;
        } break;
        case FORWARD:{
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
    DEBUG_INIT();

    // Initialize LED
    set_bit(DDRC, 0);
    set_bit(PORTC, 0);
    
    init_motor();
    _delay_ms(100);

	uint8_t source_addr[5] = { 0x0A, 0x0A, 0x0A, 0x0A, 0x0A };
    uint8_t dest_addr[5] = { 0x1B, 0x1B, 0x1B, 0x1B, 0x1B };
    spi_init();
    nrf24_init();
    nrf24_config(47, sizeof(RemoteCommand));
    nrf24_set_rx_addr(source_addr);
    nrf24_set_tx_addr(dest_addr);

    init_range();

    sei();

    // Variables for tracking free range.
    #define STOP_SAMPLES 8
    const uint8_t stopThreshold = STOP_SAMPLES / 4;
    bool stopSamples[STOP_SAMPLES];
    uint8_t stopCounter = 0;

    DEBUG_PRINT("Starting program main loop...\n");
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

        uint8_t blocked = 0;
        if (stops >= stopThreshold) {
            //clear_bit(PORTC, 0);
            blocked = 1;
            stop_motors();
        } else {
            //set_bit(PORTC, 0);
            blocked = 0;
        }


        if (nrf24_has_data()) {
            RemoteCommand newCommand;
            nrf24_recv(&newCommand, sizeof(RemoteCommand));

            switch (newCommand.cmd) {
                case RADIO_CMD_LIGHT_ON:{
                    DEBUG_PRINT("RECV CMD: RADIO_CMD_LIGHT_ON\n");
                    set_bit(PORTC, 0);
                } break;
                case RADIO_CMD_LIGHT_OFF:{
                    DEBUG_PRINT("RECV CMD: RADIO_CMD_LIGHT_OFF\n");
                    clear_bit(PORTC, 0);
                } break;
                case RADIO_CMD_STOP:{
                    DEBUG_PRINT("RECV CMD: RADIO_CMD_STOP\n");
                    stop_motors();
                } break;
                case RADIO_CMD_SET_MOTOR:{
                    DEBUG_PRINT("RECV CMD: RADIO_CMD_SET_MOTOR\n");
                    char debug_buffer[64];
                    sprintf(debug_buffer, "%d %d %d %d\n", newCommand.data[0], newCommand.data[1],
                            newCommand.data[2], newCommand.data[3]);
                    DEBUG_PRINT(debug_buffer);
                    Direction left_dir = newCommand.data[0];
                    uint8_t left_speed = newCommand.data[1];
                    Direction right_dir = newCommand.data[2];
                    uint8_t right_speed = newCommand.data[3];

                    // Only allow backing up if we're blocked.
                    if (!blocked || (left_dir == BACKWARD && right_dir == BACKWARD)) {
                        motor_set_left(left_dir, left_speed);
                        motor_set_right(right_dir, right_speed);
                    }
                } break;
                default:{
                    asm("nop"::);
                }
            }
        }
    }
}


