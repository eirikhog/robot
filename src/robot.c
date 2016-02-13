#define F_CPU 8000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "common.h"
#include "robot_debug.h"
#include "nrf24.h"
// TODO: Move out
#include "nrf24.c"

typedef struct {
    uint32_t counter;
    bool working;
} Range;

typedef enum {
    MOTOR_LEFT,
    MOTOR_RIGHT
} Motor;

static Range range_state;

void toggle_led(void);

// TODO: Move to buffer, or better struct.
static volatile bool HasCommand;
static volatile RadioCommand Command;
static volatile uint8_t CommandData[4];

#define UART_BUFFER_COUNT 64 
static uint8_t UartRecvBuffer[UART_BUFFER_COUNT];

typedef struct {
    uint8_t *data;
    uint16_t start;
    uint16_t capacity;
    uint8_t count;
} CircleBuffer;

static volatile CircleBuffer UartBuffer;

inline bool buffer_empty(volatile CircleBuffer *buffer) {
    return buffer->count == 0;
}

inline bool buffer_full(volatile CircleBuffer *buffer) {
    return buffer->count == buffer->capacity;
}

inline uint8_t buffer_read(volatile CircleBuffer *buffer) {
    uint8_t data = *(buffer->data + buffer->start);
    uint16_t newStart = buffer->start + 1;
    if (newStart == buffer->capacity) {
        buffer->start = 0;
    } else {
        buffer->start = newStart;
    }

    buffer->count--;
    return data;
}

// TODO: Make safe for concurrent usage.
// Currently fine, only used from ISR
inline void buffer_write(volatile CircleBuffer *buffer, uint8_t data) {
    if (!buffer_full(buffer)) {
        uint16_t head = (buffer->start + buffer->count) % buffer->capacity;
        *(buffer->data + head) = data;
        buffer->count++;
    }
}

ISR(USART_RX_vect) {
    // TODO: Buffering
    clear_bit(PORTC, 0);
    uint8_t packed = UDR0;
    buffer_write(&UartBuffer, packed);
}
        
bool poll_buffer(volatile CircleBuffer *buffer, volatile RadioCommand *cmd, volatile uint8_t *data) {
    // Read buffer and try to assemble a command.
    if (HasCommand) {
        // A command is already loaded, but not processed.
        return 1;
    }

    if (buffer->count >= 5) {
        uint8_t cmd_int = buffer_read(buffer);
        if (cmd_int > RADIO_CMD_COUNT || cmd_int == RADIO_CMD_RESERVED) {
            // Invalid command, we are probably at the wrong frame...
            // Ignore it, and try again next time. TODO: Try again now!
            return 0;
        }

        HasCommand = 1;
        *cmd = (RadioCommand)cmd_int;
        for (int i = 0; i < 4; ++i) {
            data[i] = buffer_read(buffer);
        }
        return 1;
    }

    return 0;
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

void motor_set_left(Direction d, uint16_t speed) {
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

static uint8_t source_addr[5] = { 0x0A, 0x0A, 0x0A, 0x0A, 0x0A };
static uint8_t dest_addr[5] = { 0x1B, 0x1B, 0x1B, 0x1B, 0x1B };

int main(void) {

    DEBUG_INIT();

    // Initialize LED
    set_bit(DDRC, 0);
    set_bit(PORTC, 0);
    UartBuffer.data = UartRecvBuffer;
    UartBuffer.start = 0;
    UartBuffer.capacity = UART_BUFFER_COUNT;

    init_motor();
    _delay_ms(100);

    //motor_set_left(FORWARD, 0xFF);
    //motor_set_right(FORWARD, 0xFF);

    DEBUG_PRINT("NRF24 Init...\n");
    //nrf24_init();
    DEBUG_PRINT("NRF24 Init done!\n");

    nrf24_config(7, 16);
    nrf24_set_rx_addr(source_addr);
    nrf24_set_tx_addr(dest_addr);

    DEBUG_PRINT("Init range...\n");
    init_range();
    sei();
    HasCommand = 0;

    #define STOP_SAMPLES 8
    const uint8_t stopThreshold = STOP_SAMPLES / 4;
    bool stopSamples[STOP_SAMPLES];
    uint8_t stopCounter = 0;

    DEBUG_PRINT("Starting program main loop...\n");
    while (1) {
        if (nrf24_has_data()) {
            uint8_t dummy = 0;
            nrf24_recv(&dummy, 1);
            DEBUG_PRINT("Got data!\n");
        }

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
            clear_bit(PORTC, 0);
        } else {
            set_bit(PORTC, 0);
        }
    }
#if 0
        while (poll_buffer(&UartBuffer, &Command, CommandData)) {
            set_bit(PORTC, 0);
            switch (Command) {
                case RADIO_CMD_LIGHT_ON:{
                    set_bit(PORTC, 0);
                } break;
                case RADIO_CMD_LIGHT_OFF:{
                    clear_bit(PORTC, 0);
                } break;
                case RADIO_CMD_STOP:{
                    stop_motors();
                } break;
                case RADIO_CMD_SET_MOTOR:{
                    Direction left_dir = CommandData[0];
                    uint8_t left_speed = CommandData[1];
                    Direction right_dir = CommandData[2];
                    uint8_t right_speed = CommandData[3];

                    motor_set_left(left_dir, left_speed);
                    motor_set_right(right_dir, right_speed);
                } break;
                case RADIO_CMD_MOTOR_LEFT_FORWARD:{
                    motor_set_left(FORWARD, data);
                } break;
                case RADIO_CMD_MOTOR_LEFT_BACKWARD:{
                    motor_set_left(BACKWARD, data);
                } break;
                case RADIO_CMD_MOTOR_RIGHT_FORWARD:{
                    motor_set_right(FORWARD, data);
                } break;
                case RADIO_CMD_MOTOR_RIGHT_BACKWARD:{
                    motor_set_right(BACKWARD, data);
                } break;
                default:{
                    asm("nop"::);
                }
            }
            HasCommand = 0;
        }
    }
#endif
}


