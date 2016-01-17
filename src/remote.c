
#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "common.h"
#include "remote_lcd.h"
#include "remote_terminal.h"

/**
 * Remote controller code.
 *
 * The purpuse of this code is to control the user input on the
 * control board, and transmit the data to the robot.
 *
 * Hardware configuration:
 * PB0: Button0
 * PD7: Button1
 * PD6: Button2
 * PD5: Button3
 * PD0-1: UART (XBee radio)
 * PC6: Reset
 * PC0: Joystick (analog)
 * PC1: Joystick (analog)
 * PC2: Joystick (button)
 * PB5-3: SPI (ISP connection, Display)
 * LCD:
 * PB1: SCE (Chip select)
 * PB2: RST (Reset)
 * PB7: D/C (Mode select)
 * PB5-3: SPI
 */

enum Buttons {
    BUTTON_UP = 0x1,
    BUTTON_DOWN = 0x2,
    BUTTON_LEFT = 0x4,
    BUTTON_RIGHT = 0x8
};

void init_buttons() {
    clear_bit(DDRB, 0);
    set_bit(PORTB, 0);

    uint8_t portd_mask = (1 << PIND7) | (1 << PIND6) | (1 << PIND5);
    DDRD &= ~portd_mask;
    PORTD |= portd_mask;
}

uint8_t get_buttons() {

    uint8_t state = 0;

    if (bit_set(PINB, 0)) {
        state |= BUTTON_UP;
    }
    if (bit_set(PIND, 7)) {
        state |= BUTTON_DOWN;
    }
    if (bit_set(PIND, 6)) {
        state |= BUTTON_LEFT;
    }
    if (bit_set(PIND, 5)) {
        state |= BUTTON_RIGHT;
    }

    return state;
}

uint8_t pressed(uint8_t button, uint8_t buttons_state, uint8_t buttons_prev_state) {
    if (buttons_state & button && !(buttons_prev_state & button)) {
        return 1;
    }
    return 0;
}

typedef struct {
    uint16_t x;
    uint16_t y;
    bool sw;
} JoyPosition;

static JoyPosition joystick;

ISR(ADC_vect) {
    if (ADMUX & (1 << MUX0)) { // ADC Channel 1
        ADMUX &= ~(1 << MUX0);
        joystick.x = ADC;
    } else { // ADC Channel 0
        ADMUX |= (1 << MUX0);
        joystick.y = ADC;
    }
}

void adc_init() {
    // PC0: Joystick (analog)
    // PC1: Joystick (analog)
    // PC2: Joystick (button)
    DDRC &= ~((1 << PINC2) | (1 << PINC1) | (1 << PINC0));
    PORTC &= ~((1 << PINC2) | (1 << PINC1) | (1 << PINC0));
    
    ADMUX = (0 << ADLAR) | (1 << REFS0) | (1 << MUX0);
    ADCSRA = (1 << ADATE) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1);
    ADCSRB = 0;

    ADCSRA |= (1 << ADEN);
    ADCSRA |= (1 << ADSC);
}

void joystick_init(void) {
    DDRC |= (1 << PINC2);
    PORTC |= (1 << PINC2);
}

void joystick_update(void) {
    if (PINC & (1 << PINC2)) {
        joystick.sw = 0;
    } else {
        joystick.sw = 1;
    }
}

int main(void) {
    init_uart(9600, F_CPU);
    init_buttons();
    lcd_init();
    terminal_init();
    adc_init();
    joystick_init();
    
    // Init leds
    DDRB |= (1 << PINB6);
    clear_bit(PORTB, PINB6);

    sei();
    
    // No interrupts for now.
    // set_bit(PCICR, PCIE0);
    // set_bit(PCMSK0, PCINT0);

    uint8_t buttons = 0xFF;
    uint8_t buttons_prev = 0xFF;

    while (1) 
    {
        buttons = get_buttons();
        joystick_update();

        if (joystick.sw) {
            printf("Joystick pressed!");
        }

        // printf("Getting ADC...\n");
        char buffer[32];
        sprintf(buffer, "%u,%u\n", joystick.x, joystick.y);
        printf(buffer);
        
        if (pressed(BUTTON_UP, buttons, buttons_prev)) {
            printf("BUTTON_UP\n");
            uart_send(RADIO_CMD_FORWARD);
        }
        if (pressed(BUTTON_DOWN, buttons, buttons_prev)) {
            printf("BUTTON_DOWN\n");
            uart_send(RADIO_CMD_BACKWARD);
        }
        if (pressed(BUTTON_LEFT, buttons, buttons_prev)) {
            printf("BUTTON_LEFT\n");
            uart_send(RADIO_CMD_TURN_LEFT);
        }
        if (pressed(BUTTON_RIGHT, buttons, buttons_prev)) {
            printf("BUTTON_RIGHT\n");
            uart_send(RADIO_CMD_TURN_RIGHT);
        }

        buttons_prev = buttons;
    }
}


