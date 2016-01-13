
#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>

#include "common.h"

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

int main(void) {
    init_uart(9600, F_CPU);
    
    init_buttons();
    
    // No interrupts for now.
    // set_bit(PCICR, PCIE0);
    // set_bit(PCMSK0, PCINT0);

    uint8_t buttons = 0;
    uint8_t buttons_prev = 0;

    while (1) 
    {
        buttons = get_buttons();
        // TODO: Probably need to do some smoothing on the button input.
        
        if (pressed(BUTTON_UP, buttons, buttons_prev)) {
            uart_send(RADIO_CMD_FORWARD);
        }
        if (pressed(BUTTON_DOWN, buttons, buttons_prev)) {
            uart_send(RADIO_CMD_STOP);
        }
        if (pressed(BUTTON_LEFT, buttons, buttons_prev)) {
            uart_send(RADIO_CMD_TURN_LEFT);
        }
        if (pressed(BUTTON_RIGHT, buttons, buttons_prev)) {
            uart_send(RADIO_CMD_TURN_RIGHT);
        }

        buttons_prev = buttons;
        _delay_ms(100);
    }
}


